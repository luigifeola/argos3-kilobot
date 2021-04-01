#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: simple_experiment.sh (from src folder) <base_config_dir> <base_config_file_name>"
    exit 11
fi

wdir=`pwd`
base_config=$1$2
if [ ! -e $base_config ]; then
    base_config=$wdir/$1/$2
    if [ ! -e $base_config ]; then
        echo "Error: missing configuration file '$base_config'" 1>&2
        exit 1
    fi
fi

res_dir=$wdir/"results/homogeneus_red_exp"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
else
    echo "Error: directory '$res_dir' already exists" 
    exit 1
fi

base_dir=`dirname $base_config`
echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

numrobots="48"
reactivation_timer="60"
hard_tasks="8"
timeout="5 60 120"

#################################
# experiment_length is in seconds
#################################
experiment_length="1800"
date_time=`date "+%Y-%m-%d"`
RUNS=2

for par1 in $timeout; do
    param_dir=$res_dir/$date_time"_robots#"$numrobots"_"$experiment_length
    if [[ ! -e $param_dir ]]; then
        cmake -E make_directory $param_dir
    fi

    for it in $(seq 1 $RUNS); do

        config=`printf 'config_nrob%d_timeout%03d_seed%03d.argos' $nrob $par1 $it`
        echo config $config
        cp $base_config $config
        sed -i "s|__NUMROBOTS__|$nrob|g" $config
        sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
        sed -i "s|__SEED__|$it|g" $config
        sed -i "s|__HARDTASKS__|$hard_tasks|g" $config
        sed -i "s|__TIMEOUT__|$par1|g" $config
        
        output_file="seed#${it}_time_results.tsv"
        sed -i "s|__OUTPUT__|$output_file|g" $config

        positions_file="seed#${it}_position.tsv"
        sed -i "s|__POSOUTPUT__|$positions_file|g" $config

        
        echo "Running next configuration Robots $nrob TIMEOUT $par1"
        echo "argos3 -c $1$config"
        argos3 -c './'$config
    mv $output_file $param_dir && mv $positions_file $param_dir
    done
done

rm *.argos

#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/bash_run/homogeneus_red_exp.sh /src/examples/experiments/bash_run dhtfs_experiment.argos dhtfc_experiment.argos

if [ "$#" -ne 3 ]; then
    echo "Usage: simple_experiment.sh (from src folder) <base_config_dir> <base_config_file_name_server> <base_config_file_name_client>"
    exit 11
fi

wdir=`pwd`
base_config_s=.$1/$2
echo "base_config_s:" $base_config_s
if [ ! -e $base_config_s ]; then
    base_config_s=$wdir$1/$2
    if [ ! -e $base_config_s ]; then
        echo "Error: missing configuration file '$base_config_s'" 1>&2
        exit 1
    fi
fi

base_config_c=.$1/$3
if [ ! -e $base_config_c ]; then
    base_config_c=$wdir$1/$3
    if [ ! -e $base_config_c ]; then
        echo "Error: missing configuration file '$base_config_c'" 1>&2
        exit 1
    fi
fi

res_dir=$wdir/"results/homogeneus_red_exp"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
# else
#     echo "Error: directory '$res_dir' already exists" 
#     exit 1
fi



base_dir=`dirname $base_config_s`
# echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

numrobots="48"
reactivation_timer="60"
hard_tasks="8"
timeout="5 60 120"
mixed="false"

###################################
# experiment_length is in seconds #
###################################
experiment_length="1800"
date_time=`date "+%Y-%m-%d"`
RUNS=100

# echo 1 $1
# configs=`printf 'configs_nrob%d_timeout%03d_seed%03d.argos' $numrobots $timeout $hard_tasks`
# echo configs $configs
# execute="$1/$2"
# echo full $execute

for par1 in $timeout; do
    param_dir=$res_dir/$date_time"_robots#"$numrobots"_timeout#"$par1"_"$experiment_length"seconds"
    if [[ ! -e $param_dir ]]; then
        cmake -E make_directory $param_dir
    fi

    for it in $(seq 1 $RUNS); do

        #server config
        configs=`printf 'configs_nrob%d_timeout%03d_seed%03d.argos' $numrobots $par1 $it`
        echo configs $configs
        cp $base_config_s $configs
        sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $configs
        sed -i "s|__SEED__|$it|g" $configs
        sed -i "s|__NUMROBOTS__|$numrobots|g" $configs
        sed -i "s|__HARDTASKS__|$hard_tasks|g" $configs
        sed -i "s|__MIXED__|$mixed|g" $configs
        sed -i "s|__TIMEOUT__|$par1|g" $configs
        
        output_file="seed#${it}_completed_taskLOG_server.tsv"
        sed -i "s|__OUTPUT__|$output_file|g" $configs

        robot_positions_file="seed#${it}_kiloLOG_server.tsv"
        sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $configs
        
        area_positions_file="seed#${it}_areaLOG_server.tsv"
        sed -i "s|__AREAPOSOUTPUT__|$area_positions_file|g" $configs

        #client config
        configc=`printf 'configc_nrob%d_timeout%03d_seed%03d.argos' $numrobots $par1 $it`
        echo configc $configc
        cp $base_config_c $configc
        sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $configc
        sed -i "s|__SEED__|$it|g" $configc
        sed -i "s|__NUMROBOTS__|$numrobots|g" $configc
        sed -i "s|__HARDTASKS__|$hard_tasks|g" $configc
        sed -i "s|__TIMEOUT__|$par1|g" $configc
        
        output_file="seed#${it}_completed_taskLOG_client.tsv"
        sed -i "s|__OUTPUT__|$output_file|g" $configc

        robot_positions_file="seed#${it}_kiloLOG_client.tsv"
        sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $configc
        
        area_positions_file="seed#${it}_areaLOG_client.tsv"
        sed -i "s|__AREAPOSOUTPUT__|$area_positions_file|g" $configc

        
        
        echo "Running next configuration seed $it Robots $numrobots TIMEOUT $par1"
        echo server $configs
        echo client $configc
        # executec="$1/$configc"
        xterm -title "server-$it" -e "argos3 -c $configs" &
        xterm -title "client-$it" -e "argos3 -c $configc"
        
        mv *.tsv $param_dir
    done
done
rm *.argos
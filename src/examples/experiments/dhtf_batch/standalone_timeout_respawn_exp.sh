#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/dhtf_batch/standalone_timeout_respawn_exp.sh /src/examples/experiments/dhtf_batch dhtf_experiment.argos

if [ "$#" -ne 2 ]; then
    echo "Usage: simple_experiment.sh (from src folder) <base_config_dir> <base_config_file_name>"
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

res_dir=$wdir/"results_dhtf/standalone_newWalkParam"
if [[ ! -e $res_dir ]]; then
    cmake -E make_directory $res_dir
# else
#     echo "Error: directory '$res_dir' already exists" 
#     exit 1
fi



base_dir=`dirname $base_config_s`
# echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

numrobots="24"
reactivation_timer="30"
desired_num_of_areas="16"
hard_tasks="8"
soft_requirement="2"
hard_requirement="4"
timeout="1 3 6 12 18 24 30 36 42 48 54 60"
region_division="false"
adaptive_walk="false"
# timeout="1 2 3 6 12 18 24 30 36 42 48 54 60 90 180"

###################################
# experiment_length is in seconds #
###################################
date_time=`date "+%Y-%m-%d"`
# experiment_length="300"
# RUNS=1
experiment_length="1800"
RUNS=100

# echo 1 $1
# configs=`printf 'configs_nrob%d_timeout%03d_seed%03d.argos' $numrobots $timeout $hard_tasks`
# echo configs $configs
# execute="$1/$2"
# echo full $execute

for par1 in $timeout; do
    for par2 in $reactivation_timer; do
        param_dir=$res_dir/"Mixed_Persistent_"$date_time"_robots#"$numrobots"_timeout#"$par1"_respawn#"$par2"_NumAreas#"$desired_num_of_areas"_redAreas#"$hard_tasks"_"$experiment_length"#seconds"
        
        #########################################################
        # #debug
        # experiment_length="900"
        # param_dir=$res_dir/"DEBUG_standalone_"$date_time"_timeout#"$par1
        # RUNS=1
        #########################################################

        if [[ ! -e $param_dir ]]; then
            cmake -E make_directory $param_dir
        fi

        for it in $(seq 1 $RUNS); do
            
            configs=`printf 'configs_nrob%d_timeout%03d_seed%03d.argos' $numrobots $par1 $it`
            # echo configs $configs
            cp $base_config_s $configs
            sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $configs
            sed -i "s|__SEED__|$it|g" $configs
            sed -i "s|__NUMROBOTS__|$numrobots|g" $configs
            sed -i "s|__DESIREDTASKS__|$desired_num_of_areas|g" $configs
            sed -i "s|__HARDTASKS__|$hard_tasks|g" $configs
            sed -i "s|__TIMEOUT__|$par1|g" $configs
            sed -i "s|__REACTIVATIONTIMER__|$par2|g" $configs
            sed -i "s|__SOFTREQUIREMENT__|$soft_requirement|g" $configs
            sed -i "s|__HARDREQUIREMENT__|$hard_requirement|g" $configs
            sed -i "s|__REGIONS__|$region_division|g" $configs
            sed -i "s|__ADAPTIVE__|$adaptive_walk|g" $configs
            
            timeout_file="seed#${it}_elapsed_timeoutLOG.tsv"
            sed -i "s|__TIMEOUTOUTPUT__|$timeout_file|g" $configs

            output_file="seed#${it}_completed_taskLOG.tsv"
            sed -i "s|__OUTPUT__|$output_file|g" $configs

            robot_positions_file="seed#${it}_kiloLOG.tsv"
            sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $configs
            
            area_positions_file="seed#${it}_areaLOG.tsv"
            sed -i "s|__AREAPOSOUTPUT__|$area_positions_file|g" $configs


            
            
            echo "Running next configuration seed $it Robots $numrobots TIMEOUT $par1 RESPAWN $par2"
            echo "argos3 -c $1$configs"
            argos3 -c './'$configs
            
            mv *.tsv $param_dir
            rm *.argos
        done

    done
done
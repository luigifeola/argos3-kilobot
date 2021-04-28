#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/bash_run/random_exp.sh /src/examples/experiments/bash_run dhtfs_experiment.argos dhtfc_experiment.argos

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

res_dir=$wdir/"results/random_exp"
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
reactivation_timer="60"
hard_tasks="4"
timeout="1"
# red_timeout="15 18 21 24 27 30"
# blue_timeout="15 18 21 24 27 30"
red_timeout="30"
blue_timeout="24 27 30"
augmented_knowledge="false"

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

for red_t in $red_timeout; do
    for blue_t in $blue_timeout; do

        param_dir=$res_dir/"random_"$date_time"_robots#"$numrobots"_timeoutR#"$red_t"_timeoutB#"$blue_t"_"$experiment_length"seconds"
        
        
        #########################################################
        # #debug
        # experiment_length="900"
        # param_dir=$res_dir/"DEBUG_random_"$date_time"_timeout#"$red_t
        # RUNS=1
        #########################################################
        
        if [[ ! -e $param_dir ]]; then
            cmake -E make_directory $param_dir
        fi

        for it in $(seq 1 $RUNS); do
            
            seedc=$(($it + 200))

            #server config
            configs=`printf 'configs_nrob%d_timeoutRED%03d_timeoutBLUE%03d_seed%03d.argos' $numrobots $red_t $blue_t $it`
            # echo configs $configs
            cp $base_config_s $configs
            sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $configs
            sed -i "s|__SEED__|$it|g" $configs
            sed -i "s|__NUMROBOTS__|$numrobots|g" $configs
            sed -i "s|__HARDTASKS__|$hard_tasks|g" $configs
            sed -i "s|__RR__|$red_t|g" $configs
            sed -i "s|__BB__|$blue_t|g" $configs
            
            output_file="seed#${it}_completed_taskLOG_server.tsv"
            sed -i "s|__OUTPUT__|$output_file|g" $configs

            robot_positions_file="seed#${it}_kiloLOG_server.tsv"
            sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $configs
            
            area_positions_file="seed#${it}_areaLOG_server.tsv"
            sed -i "s|__AREAPOSOUTPUT__|$area_positions_file|g" $configs

            #client config
            configc=`printf 'configc_nrob%d_timeoutRED%03d_timeoutBLUE%03d_seed%03d.argos' $numrobots $red_t $blue_t $it`
            # echo configc $configc
            cp $base_config_c $configc
            sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $configc
            sed -i "s|__SEED__|$seedc|g" $configc
            sed -i "s|__NUMROBOTS__|$numrobots|g" $configc
            sed -i "s|__RR__|$red_t|g" $configc
            sed -i "s|__BB__|$blue_t|g" $configc
            
            output_file="seed#${it}_completed_taskLOG_client.tsv"
            sed -i "s|__OUTPUT__|$output_file|g" $configc

            robot_positions_file="seed#${it}_kiloLOG_client.tsv"
            sed -i "s|__ROBPOSOUTPUT__|$robot_positions_file|g" $configc
            
            area_positions_file="seed#${it}_areaLOG_client.tsv"
            sed -i "s|__AREAPOSOUTPUT__|$area_positions_file|g" $configc

            
            
            echo "Running next configuration seed $it Robots $numrobots timeotRED $red_t timeotBLUE $blue_t"
            # echo server $configs
            # echo client $configc
            # executec="$1/$configc"
            xterm -title "server-$it--timeR-$red_t--timeB-$blue_t" -e "sleep 0.1; argos3 -c $configs; sleep 0.1" &
            xterm -title "client-$seedc--timeR-$red_t--timeB-$blue_t" -e "sleep 0.1; argos3 -c $configc; sleep 0.1"
            sleep 1
            
            mv *.tsv $param_dir
            rm *.argos
        done
    done
done
#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: run_experiments.sh <base_config_dir> <base_config_file>"
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

base_dir=`dirname $base_config`
echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

crw="0.3"
levy="1"

RUNS=2

for par1 in $levy; do
    for par2 in $crw; do
        for it in $(seq 1 $RUNS); do
            filename=`printf 'config_levy%02d_crw%03d_seed%03d.argos' $par1 $par2 $it`
            config=`printf 'config_levy%02d_crw%03d_seed%03d.argos' $par1 $par2 $it`
            cp $base_config $config
            echo arg1 $1
            echo arg1_config $1$config
            sed -i "s|__SEED__|$it|g" $config
            sed -i "s|__CRW__|$par2|g" $config
            sed -i "s|__LEVY__|$par1|g" $config
            output_file=$par1"_"$par2"_"$it
            sed -i "s|__OUTPUT__|$output_file|g" $config
            echo "Running next configuration LEVY $par1 CRW $par2"
            echo "argos3 -c $1$config"
            argos3 -c './'$config
        done
    done
done

rm *.argos


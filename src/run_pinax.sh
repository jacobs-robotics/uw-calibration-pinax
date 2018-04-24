#!/bin/bash

#Read the calibration params file
params_file='/root/input_data/calibration_params.txt'
params[0]='start'
cc=0

while IFS='' read -r line || [[ -n "$line" ]]; do
    echo "Text read from file: $line"
        let c=c+1
        params[$c]="$(cut -d' ' -f2 <<<$line)"
done < "$params_file"

cd /root/camodocal/build/bin/
/bin/bash -c "./stereo_calib -w ${params[2]} -h ${params[3]} -s ${params[4]} --camera-model ${params[5]} -v -i /root/input_data/raw_imgs/ -o /root/output_data/calibration_files/"
chmod a+rw /root/output_data/calibration_files/*


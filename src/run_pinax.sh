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

cd /root/pinax/src
nohup /opt/ros/kinetic/bin/roscore &
sleep 5
/bin/bash -c "rosrun defraction_map_finder stereo_defraction_map_finder -i /root/output_data/calibration_files -o /root/output_data/pinax_maps --refraction-d0 ${params[8]} --refraction-d0-offset ${params[9]} --refraction-d1 ${params[10]} --refraction-n-glass ${params[11]} --refraction-n-water ${params[12]} --rectified-f ${params[13]} --rectified-width ${params[14]} --rectified-height ${params[15]} --rectified-cx ${params[16]} --rectified-cy ${params[17]}"
chmod a+rw /root/output_data/pinax_maps/*

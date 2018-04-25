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

if [ ${params[2]} = "true" ] || [ ${params[2]} = "True" ]; then  
	cd /root/camodocal/build/bin/
	/bin/bash -c "./stereo_calib -w ${params[3]} -h ${params[4]} -s ${params[5]} --camera-model ${params[6]} -v -i /root/input_data/raw_imgs/ -o /root/output_data/calibration_files/"
	chmod a+rw /root/output_data/calibration_files/*
fi

if [ ${params[9]} = "true" ] || [ ${params[9]} = "True" ]; then
	cd /root/pinax/src
	nohup /opt/ros/kinetic/bin/roscore &
	sleep 5
	/bin/bash -c "rosrun defraction_map_finder stereo_defraction_map_finder -i /root/output_data/calibration_files -o /root/output_data/pinax_maps --refraction-d0 ${params[10]} --refraction-d0-offset ${params[11]} --refraction-d1 ${params[12]} --refraction-n-glass ${params[13]} --refraction-n-water ${params[14]} --rectified-f ${params[15]} --rectified-width ${params[16]} --rectified-height ${params[17]} --rectified-cx ${params[18]} --rectified-cy ${params[19]}"
	chmod a+rw /root/output_data/pinax_maps/*
fi

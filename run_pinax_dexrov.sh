# TODO run CamOdoCal calibration

# run PinAx refraction correction
mkdir -p data/maps/DexROVtrialsCalib
rosrun defraction_map_finder stereo_defraction_map_finder \
	-i data/calibration/DexROVtrialsCalib \
	-o data/maps/DexROVtrialsCalib \
	--refraction-d0 0.0028 --refraction-d0-offset 0.00165 \
	--refraction-d1 0.02 \
	--refraction-n-glass 1.7751 --refraction-n-water 1.34 \
	--rectified-f 580 \
	--rectified-width 688 \
	--rectified-height 516 \
	--rectified-cx 344 \
	--rectified-cy 258 \
	--show

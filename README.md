# Underwater Camera Calibration (CamOdoCal+PinAx)

Docker container with the [CamOdoCal](https://github.com/hengli/camodocal) camera calibration software and [PinAx](https://github.com/tomluc/Pinax-camera-model) model for underwater cameras with flat panels.
The container automatically performs camera intrinsic+extrinsic calibration from images colleceted in-air, and also provides underwater rectification maps.

## Getting Started


### Prerequisites

In order to build and run the docker container, the following is needed:

* [Docker](https://docs.docker.com/install/) >= 1.12  
* [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker) (Note that you need NVDIA drivers)

### Installing

After cloning the repository, you only need to run the next command (UNIX systems):

```
./docker_helper.sh build
```

If you are using a Windows plaform, run the following command:

```
docker build uw-calibration-pinax:1.0 .
```

## Running the tests

* After the container is built; the images to be used for calibration have to placed under the *input_data/raw_imgs/* folder.
* The parameters in *input_data/calibration_params.txt* have to be changed according to the calibration pattern used (checkerboard)
and the design of the underwater housing used (thickness of the glass, camera position behing the glass, etc). Please check the 
[CamOdoCal](https://github.com/hengli/camodocal) and [PinAx](https://github.com/tomluc/Pinax-camera-model) links for more information, a brief summary is given below:

```
*CamOdoCal parameters*
x_corners -- Checkerboard/calibration pattern number of inner corners along the X axis
y_corners -- Checkerboard/calibration pattern number of inner corners along the Y axis
square_size -- Checkerboard/calibration pattern square size in *mm*
camera_model -- Camera model used to compute intrinsic and extrinsic paramaters (pinhole,kannala-brandt,mei)

*pinax params*
d_cam2panel -- Distance from the glass panel to the camera in *mm*
d_virtualcam -- Distance from the glass panel to the virtual camera used by PinAx *mm*
glass_thickness -- The glass thickness of the underwater camera housing
n_glass -- Refraction index of the glass
n_water -- Refraction index of the water
focal_length -- Focal length
im_width -- Rectified image width (can be scaled from the original dimensions)
im_height -- Rectified image height (can be scaled from the original dimensions)
cx -- Center of the rectified image in X-axis
cy -- Center of the rectified image in Y-axis
```

## References

In case of using this software for research activities, please cite the following papers:

```
- Lionel Heng, Paul Furgale, and Marc Pollefeys,
	Leveraging Image-based Localization for Infrastructure-based Calibration of a Multi-camera Rig,
    Journal of Field Robotics (JFR), 2015.
- Tomasz Łuczyński, Max Pfingsthorn, Andreas Birk,
	The Pinax-model for accurate and efficient refraction correction of underwater cameras in flat-pane housings,
	Ocean Engineering, Volume 133,2017.
```


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <string>
#include <jir_refractive_image_geometry/refracted_pinhole_camera_model.hpp>
#include "defraction_map_finder/MapFinder.hpp"
#include "defraction_map_finder/CameraFactory.h"
#include <Eigen/Geometry>

#include <boost/program_options.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;

using namespace camodocal;

struct options {
	// read from commandline
	string left_intrinsics_fn;
	string right_intrinsics_fn;
	string extrinsics_fn;

	// refraction extrinsics
	double d0;
	double d0off;
	double d_1;
	double n_g;
	double n_w;

	// output options
	double output_focal_length;
	int output_image_height;
	int output_image_width;
	double output_cx;
	double output_cy;

	// output filenames
	string output_left_map_fn;
	string output_right_map_fn;


	bool show_masks;

	options() { // defaults
		left_intrinsics_fn = "camera_left.yaml";
		right_intrinsics_fn = "camera_right.yaml";
		extrinsics_fn = "extrinsics.yaml";

		d0=0.0014;
		d0off=0.0005;
		d_1=0.01;
		n_g=1.5;
		n_w=1.34;

		output_focal_length = -1;
		output_image_width = -1;
		output_image_height = -1;

		output_left_map_fn = "left_rect_map.yaml.gz";
		output_right_map_fn = "right_rect_map.yaml.gz";

		show_masks = false;
	}
};

bool read_options(int argc, char** argv, options& op);

bool read_extrinsics(const std::string& fn, Eigen::Vector3d& tr, Eigen::Matrix3d& R);

int main(int argc, char** argv)
{
	options op;
	bool ok = read_options(argc, argv, op);

	if(!ok) {
		std::cerr << "error reading options!" << std::endl;
		return 1;
	}

	cout << "going to read extrinsics:" << endl << op.extrinsics_fn << endl;
	
	//**** read extrinsics ****
	Vector3d tr; Matrix3d R;
	ok = read_extrinsics(op.extrinsics_fn, tr, R);

	if(!ok) {
		std::cerr << "error reading extrinsics" << std::endl;
		return 1;
	}
	
	camodocal::CameraPtr left_cam = CameraFactory::instance()->generateCameraFromYamlFile(op.left_intrinsics_fn);
	camodocal::CameraPtr right_cam = CameraFactory::instance()->generateCameraFromYamlFile(op.right_intrinsics_fn);

	if( !left_cam || !right_cam ) {
		std::cerr << "error reading intrinsics" << std::endl;
		return 1;
	}

	if( op.output_image_width < 0 && op.output_image_height < 0 ) {
		// take values from cam
		op.output_image_height = right_cam->imageHeight();
		op.output_image_width = right_cam->imageWidth();
	}
	if( op.output_cx <0 ) {
		op.output_cx = op.output_image_width/2.0;
	}
	if( op.output_cy <0 ) {
		op.output_cy = op.output_image_height/2.0;
	}

	if( op.output_focal_length < 0 ) {
		std::cerr << "error: rectified focal length must be set!" << std::endl;
		return 1;
	}

	// compute rectification
	Vector3d n_cam = ( .5 * ( Vector3d::UnitZ() + R * Vector3d::UnitZ() ) ).normalized();

	Vector3d x = (-tr).normalized();
    Vector3d y = n_cam.cross(x);
    Vector3d z = x.cross(y);
    
    Matrix3d R_r;
    R_r.col(0) = x;
    R_r.col(1) = y;
    R_r.col(2) = z;
	
    Matrix3d R_l = R_r * R.transpose();

    double baseline = tr.norm();

    cout << "refraction intrinsics:" << endl;
    cout << "d0        : " << op.d0 << endl;
    cout << "d0 offset : " << op.d0off << endl;
    cout << "d1        : " << op.d_1 << endl;
    cout << "n glass   : " << op.n_g << endl;
    cout << "n water   : " << op.n_w << endl;
    
    cout << "rectified intrinsics:" << endl;
    cout << "f         : " << op.output_focal_length << endl;
    cout << "cx        : " << op.output_cx << endl;
    cout << "cy        : " << op.output_cy << endl;
    cout << "w         : " << op.output_image_width << endl;
    cout << "h         : " << op.output_image_height << endl;
	
	cout << "rectified extrinsics:" << endl;
	cout << "baseline  : " << baseline << endl;
	
	//**** create camera info ****
	sensor_msgs::CameraInfo info;
	
	info.height=op.output_image_height;
	info.width=op.output_image_width;
	
	info.K[0]=op.output_focal_length;
	info.K[2]=op.output_cx;
	
	info.K[4]=op.output_focal_length;
	info.K[5]=op.output_cy;
	
	info.K[8]=1;
	
	info.R[0] = info.R[4] = info.R[8] = 1.0;

	info.P[0] = op.output_focal_length;
	info.P[2] = op.output_cx;
	info.P[5] = op.output_focal_length;
	info.P[6] = op.output_cy;
	info.P[10] = 1.0;

	info.binning_x = 0;
	info.binning_y = 0;
	


	MapFinder find_left(
			op.output_image_width, 
			op.output_image_height, 
			op.d0, 
			op.d0off, 
			op.d_1, 
			op.n_g, 
			op.n_w,
			info,
			left_cam);

	MapFinder find_right(
			op.output_image_width, 
			op.output_image_height, 
			op.d0, 
			op.d0off, 
			op.d_1, 
			op.n_g, 
			op.n_w,
			info,
			right_cam);


	cout << "generating left points" << endl;
	find_left.generate3Dpoints();
	cout << "generating right points" << endl;
	find_right.generate3Dpoints();
	cout << "points generated" << endl;

	cout << "projecting left points" << endl;
	find_left.projectPoints(R_l);
	cout << "projecting right points" << endl;
	find_right.projectPoints(R_r);
	cout << "points projected" << endl;

	cout << "writing output maps: " << endl;
	cout << op.output_left_map_fn << endl;
	cout << op.output_right_map_fn << endl;

	find_left.saveMaps(op.output_left_map_fn, baseline);
	find_right.saveMaps(op.output_right_map_fn, 0);


	if(op.show_masks) {

		cv::Mat right_mask = find_right.getMask();
		cv::Mat left_mask = find_left.getMask();

		cv::imshow("left mask", left_mask);
		cv::imshow("right mask", right_mask);

		size_t left_black=0, right_black=0;
		for(int r=0; r<right_mask.rows; r++)
		for(int c=0; c<right_mask.cols; c++) {
			if( left_mask.at<uint8_t>(r,c) == 0 ) {
				left_black++;
			}
			if( right_mask.at<uint8_t>(r,c) == 0 ) {
				right_black++;
			}
		}

		cout << "total black pixels: " << endl;
		cout << "left : " << left_black << endl;
		cout << "right: " << right_black << endl;

		cout << "press q to quit" << endl;
		while( cv::waitKey() != 'q' ) {}
	}

	
	return 0;
}

bool read_extrinsics(const std::string& fn, Eigen::Vector3d& tr, Eigen::Matrix3d& R) {
	cv::FileStorage fs(fn, cv::FileStorage::READ);
	if (!fs.isOpened())
    {
        return false;
    }
    cv::FileNode n = fs["transform"];
    
    tr[0] = static_cast<double>(n["t_x"]);
    tr[1] = static_cast<double>(n["t_y"]);
    tr[2] = static_cast<double>(n["t_z"]);
    
    Quaternion<double> q (static_cast<double>(n["q_w"]), static_cast<double>(n["q_x"]),static_cast<double>(n["q_y"]),static_cast<double>(n["q_z"])); 
    R=q.toRotationMatrix();

    return true;
}


bool read_options(int argc, char** argv, options& op) {
	namespace po = boost::program_options;

	string input_dir, output_dir;

	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("input-dir,i", po::value<std::string>(&input_dir)->default_value(""), "input directory where files are located")
		("output-dir,o", po::value<std::string>(&output_dir)->default_value(""), "output directory to put output files")
		("input-left", po::value<std::string>(&op.left_intrinsics_fn)->default_value(op.left_intrinsics_fn), ("input file name for left intrinsics (default: "+op.left_intrinsics_fn+")").c_str())
		("input-right", po::value<std::string>(&op.right_intrinsics_fn)->default_value(op.right_intrinsics_fn), ("input file name for right intrinsics (default: "+op.right_intrinsics_fn+")").c_str())
		("input-extrinsics", po::value<std::string>(&op.extrinsics_fn)->default_value(op.extrinsics_fn), ("input file name for extrinsics (default: "+op.extrinsics_fn+")").c_str())
		("output-left", po::value<std::string>(&op.output_left_map_fn)->default_value(op.output_left_map_fn), ("set output file name for left camera maps (default: "+op.output_left_map_fn+")").c_str())
		("output-right", po::value<std::string>(&op.output_right_map_fn)->default_value(op.output_right_map_fn), ("set output file name for right camera maps (default: "+op.output_right_map_fn+")").c_str())
		("rectified-f", po::value<double>(&op.output_focal_length), "focal length of rectified images")
		("rectified-cx", po::value<double>(&op.output_cx), "image center (x) of rectified images")
		("rectified-cy", po::value<double>(&op.output_cy), "image center (y) of rectified images")
		("rectified-height", po::value<int>(&op.output_image_height), "height of rectified images")
		("rectified-width", po::value<int>(&op.output_image_width), "width of rectified images")
		("refraction-d0", po::value<double>(&op.d0)->default_value(op.d0), "d0 of physical model")
		("refraction-d0-offset", po::value<double>(&op.d0off)->default_value(op.d0off), "d0 offset of physical model")
		("refraction-d1", po::value<double>(&op.d_1)->default_value(op.d_1), "d1 (glass thickness) of physical model")
		("refraction-n-glass", po::value<double>(&op.n_g)->default_value(op.n_g), "index of refraction of used glass")
		("refraction-n-water", po::value<double>(&op.n_w)->default_value(op.n_w), "index of refraction of water")
		("show-masks", po::bool_switch(&op.show_masks), "show resulting masks to spot unmapped image areas")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
	    std::cout << desc << std::endl;
	    return false;
	}

	if( !vm.count("rectified-f")) {
		std::cerr << "rectified-f is required!" << std::endl;
		return false;
	}
	if( !vm.count("rectified-height") && !vm.count("rectified-width")) {
		op.output_image_height = op.output_image_width = -1;
	}
	if( !vm.count("rectified-cx") && !vm.count("rectified-cy") ) {
		op.output_cx = op.output_cy = -1;
	}

	if( vm.count("input-dir") ) {
		string infix = "/";
		if( input_dir[ input_dir.size()-1 ] == '/' ) {
			infix = "";
		}
		op.left_intrinsics_fn = input_dir +infix+ op.left_intrinsics_fn;
		op.right_intrinsics_fn = input_dir +infix+ op.right_intrinsics_fn;
		op.extrinsics_fn = input_dir +infix+ op.extrinsics_fn;
	}

	if( vm.count("output-dir")) {
		string infix = "/";
		if( output_dir[ output_dir.size()-1 ] == '/' ) {
			infix = "";
		}
		op.output_left_map_fn = output_dir +infix+ op.output_left_map_fn;
		op.output_right_map_fn = output_dir +infix+ op.output_right_map_fn;
	}


	return true;
}


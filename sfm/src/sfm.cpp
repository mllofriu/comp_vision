//============================================================================
// Name        : sfm.cpp
// Author      : Martin Llofriu
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

int main() {
	// Camera coordinates
	Vec3d cam_t;
	cam_t[2] = 10;
	Vec3d cam_rot;
	cam_rot[0] = 0;
	// Extrinsec camera
	Matx44d cam_m(1, 0, 0, cam_t[0],
				  0, cos(cam_rot[0]), -sin(cam_rot[0]), cam_t[1],
				 0, sin(cam_rot[0]), cos(cam_rot[0]), cam_t[2],
				 0, 0, 0, 1);
	Matx33d cam_k(.1, 0, 300,
				  0, .1, 300,
				  0, 0, 1);

	// Model Coordinates
	Matx44d model_points(-1, 0, 0 , 1,
						 1, 0, 0, 1,
						 0, -1, 0, 1,
						 0, -.5, 1, 1);
	model_points = model_points.t();


	cout << "Camera " << endl << cam_t << endl << cam_rot << endl << cam_m << endl;
	cout << "Model Points " << endl << model_points << endl;


	// Compute the point image coordinates
	Matx44d p_in_cam = cam_m * model_points;

	cout << "Points in camera frame " << endl << p_in_cam;

	return 0;
}

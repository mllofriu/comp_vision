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
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Mat extrinsec(double rotx, double roty, double rotz, Vec3d t) {
	Matx33d r_x(1, 0, 0, 0, cos(rotx), -sin(rotx), 0, sin(rotx), cos(rotx));
	Matx33d r_y(cos(roty), 0, sin(roty), 0, 1, 0, -sin(roty), 0, cos(roty));
	Matx33d r_z(cos(rotz), -sin(rotz), 0, sin(rotz), cos(rotz), 0, 0, 0, 1);
	Matx33d rot = r_z * r_y * r_x;

	Matx44d cam_m(
			rot(0,0),rot(0,1), rot(0,2),t[0],
			rot(1,0),rot(1,1), rot(1,2),t[1],
			rot(2,0),rot(2,1), rot(2,2),t[2],
			0, 0, 0, 1);
	return (Mat) cam_m;
}

void show_points(Mat img_coords){
	Mat frame(1000, 1000, CV_32FC3);
	frame.setTo(Scalar(0, 0, 0));
	for (int i = 0; i < img_coords.cols; i++) {
		Scalar color;
		switch (i) {
		case 0: color = Scalar(255, 0, 0); break;
		case 1: color = Scalar(0, 255, 0); break;
		case 2: color = Scalar(0, 0, 255); break;
		case 3: color = Scalar(255, 255, 0); break;
		}
		circle(frame, Point(img_coords.at<double>(0,i), img_coords.at<double>(1,i)), 10, color);
		for (int j = 0; j < img_coords.cols; j++){
			if (i != j){
				line(frame, Point(img_coords.at<double>(0,i), img_coords.at<double>(1,i)),
						Point(img_coords.at<double>(0,j), img_coords.at<double>(1,j)),Scalar(255,255,255));
			}
		}
	}

	imshow("Frame", frame);
	waitKey(100);
}

int main() {
	// Extrinsec camera matrix (t from cam to world)
	// Rotation of pi around x and (0,0,10) tranlation
	Matx33d cam_k(1000, 0, 500, 0, 1000, 500, 0, 0, 1);

	// Model Coordinates
	Matx44d model_points(-1, 0, 0, 1, 1, 0, 0, 1, 0, -1, 0, 1, 0, -.5, 1, 1);
	model_points = model_points.t();

	namedWindow("Frame");
	Mat frame(200, 200, CV_32FC3);

	for (float angle = 0; angle < M_PI * 2; angle += M_PI / 64) {
		Mat cam_m = extrinsec(angle, angle, angle, Vec3d(0, 0, 10));

		// Compute the point image coordinates
		Mat p_in_cam(cam_m * (Mat) model_points);

		// Divide by last row
		Mat non_hom = p_in_cam.rowRange(0, 3);
		Mat z_coord = non_hom.row(2);
		Mat tmp;
		repeat(z_coord, 3, 1, tmp);
		non_hom = non_hom / tmp;

		Mat img_coords = ((Mat) cam_k) * non_hom;

		cout << "Points in image coordinates " << endl << img_coords << endl;

		show_points(img_coords);

	}

	return 0;
}

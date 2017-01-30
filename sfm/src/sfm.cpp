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
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

Matx33d get_rot_mat(double rotx, double roty, double rotz)
{
	Matx33d r_x(1, 0, 0, 0, cos(rotx), -sin(rotx), 0, sin(rotx), cos(rotx));
	Matx33d r_y(cos(roty), 0, sin(roty), 0, 1, 0, -sin(roty), 0, cos(roty));
	Matx33d r_z(cos(rotz), -sin(rotz), 0, sin(rotz), cos(rotz), 0, 0, 0, 1);
	Matx33d rot = r_z * r_y * r_x;
	return rot;
}
Mat extrinsec(double rotx, double roty, double rotz, Vec3d t) {
	Matx33d rot = get_rot_mat(rotx, roty, rotz);

	Matx44d cam_m(
			rot(0,0),rot(0,1), rot(0,2),t[0],
			rot(1,0),rot(1,1), rot(1,2),t[1],
			rot(2,0),rot(2,1), rot(2,2),t[2],
			0, 0, 0, 1);
	return (Mat) cam_m;
}

void show_points(Mat img_coords, string frame_name){
	Mat frame(400, 400, CV_32FC3);
	frame.setTo(Scalar(0, 0, 0));
	for (int i = 0; i < img_coords.cols; i++) {
		circle(frame, Point(img_coords.at<double>(0,i), img_coords.at<double>(1,i)), 4, Scalar(255,255,255));
//		for (int j = 0; j < img_coords.cols; j++){
//			if (i != j){
//				line(frame, Point(img_coords.at<double>(0,i), img_coords.at<double>(1,i)),
//						Point(img_coords.at<double>(0,j), img_coords.at<double>(1,j)),Scalar(255,255,255));
//			}
//		}
	}

	imshow(frame_name, frame);
}

Mat get_norm_img_coord(Mat model_points, Mat cam_m)
{
	// Compute the point image coordinates
	Mat p_in_cam(cam_m * (Mat) model_points);

	// Divide by last row
	Mat non_norm = p_in_cam.rowRange(0, 3);
	Mat z_coord = non_norm.row(2);
	Mat tmp;
	repeat(z_coord, 3, 1, tmp);
	Mat norm = non_norm / tmp;

	return norm;
}

Mat get_img_coords(Mat model_points, Mat cam_k, Mat cam_m)
{
	Mat norm_points = get_norm_img_coord(model_points, cam_m);

	Mat img_coords = ((Mat) cam_k) * norm_points;

	cout << "Points in image coordinates " << endl << img_coords << endl;

	return img_coords;
}

int main() {
	// Extrinsec camera matrix (t from cam to world)
	// Rotation of pi around x and (0,0,10) tranlation
	Matx33d cam_k(400, 0, 200, 0, 400, 200, 0, 0, 1);

	// Model Coordinates
	Mat model_points = (Mat_<double>(8,4) <<
			-1, 1, -1, 1,
			1, 1, -1, 1,
			0, -1, -1, 1,
			-1, 1, 1, 1,
			1, 1, 1, 1,
			0, -1, 1, 1,
			0, -.5, 2, 1,
			0, -.5, -2, 1);
	model_points = model_points.t();

	namedWindow("Frame C1");
	namedWindow("Frame C2");

	Vec3d c1_t(1, 4, 0);
	Vec3d c2_t(4, 1, 0);
	double c1_x = 0, c1_y = 0, c1_z = 0;
	double c2_x = 0, c2_y = 0, c2_z = M_PI/4;
	Mat cam1_m = extrinsec(c1_x, c1_y, c1_z, c1_t);
	Mat cam2_m = extrinsec(c2_x, c2_y, c2_z, c2_t);

	Mat img_coords_c1 = get_img_coords((Mat)model_points, (Mat)cam_k, cam1_m);
	Mat img_coords_c2 = get_img_coords((Mat)model_points, (Mat)cam_k, cam2_m);

	cout << "Showing frames" << endl;

	show_points(img_coords_c1, "Frame C1");
	show_points(img_coords_c2, "Frame C2");

	waitKey(0);

	// Get E matrix
	Vec3d rel_t = c2_t - c1_t;
	Matx33d rel_r = get_rot_mat(c2_x, c2_y, c2_z).inv() * get_rot_mat(c1_x, c1_y, c1_z);
	Matx33d cross_t(
			0, -rel_t[2], rel_t[1],
			rel_t[3], 0, -rel_t[0],
			-rel_t[1], rel_t[0], 0);
	cout << "rel_rot * rel_rot.t()" << endl << rel_r * rel_r.t() << endl;
	Matx33d e = cross_t * rel_r;
	// Normalize for scale
	e = e * (1/e(0,0));



	// Get the normalized points
	Mat c1_p = get_norm_img_coord(model_points, cam1_m);
	Mat c2_p = get_norm_img_coord(model_points, cam2_m);

	cout << "Norm points 1" << endl << c1_p << endl;
	cout << "Norm points 2" << endl << c2_p << endl;

	Mat A(c1_p.cols, 9, CV_64F);
	for (int r = 0; r < c1_p.cols; r++){
		for(int i1 = 0; i1 < 3; i1++){
			for (int i2 = 0; i2 < 3; i2++){
				A.at<double>(r, 3 * i1 + i2) = c2_p.at<double>(i2, r) * c1_p.at<double>(i1,r);
			}
		}
	}

	cout << A << endl;

	Mat e_sol;

	Mat w, u, vt;
	SVD::compute(A, w, u, vt);



	Mat v = vt.t();
	v.col(v.cols-1).copyTo(e_sol);
//	cout << w << endl;

//	SVD::solveZ(A,e_sol);
	e_sol = e_sol.reshape(3,3);
	e_sol = e_sol * (1/e_sol.at<double>(0,0));

	cout << e << endl << e_sol << endl;

	return 0;
}

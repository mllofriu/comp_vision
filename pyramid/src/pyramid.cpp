#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;

int main() {
	Mat img = imread("train.jpg");

	namedWindow("Original");
	imshow("Original", img);

	Mat grey;
	cvtColor(img, grey, CV_BGR2GRAY);

	namedWindow("Gray");
	imshow("Gray", grey);

	Mat down;
	pyrDown(grey, down);

	namedWindow("down 2");
	imshow("down 2", down);

	Mat down4x;
	pyrDown(down, down4x);

	namedWindow("down 4");
	imshow("down 4", down4x);

	Mat up;
	pyrUp(down, up);
	namedWindow("back up");
	imshow("back up", up);


	waitKey(0);
}

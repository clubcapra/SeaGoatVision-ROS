#include "opencv2/opencv.hpp"
//#include "opencv2/gpu/gpu.hpp"
#include <fstream>
#include <stdio.h>

#define DOCSTRING "C++ Perspective Calibration"

using namespace cv;


int topLeftX_ = 0;
int topLeftY_ = 0;
int bottomLeftX_ = 0;
int bottomLeftY_ = 0;
int topRightX_ = 0;
int topRightY_ = 0;
int bottomRightX_ = 0;
int bottomRightY_ = 0;

int dx_ = 0;
int dy_ = 0;
int zoom_ = 100;

void init() {
	//rotationX_ = ParameterAsInt("rotationX_", -360, 360, -160);
	//rotationY_ = ParameterAsInt("rotationY_", -360, 360, 0);
	zoom_ = ParameterAsInt("Zoom", 0, 500, 100);
	dx_ = ParameterAsInt("Translation X", -1000, 1000, 0);
	dy_ = ParameterAsInt("Translation Y", -1000, 1000, 0);


    topLeftX_ = ParameterAsInt("Top Left X",-1500,1500,-200);
    topLeftY_ = ParameterAsInt("Top Left TY",-1500,1500,0);
    bottomLeftX_ = ParameterAsInt("Bottom Left X",0,1500,202);
    bottomLeftY_ = ParameterAsInt("Bottom Left Y",0,1500,734);
    topRightX_ = ParameterAsInt("Top Right X",0,1500,1292);
    topRightY_ = ParameterAsInt("Top Right Y",0,1500,1292);
    bottomRightX_ = ParameterAsInt("Bottom Right X",0,1500,1090);
    bottomRightY_ = ParameterAsInt("Bottom Right Y",0,1500,734);
}

cv::Mat perspective(cv::Mat image){

	    const double w = image.cols;
	    const double h = image.rows;

        cv::Point2f src_vertices[4];
        src_vertices[0] = cv::Point(0,0);
        src_vertices[1] = cv::Point(0,h);
        src_vertices[2] = cv::Point(w,0);
        src_vertices[3] = cv::Point(w,h);

        cv::Point2f dst_vertices[4];
        dst_vertices[0] = cv::Point(topLeftX_, topLeftY_);
        dst_vertices[1] = cv::Point(bottomLeftX_, bottomLeftY_);
        dst_vertices[2] = cv::Point(topRightX_, topRightY_);
        dst_vertices[3] = cv::Point(bottomRightX_, bottomRightY_);

        cv::Mat warpMatrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);

        cv::Mat imageWarped;
        cv::warpPerspective(image, imageWarped, warpMatrix, imageWarped.size());

        return imageWarped;
}

cv::Mat transform(cv::Mat image){

	const double w = image.cols;
	const double h = image.rows;

	// Projection 2D -> 3D matrix
	Mat A1 = (Mat_<double>(4,3) <<
		1, 0, -w/2,
		0, 1, -h/2,
		0, 0,    0,
		0, 0,    1);

    // The rotation matrices are not used by this filter but are left there if
    // someone wants to use them at some point

	// Rotation matrices around the X axis
	int rotationX_ = 0;
	const double alpha = rotationX_ / 180. * 3.1416 / w;
	Mat RX = (Mat_<double>(4, 4) <<
		1,          0,           0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha),  cos(alpha), 0,
		0,          0,           0, 1);

	// Rotation matrices around the X axis
    int rotationY_ = 0;
	const double beta = rotationY_ / 180. * 3.1416 / h;
	Mat RY = (Mat_<double>(4, 4) <<
		cos(beta),  0,           sin(beta), 0,
		0,			1,			 0, 0,
		-sin(beta), 0,			 cos(beta), 0,
		0,          0,           0, 1);

	// Translation matrix on the Z axis
	Mat T = (Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, zoom_/100.0f,
		0, 0, 0, 1);

	const double f = 1;
	// Camera Intrisecs matrix 3D -> 2D
	Mat A2 = (Mat_<double>(3,4) <<
		f, 0, w/2, 0,
		0, f, h/2, 0,
		0, 0,   1, 0);

	Mat transfo = A2 * (T * (RX * RY * A1));

	Mat destination;

	warpPerspective(image, destination, transfo, image.size(), INTER_CUBIC | WARP_INVERSE_MAP);

	return destination;
}

void configure() {
	init();
}

cv::Mat execute(cv::Mat image)
{
    // Change perspective with the 4 corners
    Mat m = perspective(image);

    // Transform with the zoom and translation
    return transform(m);

}
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <Eigen/Dense>
#include <vector>

using namespace std;

#define WIDTH 640
#define HEIGHT 480

// Project 3D point onto image (lazy and assuming static camera at Z = -125)
Eigen::Vector2d ProjectOntoImage(Eigen::Vector3f point, double rotation){

	// rotate a bitch
		Eigen::Matrix2f rotMatrix;

		rotMatrix(0) = cos(rotation);
		rotMatrix(1) = -sin(rotation);
		rotMatrix(2) = sin(rotation);
		rotMatrix(3) = cos(rotation);

		//-17 creates a slightly more interesting center of rotation
		Eigen::Vector2f point2D(point.x() - 17, point.z() - 17);

		point2D = rotMatrix * point2D;

	point.x() = point2D.x();
	point.z() = point2D.y() + 125;
	
	Eigen::Vector2d projection;

	projection.x() = point.x() * 525 / (point.z()) + WIDTH/2;
	projection.y() = point.y() * 525 / (point.z()) + HEIGHT/2;

	return projection;
}

// Update the state, lorenz params:
// sigma = x()
// rho = y()
// beta = z()
double UpdateLorenzState(Eigen::Vector3f &point, Eigen::Vector3f lorenz_params){
	double dx = lorenz_params.x() * (point.y() - point.x());
    double dy = point.x() * (lorenz_params.y() - point.z()) - point.y();
    double dz = point.x() * point.y() - lorenz_params.z() * point.z();

	point.x() += 0.01 * dx;
	point.y() += 0.01 * dy;
	point.z() += 0.01 * dz;

	return Eigen::Vector3f(dx, dy, dz).norm();
}

// Render the scene
void Render(std::vector<Eigen::Vector3f> &points, std::vector<double> velocities, IplImage *img){
	static float degree = 0;
	// clear image
	for(int i = 0; i < img->height * img->widthStep; i++){
		img->imageData[i] = 0;
	}

	// draw attractor
	double max = 200;
	int start = std::max<int>(points.size() - 1000, 0);

	Eigen::Vector2d prev_point = ProjectOntoImage(points[start], degree * 180.0 / 3.14159f);
	printf("%d\n", points.size());
	for(int i = start+1; i < points.size() && i < 1250; i++){
		Eigen::Vector2d point = ProjectOntoImage(points[i], degree * 180.0 / 3.14159f);
		
		if(point.x() >= 0 && point.x() < WIDTH && point.y() >= 0 && point.y() < HEIGHT){
			double velocity = (prev_point - point).norm();
			double alpha = std::min<double>(velocities[i] / max, 1.0);
			cvDrawLine(img, cvPoint(prev_point.x(), prev_point.y()), cvPoint(point.x(), point.y()), cvScalar(255 * (1 - alpha), 0, 255 * alpha), 2);
		}
		prev_point = point;
	}

	prev_point = ProjectOntoImage(points[start], degree * 180.0 / 3.14159f);
	prev_point.x() = WIDTH - prev_point.x();
	for(int i = start+1; i < points.size()  && i < 1250; i++){
		Eigen::Vector2d point = ProjectOntoImage(points[i], degree * 180.0 / 3.14159f);
		point.x() = WIDTH - point.x();
		if(point.x() >= 0 && point.x() < WIDTH && point.y() >= 0 && point.y() < HEIGHT){
			double velocity = (prev_point - point).norm();
			double alpha = std::min<double>(velocities[i] / max, 1.0);
			cvDrawLine(img, cvPoint(prev_point.x(), prev_point.y()), cvPoint(point.x(), point.y()), cvScalar(0, 255 * alpha, 255 * (1 - alpha)), 2);
		}
		prev_point = point;
	}
	degree += 0.0001;
}

int main(){
	// initialize image
	IplImage *img = cvCreateImage(cvSize(WIDTH,HEIGHT), IPL_DEPTH_8U, 3);

	std::vector<Eigen::Vector3f> points;
	std::vector<double> velocity;

	Eigen::Vector3f state(0.1, 0.1, 0.1);
	Eigen::Vector3f params(5.0f, 40.0f, 1);

	int frame = 0;
	char fname[1000];

	while(1){
		points.push_back(state);
		velocity.push_back(UpdateLorenzState(state, params));

		Render(points, velocity, img);
		
		cvShowImage("Lorenz", img);
		cvWaitKey(5);
		sprintf(fname, "output/image%04d.jpg", frame);
		if(frame % 6 == 0){
			cvSaveImage(fname, img);
		}
		frame++;
	}
}
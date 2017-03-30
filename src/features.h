/*	Comp5421 Computer Vision project, Spring 2017, Thomas Lew
	Last update: 22 feb 2017
	
	Implements Features Detection

*/



#ifndef features_h
#define features_h

using namespace cv; // Necessary for using -Mat-

#define WIDTH 800
#define HEIGHT 500

typedef struct features
{
	vector<KeyPoint> keypoints;
	Mat descriptors;
}FEATURES;

typedef struct matches
{
	FEATURES features;
	vector<DMatch> matches;
}MATCHES;

typedef struct epipolar_lines
{
	vector<Vec3f> epilines1;
	vector<Vec3f> epilines2;
}EPIPOLAR_LINES;

// Prints an image using normal OpenCV Functions with appropriate size
void opencv_show_image(char* image_name, Mat image_content);

// Features detection in the image. Draws the image and the features on it
FEATURES features_detection(Mat image, char* detector_type);

// Returns the good matches between the features in the images
vector<DMatch> features_matcher(FEATURES features1, FEATURES features2);

// Returns the fundamental matrix from matches between the features
Mat features_fundMat(FEATURES feat1, FEATURES feat2, vector<DMatch> matches);

// Computes and draws the epipolar lines
EPIPOLAR_LINES features_epipolar_lines(FEATURES feat1, FEATURES feat2, 
									   vector<DMatch> matches, Mat fund_mat,
									   Mat image1, Mat image2);

#endif

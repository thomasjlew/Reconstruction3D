/*	----------------------------------------------------------------------------
	Comp5421 Computer Vision project, Spring 2017, Thomas Lew
	Last update: 3 May 2017
	
	Implements Features Detection, Matching and 3D reconstruction.
    ------------------------------------------------------------------------- */


#ifndef features_h
#define features_h

using namespace cv;

#define WIDTH  800
#define HEIGHT 500

/*	--------------------------------------------------------------------------
	Intrinsic parameters of the camera
	See final report for more details										*/
const Mat K =  (Mat_<double>(3,3) << -2759.48,    0,    1520.69,
		 						  	     0,   -2764.16, 1006.81,
		 						  	     0, 	  0, 	   1   );
/*	----------------------------------------------------------------------- */

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

struct my_match
{
	Point2f point1;
	Point2f point2;
	double  distance;
};

//	Structure used in features_twoview_triangulation for structure from motion.
//	Used to scale the depth of the points and the translation between 2 images
typedef struct sfm
{
	/*Mat_<double> Rt01;	//	[R|t] extrinsics matrix between origin to 1st image
	Mat_<double> Rt12;	//	[R|t] extrinsics matrix between 1st to 2nd image*/
	Mat_<double> Rt;
	vector<my_match> vec_matches;	//get_most_precise_matches(feat1, feat2, matches)
	Mat_<double> homog_3d_pts;	//	For depth-checking, should be of size of vec_matches and same order
}SFM;

// Displays an image using usual OpenCV functions with appropriate size
void opencv_show_image(char* image_name, Mat image_content);
/* -------------------------------------------------------------------------- */

// Features detection in the image. Draws the image and the features on it
FEATURES features_detection(Mat image, char* detector_type);
/* -------------------------------------------------------------------------- */

// Returns the good matches between the features in the images
// Precision can be "GOOD" or "VERY_PRECISE"
// Recommendations: Use "GOOD" to get matches for 3d reconstruction
//					Use "VERY_PRECISE" to compute the fundamental matrix
vector<DMatch> features_matcher(FEATURES features1, FEATURES features2, 
								string precision, int num_image = 0);
/* -------------------------------------------------------------------------- */

// Sorts the matches with the most precise ones first
vector<my_match> get_most_precise_matches(FEATURES feat1, FEATURES feat2, 
											vector<DMatch> matches);

// Returns the fundamental matrix from matches between the features
Mat features_fundMat(FEATURES feat1, FEATURES feat2, vector<DMatch> matches);
/* -------------------------------------------------------------------------- */

// Computes and draws the epipolar lines
EPIPOLAR_LINES features_epipolar_lines(FEATURES feat1, FEATURES feat2, 
									   vector<DMatch> matches, Mat fund_mat,
									   Mat image1, Mat image2);
/* -------------------------------------------------------------------------- */


// Single pair of images Triangulation function
void features_triangulation(Mat fund_mat, Mat img1, Mat img2, FEATURES feat1, 
							FEATURES feat2,	vector<DMatch> matches);
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	features_twoview_triangulation funtion.
	For a pair of images, computes the camera matrices and triangulates the 3D
	features.
	Input:	- Fundamental matrix for the pair of images
			- Two images
			- Features and matches
			- sfm structure of previous pair of images
				(Camera matrix to set the origin and rotation of first camera)
	Output:	- sfm structure of this pair of images
					(Rotation and translation between the two stereo images)
			- (Written in a Ply file) Triangulated good features 			 	
----------------------------------------------------------------------------- */
SFM features_twoview_triangulation(Mat fund_mat,
									Mat img1, Mat img2, 
									FEATURES feat1, FEATURES feat2,
									vector<DMatch> matches, SFM sfm_previous);
/* -------------------------------------------------------------------------- */

#endif

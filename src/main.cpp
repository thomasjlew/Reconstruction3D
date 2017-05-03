#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "features.h"

using namespace std;
using namespace cv;

void main_argc_error(){
	cout << "How to use this program:" << endl
		 << "- 1) Test Feature detection:" << endl
		 << "  ./cvision detection ../fountain_dense/urd/0002.png" << endl
		 << "- 2) Test Feature matching:" << endl
		 << "  ./cvision matching image1.png image2.png" << endl
		 << "- 3) Test Epipolar Geometry:" << endl
		 << "  ./cvision geometry image1.png image2.png" << endl
		 << "- 4) Test Two-view Triangulation:" << endl
		 << "  ./cvision triangulation image1.png image2.png" << endl;
		 /*
		 ./bin/cvision sfm ../fountain_dense/urd/0000.png 
		 ../fountain_dense/urd/0001.png ../fountain_dense/urd/0002.png 
		 ../fountain_dense/urd/0003.png ../fountain_dense/urd/0004.png 
		 ../fountain_dense/urd/0005.png ../fountain_dense/urd/0006.png 
		 ../fountain_dense/urd/0007.png ../fountain_dense/urd/0008.png 
		 ../fountain_dense/urd/0009.png 									*/
}

//	Extracts features from an image. The detector type can be changed here.
FEATURES detection(char *im1){//, char* detectorType){
	//cout << "Starting detection of features." << endl;

	Mat image;
    image = imread(im1, CV_LOAD_IMAGE_GRAYSCALE);
    if(!image.data)
    {
        cout <<  "Could not open or find the image " << endl;
        FEATURES problem_features;
        return problem_features;
    }
    /*
    opencv_show_image("Grayscale Source Image", image);
   	namedWindow("Grayscale Source Image", WINDOW_NORMAL); // WINDOW_AUTOSIZE
    imshow("Grayscale Source Image", image);
	resizeWindow("Grayscale Source Image", WIDTH, HEIGHT);
		*/

    //cout << "press a key to continue" << endl;
    waitKey(1);//0);

    char detectorType[] = "SIFT";		// SiftFeatureDetector
    //char detectorType[] = "SURF";		// SurfFeatureDetector
    //char detectorType[] = "FAST";		// FastFeatureDetector
	//char detectorType[] = "ORB";		// OrbFeatureDetector
	// These detector types DO NOT WORK ----------------------
    //char detectorType[] = "MSER"; 	// MserFeatureDetector
    // http://www.ai.mit.edu/courses/6.891/handouts/shi94good.pdf
    // use goodFeaturesToTrack()
    //char detectorType[] = "GFTT";		// GoodFeaturesToTrackDetector
    //char detectorType[] = "HARRIS";	// GFTT with Harris detector enabled
    FEATURES features = features_detection(image, detectorType);

	//	cout << "End of function to detect features." << endl << " " << endl;
	return features;
}

// Extracts features from two images and computes the matches between two images
int matching(char *im1, char *im2){
	cout << "Starting matching of features." << endl;

	// Not optimal: image loaded twice (Also loaded in detection())
	// can be optimized by direct call to features_detector
	Mat image1 = imread(im1, CV_LOAD_IMAGE_GRAYSCALE);
	Mat image2 = imread(im2, CV_LOAD_IMAGE_GRAYSCALE);

	// Image opening already tested in "detection()"
    if(!image1.data || !image2.data)
    {
        cout <<  "Could not open one of the images" << endl ;
        return -1;
    }

    // Feature extraction
    FEATURES features1 = detection(im1); // the number of features is about 4000
    FEATURES features2 = detection(im2);

    // Feature matching. Only draws the good matchs
    vector<DMatch> matches = features_matcher(features1, features2, 
													  "VERY_PRECISE");

    //-- Draw only "good" matches
	Mat img_matches;
	drawMatches(image1, features1.keypoints, image2, features2.keypoints,
	           matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	           vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    namedWindow("Good Matches", WINDOW_NORMAL); // WINDOW_AUTOSIZE
	imshow("Good Matches", img_matches);
	resizeWindow("Good Matches", WIDTH, HEIGHT);

	for( int i = 0; i < (int)matches.size(); i++ )
	{ 
		printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, 
			matches[i].queryIdx, matches[i].trainIdx ); 
	}

	cout << "press a key on the image to continue" << endl;
	waitKey(0);

	cout << "End of function to match features." << endl;
	return 1;
}

int geometry(char *im1, char *im2){
	cout << "                 " << endl;
	cout << "-----------------" << endl;
	cout << "Start of Geometry" << endl;
	cout << "-----------------" << endl;
	cout << "                 " << endl;

	Mat image1 = imread(im1, CV_LOAD_IMAGE_GRAYSCALE);
	Mat image2 = imread(im2, CV_LOAD_IMAGE_GRAYSCALE);

    // Features extraction
    FEATURES feat1 = detection(im1);
    FEATURES feat2 = detection(im2);

    // Feature matching.
    vector<DMatch> matches = features_matcher(feat1, feat2, "GOOD");
    vector<DMatch> precise_matches = features_matcher(feat1, feat2, 
													  "VERY_PRECISE");

    // Compute fundamental matrix
	Mat fundamental_mat = features_fundMat(feat1, feat2, precise_matches);
	cout << "Main: Fundamental matrix computed: " << endl;
	cout << fundamental_mat << endl;

	// Compute epipolar lines
	EPIPOLAR_LINES epilines = features_epipolar_lines(feat1, feat2, matches, 
											fundamental_mat, image1, image2);

	cout << "End of Geometry function." << endl;

	return 1;
}

/*	Triangulation funtion computes the fundamental matrices between a pair
	of images, camera matrices and triangulates the 3D features on the images. 
	Input:	Two images
	Output:	(Computed) Rotation and translation between the two stereo images
			(On terminal) Fundamental matrix between the pair of images
			(Written in a Ply file) Triangulated good features 			 */
int triangulation(char *im1, char *im2){
	cout << "                      " << endl;
	cout << "----------------------" << endl;
	cout << "Start of Triangulation" << endl;
	cout << "----------------------" << endl;
	cout << "                      " << endl;

	Mat image1 = imread(im1, CV_LOAD_IMAGE_GRAYSCALE);
	Mat image2 = imread(im2, CV_LOAD_IMAGE_GRAYSCALE);

    // Features extraction
    FEATURES feat1 = detection(im1);
    FEATURES feat2 = detection(im2);
    cout << "feat1.keypoints.size()" << feat1.keypoints.size() << endl;
    cout << "feat1.descriptors.size()" << feat1.descriptors.size() << endl;

    // Feature matching.
    vector<DMatch> matches = features_matcher(feat1, feat2, "GOOD", 7); 	//	<-------------------------
    vector<DMatch> precise_matches = features_matcher(feat1, feat2, 
													  "VERY_PRECISE", 7);

    // Compute fundamental matrix
	Mat fundamental_mat = features_fundMat(feat1, feat2, precise_matches);
	cout << "Main: Fundamental matrix computed: " << endl;
	cout << fundamental_mat << endl;

	/*	Compute epipolar lines. 
		Result using matches instead of precise_matches is similar			  */
	/*EPIPOLAR_LINES epilines = features_epipolar_lines(feat1, feat2, matches, 
											fundamental_mat, image1, image2); */
	EPIPOLAR_LINES epilines = features_epipolar_lines(feat1, feat2, 
							precise_matches, fundamental_mat, image1, image2);

	//	Start of triangulation functions!
	features_triangulation(fundamental_mat, image1, image2, feat1, feat2, 
						   matches);


	cout << "Triangulation function finished successfully." << endl;
	return 1;
}

/*	Implements structure from motion (SFM) 
	Reconstruct a 3D scene from multiple images 	*/
int sfm(char** images){
	//int number_images = 0;
	char* im0;
	char* im1;
	char* im2;

	//	Extract the two first images
	if(images[0] == NULL || images[1] == NULL || images[2] == NULL)
	{
		cerr << "Problem opening the images for structure_from_motion: "
				 "not enough pictures!" << endl;
		return 0;
	}
	im0 = images[0];
	im1 = images[1];
	im2 = images[2];

	//	Read and extract features in first pair of images
	Mat image0 = imread(im0, CV_LOAD_IMAGE_GRAYSCALE);
	Mat image1 = imread(im1, CV_LOAD_IMAGE_GRAYSCALE);
	Mat image2;
	FEATURES feat0 = detection(im0);
	FEATURES feat1 = detection(im1);
	FEATURES feat2;

	//	Initial extrinsics [R|t] Camera matrix compared to world frame. 
	Mat Rt01 = (Mat_<double>(3,4) <<  1,     0,     0,     0,
								      0,     1,     0,     0,
								      0,     0,     1,     0);
	Mat F;		//	Fundamental matrix

	// Feature matching.
	vector<DMatch> matches_01;
	vector<DMatch> matches_12;
	vector<DMatch> precise_matches_01;
	vector<DMatch> precise_matches_12;

	//	------------------------------------------------------------------------
	//	Triangulation of first pair of images
	//	------------------------------------------------------------------------
	cout << endl << "---------------------------------" << endl << endl;

	//	Feature matching.
	matches_01 = features_matcher(feat0, feat1, "GOOD");
	precise_matches_01 = features_matcher(feat0, feat1, "VERY_PRECISE");

	//	Compute fundamental matrix
	F = features_fundMat(feat0, feat1, precise_matches_01);
	//cout << "Fundamental Matrix n°" << i+1 << ": " << F << endl;

	//	Get precise matches common in 2 image pairs for scale of translation


	//	Compute the fundamental matrix, triangulate the features between two
	//	images and obtain the rotation and translation between two matrices
	//P2 = two_view_geometry(im1, im2, P1);

	//	Triangulation of the extracted features for this image pair
	/*P01 = features_twoview_triangulation(F, P01, image0, image1, 
										 feat0, feat1, matches_01);*/
	SFM sfm_01;
	sfm_01.Rt = Rt01;
	//sfm_01.vec_matches;
	//sfm_01.homog_3d_pts = P01;	//			P01 formerly which is correct
	sfm_01 = features_twoview_triangulation(F, image0, image1, 
										 	feat0, feat1, matches_01, sfm_01);

	cout << endl << "---------------------------------" << endl << endl;
	//	For each consecutive pairs of images
	for(int i=2; im2 != NULL; i++)
	{
		cout << endl << "Start analysing image n°" << i+1 << endl;
		cout << endl << "---------------------------------" << endl << endl;
		//	Read and extract features in second image
		image2 = imread(im2, CV_LOAD_IMAGE_GRAYSCALE);

		//	Features extraction
		feat2 = detection(im2);

		//	Feature matching.
		matches_12 = features_matcher(feat1, feat2, "GOOD", i);
		precise_matches_12 = features_matcher(feat1, feat2, "VERY_PRECISE", i);

		//	Compute fundamental matrix
		F = features_fundMat(feat1, feat2, precise_matches_12);
		//cout << "Fundamental Matrix n°" << i+1 << ": " << F << endl;

		//	Get precise matches common in 2 image pairs for scale of translation


		//	Compute the fundamental matrix, triangulate the features between two
		//	images and obtain the rotation and translation between two matrices
		//P2 = two_view_geometry(im1, im2, P1);

		//	Triangulation of the extracted features for this image pair
		/*Mat P12 = features_twoview_triangulation(F, P01, image1, image2, 
												feat1, feat2, matches_12);*/
		SFM sfm_12 = features_twoview_triangulation(F, image1, image2, 
												feat1, feat2, matches_12, sfm_01);

		//	Compute new rotation and translation compared to first image
		//P01 = P12;
		sfm_01 = sfm_12;

		//	Select the two next images
		im0 = im1;
		im1 = im2;
		im2 = images[i+1];
		feat0 = feat1;
		feat1 = feat2;
		matches_01 = matches_12;
		precise_matches_01 = precise_matches_12;
	}
	return 1;
}

int main(int argc, char *argv[]){
	// no task given to the module
	if(argc < 2){
		main_argc_error();
		return 1;
	}
	
	string task = argv[1];
	if(task == "detection"){
		if(argc != 3){
			main_argc_error();
			return 1;
		}
		detection(argv[2]);
		return 1;
	}
	else if(task == "matching"){
		if(argc != 4){
			main_argc_error();
			return 1;
		}
		return matching(argv[2], argv[3]);
	}
	else if(task == "geometry"){
		if(argc != 4){
			main_argc_error();
			return 1;
		}
		return geometry(argv[2], argv[3]);
	}
	else if(task == "triangulation"){
		if(argc != 4){
			main_argc_error();
			return 1;
		}
		return triangulation(argv[2], argv[3]);
	}
	else if(task == "sfm"){
		if(argc < 4){
			main_argc_error();
			return 1;
		}
		char** image_p = &argv[2];
		return sfm(image_p);
	}
	else{
		main_argc_error();
		return 1;
	}

	/*switch(task){
		case "detection":
			if(argc == 3){
				return detection(argv[2], argv[3]);
				break;
			}
		case "matching":
			if(argc == 4){
				return matching(argv[2], argv[3]);
			}
		case "geometry":
			if(argc == 4){
				return geometry(argv[2], argv[3]);
			}
		case "triangulation":
			if(argc == 4){
				return triangulation(argv[2], argv[3]);
			}
		default:
			main_argc_error();
			return 0;
			break;
	}*/

	return 0;
}


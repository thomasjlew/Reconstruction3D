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
}

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
    //opencv_show_image("Grayscale Source Image", image);
   /* namedWindow("Grayscale Source Image", WINDOW_NORMAL); // WINDOW_AUTOSIZE
    imshow("Grayscale Source Image", image);
	resizeWindow("Grayscale Source Image", WIDTH, HEIGHT);*/

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

	cout << "End of function to detect features." << endl << " " << endl;
	return features;
}

// Functions templates
int matching(char *im1, char *im2){
	cout << "Starting matching of features." << endl;

	// Not optimal: image loaded twice
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
    vector<DMatch> matches = features_matcher(features1, features2);

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

    // Feature matching. Only draws the good matchs
    vector<DMatch> matches = features_matcher(feat1, feat2);

    // Compute fundamental matrix
	Mat fundamental_mat = features_fundMat(feat1, feat2, matches);
	cout << "Main: Fundamental matrix computed: " << endl;
	cout << fundamental_mat << endl;

	// Compute epipolar lines
	EPIPOLAR_LINES epilines = features_epipolar_lines(feat1, feat2, matches, 
											fundamental_mat, image1, image2);

	cout << "End of Geometry function." << endl;
	return 1;
}

int triangulation(char *im1, char *im2){
	cout << "Triangulation hasn't been implemented yet." << endl;
	return 1;
}

int two_view_geometry(char* task, char* im1, char* im2){
	cout << "Two-view Geometry has started with task" << task << endl;
	// Test to open images depending on the task (test im2 or not)

	// Call good functions depending on the task

	cout << task << " finished successfully." << endl;
	return 1;
}

int main(int argc, char *argv[]){
	// no task given to the module
	if(argc < 2){
		main_argc_error();
		return 1;
	}

	// try to use two_view_geometry

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


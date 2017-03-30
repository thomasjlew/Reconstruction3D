#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>  // for initModule_nonfree()
#include <opencv2/calib3d/calib3d.hpp>	// for FM_RANSAC
#include <opencv2/imgproc/imgproc.hpp>	// for cvt

#include "features.h"

using namespace std;
using namespace cv;


//indications for triangulation?
//http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/SfM.html


//	Drawing Constants
#define RADIUS_FEAT_DRAW 10
#define THICKN_FEAT_DRAW 2
#define NB_EPILINES_TO_DRAW 15

void opencv_show_image(char* image_name, Mat image_content){
    namedWindow(image_name, WINDOW_NORMAL);
	imshow(image_name, image_content);
	resizeWindow(image_name, WIDTH, HEIGHT);
}

FEATURES features_detection(Mat image, char* detector_type)
{
	// Features and Descriptors recognised in the image
    FEATURES features;
    //vector<KeyPoint> keypoints;
	//Mat descriptors;
	
	cout << "Features detector type used: " << detector_type << endl;
	// First, select SIFT or SURF detector
    /*SiftFeatureDetector feature_detector;
	feature_detector.detect(image, keypoints);*/
	initModule_nonfree();
	Ptr<FeatureDetector> feature_detector = 
		FeatureDetector::create(detector_type);
    feature_detector->detect(image, features.keypoints);

	// Compute the descriptor for each keypoint.
	Ptr<DescriptorExtractor> featureExtractor = 
		DescriptorExtractor::create(detector_type);
	featureExtractor->compute(image, features.keypoints, features.descriptors);

    // Draw the keypoints (features) on the original image
    Mat image_with_features;
    drawKeypoints(image, features.keypoints, image_with_features);
    opencv_show_image("Image with Features", image_with_features);
    /*namedWindow("Image with Features", WINDOW_AUTOSIZE); //WINDOW_NORMAL
    imshow("Image with Features", image_with_features);*/
    //cout << "press a key to continue" << endl;
    waitKey(1);//0); 

    return features;
}

vector<DMatch> features_matcher(FEATURES feat1, FEATURES feat2)
{
	// Fast Approximate Nearest Neighbor Search Library
	/* see "http://docs.opencv.org/2.4/doc/tutorials/features2d/
	    	feature_flann_matcher/feature_flann_matcher.html"		*/
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(feat1.descriptors, feat2.descriptors, matches);

	//---------------------------------------------------------------
	//-- Quick calculation of max and min distances between keypoints
	double max_dist = 0; double min_dist = 100;
	for(int i = 0; i < feat1.descriptors.rows; i++)
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	
	/*printf("-- Max dist : %f \n", max_dist );
	printf("-- Min dist : %f \n", min_dist );*/

	//------------------------------------------------------------------------
	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	vector<DMatch> good_matches;

	for(int i = 0; i < feat1.descriptors.rows; i++)
	{ 
		if(matches[i].distance <= max(2*min_dist, 0.02) )
			{ 
				good_matches.push_back(matches[i]);
			}
	}

	return good_matches;
}

Mat features_fundMat(FEATURES feat1, FEATURES feat2, vector<DMatch> matches)
{
	// Include test for feature matching. Do not try to compute fund mat if
	// The features don't match...
	// ---------------
	// 

	int number_matches = (int)matches.size();

	vector<Point2f> points1(number_matches);
	vector<Point2f> points2(number_matches);

	for(int i = 0; i < number_matches; i++)
	{
	    points1[i] = feat1.keypoints[matches[i].queryIdx].pt;
	    points2[i] = feat2.keypoints[matches[i].trainIdx].pt;
	}

	Mat fund_mat;
	if(number_matches == 7){
		fund_mat = findFundamentalMat(points1, points2, CV_FM_7POINT, 1., 0.99);
	}
	else if(number_matches >= 8){
		fund_mat = findFundamentalMat(points1, points2, CV_FM_8POINT, 1., 0.99);
	}
	else{
		cout << "Not enough points to compute fundamental matrice." << endl;
		cout << "Fundamental Matrix: " << fund_mat << endl;
	}

	return fund_mat;
}

void draw_epipolar_lines(Mat img1, Mat img2, 
	vector<Point2f> points1, vector<Point2f> points2, 
	vector<Vec3f> epilines1, vector<Vec3f> epilines2)
{
	/*  -----------------------------------------------------------
		Draw the Epipolar Lines on the two images

		General line equation  :          a*x +          b*y +          c = 0
		Epipolar lines equation: epiline(0)*x + epiline(1)*y + epiline(3) = 0
		-> solve for x = 0 and x = image_width to get border points on images

		Number of epipolar lines and points drawn per image: NB_EPILINES_TO_DRAW
	   ----------------------------------------------------------- */
	int nb_drawn_epilines = 0;
	RNG rng(0);		// random colour for each epipolar line and point

	Mat im_cam1;
	Mat im_cam2;
	cvtColor(img1, im_cam1, CV_GRAY2BGR);
	cvtColor(img2, im_cam2, CV_GRAY2BGR);

	for(int i=0; i<points1.size() && nb_drawn_epilines<NB_EPILINES_TO_DRAW; i++)
	{
		Scalar color(rng(256),rng(256),rng(256));//Scalar( 0, 255, 0));
		line(im_cam2, 
			 Point(0,-epilines1[i][2]/epilines1[i][1]),
			 Point(im_cam1.cols, 
			   -(epilines1[i][2]+epilines1[i][0]*im_cam1.cols)/epilines1[i][1]),
			 color);  
		circle(im_cam1, points1[i], RADIUS_FEAT_DRAW, color, THICKN_FEAT_DRAW, 
			   CV_AA); // with antialiasing

		line(im_cam1,
			 Point(0,-epilines2[i][2]/epilines2[i][1]),
			 Point(im_cam2.cols,
			   -(epilines2[i][2]+epilines2[i][0]*im_cam2.cols)/epilines2[i][1]),
			 color);
		circle(im_cam2, points2[i], RADIUS_FEAT_DRAW, color, THICKN_FEAT_DRAW, 
			   CV_AA); // with antialiasing

		nb_drawn_epilines++;
	}

	opencv_show_image("Epipolar Lines on Image 1", im_cam1);
	opencv_show_image("Epipolar Lines on Image 2", im_cam2);
	cout << "-> Epipolar lines computed and drawn" << endl;
	waitKey(0);
}

EPIPOLAR_LINES features_epipolar_lines(FEATURES feat1, FEATURES feat2,
							vector<DMatch> matches, Mat fund_mat, 
							Mat img1, Mat img2)
{	//https://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/HZepipolar.pdf
	// Matched features used in epipolar lines computation
	int number_matches = (int)matches.size();
	vector<Point2f> points1(number_matches);
	vector<Point2f> points2(number_matches);

	// Reorder the matched features
	for(int i = 0; i < number_matches; i++){
	    points1[i] = feat1.keypoints[matches[i].queryIdx].pt;
	    points2[i] = feat2.keypoints[matches[i].trainIdx].pt;
	}

	// Epipolar lines computation
	EPIPOLAR_LINES epi_lines;
	/*vector<Vec3f> epilines1(points1.size());
	vector<Vec3f> epilines2(points2.size());*/
	cv::computeCorrespondEpilines(points1, 1, fund_mat, epi_lines.epilines1);
	cv::computeCorrespondEpilines(points2, 2, fund_mat, epi_lines.epilines2);

	draw_epipolar_lines(img1, img2,	points1, points2, 
						epi_lines.epilines1, epi_lines.epilines2);


	Mat H = findHomography(points1, points2, CV_RANSAC);
	cout << H << endl;

	return epi_lines;
}
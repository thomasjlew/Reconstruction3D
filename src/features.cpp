#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>  // for initModule_nonfree()
#include <opencv2/calib3d/calib3d.hpp>	// for FM_RANSAC
#include <opencv2/imgproc/imgproc.hpp>	// for cvt

#include "features.h"
#include "utils.h"

using namespace std;
using namespace cv;

//	Drawing Constants
#define RADIUS_FEAT_DRAW 10
#define THICKN_FEAT_DRAW 2
#define NB_EPILINES_TO_DRAW 15
#define NB_MAX_FEAT_FOR_FUND_MAT 25


/*	----------------------------------------------------------------------------
	Function to detect features in an image.
	Currently supported detector types: "SIFT", "SURF", "FAST", "ORB".
----------------------------------------------------------------------------- */
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
    waitKey(1);


    //	Displays one keypoint and outputs its coordinates to terminal
    /*
    Mat image_test;
	Scalar color((255),(0),(0));//Scalar( 0, 255, 0));
    image_test = image;
    Point center( features.keypoints[0].pt.x, features.keypoints[0].pt.y );
    cout << "POINT X: " << features.keypoints[0].pt.x 
    	 << "POINT Y: " << features.keypoints[0].pt.y << endl;
    circle( image_test, center, 10*RADIUS_FEAT_DRAW, color, 1, CV_AA );
    opencv_show_image("Test Image with one feature", image_test);
    	*/
	/*
	cout << "Displaying first 5 extracted features:" << endl;
	for(int i=0; i< 5; i++)
    	cout << " Extracted feature number " << i << ": (" 
    		 << features.keypoints[i].pt.x << "; " 
    		 << features.keypoints[i].pt.y << ")"<< endl; 
    	*/

    return features;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to compute the euclidean distance between two vectors.
----------------------------------------------------------------------------- */
double euclidean_distance(Mat_<double> v1, Mat_<double> v2)
{
	double distance = 0;

	if(v1.size() != v2.size())
	{
		cerr << "Unable to compute euclidean distance: Different sizes" << endl;
		return distance;
	}

	Mat_<double> diff_vectors = v1 - v2;
	Mat_<double> dist_mat_1x1_squared = diff_vectors.t() * diff_vectors;
	distance = sqrt(dist_mat_1x1_squared(0,0));

	return distance;
}
/* -------------------------------------------------------------------------- */

/*	----------------------------------------------------------------------------
	Function to match two set of features using brute force algorithms:
		Computes all euclidean distances successively (saved in a N1xN2 matrix)
		and matches the features with closest euclidean distance 
	!!! Current version is not used in project, too computationnaly expensive			
----------------------------------------------------------------------------- */
vector<DMatch> brute_force_matcher(Mat_<double> descriptors1, 
								   Mat_<double> descriptors2)
{
	vector<DMatch> matches;
	/*	matches[i].distance
		matches[i].queryIdx	descriptors1
		matches[i].trainIdx	descriptors2	*/
	int nb_cols1 = descriptors1.rows;
	int nb_cols2 = descriptors2.rows;
	cout << "nb_cols1" << nb_cols1 << endl;
	cout << "nb_cols2" << nb_cols2 << endl;
	//usual size: feat1.descriptors.size()[128 x 4572]
	Mat_<double> dists_mat(nb_cols1, nb_cols2, CV_64FC1);

	//cout << "descriptors1.size()" << descriptors1.size() << endl;

	//	Compute all euclidean distances between descriptors
	cout << "Computing euclidean distances for all features" << endl;
	for(int i=0; i < nb_cols1; i++)
	{
    	tic();
		for(int j=0; j < nb_cols2; j++)
		{
			Mat_<double> descr_vec_1 = descriptors1.row(i);
			Mat_<double> descr_vec_2 = descriptors2.row(j);

			dists_mat(i,j) = euclidean_distance(descr_vec_1, descr_vec_2);
		}
    	toc();
	}

	//	Id for one features linking to matching identifier in the other features
	vector<int> best_match_1(nb_cols1);
	vector<int> best_dists_1(nb_cols1);
	vector<int> best_match_2(nb_cols2);
	vector<int> best_dists_2(nb_cols2);
	double min_dist = 1000;
	cout << "Checking best matches for first features" << endl;
	for(int i=0; i < nb_cols1; i++)
	{
		min_dist = 1000;
		for(int j=0; j < nb_cols2; j++)
		{
			if(dists_mat(i,j) < min_dist)
			{
				min_dist = dists_mat(i,j);
				best_match_1[i] = j;
				best_dists_1[i] = min_dist;
			}
		}
	}
	cout << "Checking best matches for second features" << endl;
	for(int j=0; j < nb_cols2; j++)
	{
		min_dist = 1000;
		for(int i=0; i < nb_cols1; i++)
		{
			if(dists_mat(i,j) < min_dist)
			{
				min_dist = dists_mat(i,j);
				best_match_2[j] = i;
				best_dists_2[j] = min_dist;
			}
		}
	}

	//	Check that the best match for one features is also the one for the 2nd one
	cout << "Checking if best matches are the same for both images" << endl;
	for(int i=0; i < nb_cols1; i++)
		if(best_match_2[best_match_1[i]] == i)
		{
			DMatch new_match;
			new_match.queryIdx = i;
			new_match.trainIdx = best_match_1[i];
			new_match.distance = best_dists_1[i];

			matches.push_back(new_match);
		}

	return matches;
}
/* -------------------------------------------------------------------------- */

/*	----------------------------------------------------------------------------
	Function to match two set of features with a given precision.
	Inputs: 	- First set of features
				- Second set of features
				- precision: "GOOD" or "VERY_PRECISE"
	See features.h for recommendations.
----------------------------------------------------------------------------- */
vector<DMatch> features_matcher(FEATURES feat1,FEATURES feat2, string precision,
								int num_image )
{
	// Fast Approximate Nearest Neighbor Search Library
	/* see "http://docs.opencv.org/2.4/doc/tutorials/features2d/
	    	feature_flann_matcher/feature_flann_matcher.html"		*/
	int MIN_DIST_SCALAR = 2;
	if (precision == "GOOD")
		MIN_DIST_SCALAR = 10;
	if (precision == "VERY_PRECISE")
		MIN_DIST_SCALAR = 2;

	//	Other methods to be tried and implemented: 
	//		https://en.wikipedia.org/wiki/Nearest_neighbor_search
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(feat1.descriptors, feat2.descriptors, matches);
	//BFMatcher BFMatcher;
	//BFMatcher.match(feat1.descriptors, feat2.descriptors, matches);
	//	Algorithms in this file.
	//	brute_force_matcher is too computationaly expensive
	//matches = brute_force_matcher(feat1.descriptors, feat2.descriptors);

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
	cout << "num_image: " << num_image << endl;
	for(int i = 0; i < feat1.descriptors.rows; i++)
	{ 
		if(matches[i].distance <= max(MIN_DIST_SCALAR*min_dist, 0.02) )
			{ 
				/*if(feat2.keypoints[matches[i].trainIdx].pt.x > 2700)
				cout << ".x: " << feat2.keypoints[matches[i].trainIdx].pt.x << endl;
				if((num_image == 7 && feat2.keypoints[matches[i].trainIdx].pt.x > 2300))
					cout << "num_image == 7 && feat2.keypoints[matches[i].trainIdx].pt.x > 2300" << endl;*/
				if(!((num_image > 7 && feat2.keypoints[matches[i].trainIdx].pt.x > 2300)
					||(num_image == 7 && feat2.keypoints[matches[i].trainIdx].pt.x > 2300)))
					good_matches.push_back(matches[i]);
			}
	}

	//cout << "number of initial matches: " << matches.size()      << endl;
	//cout << "number of good matches:    " << good_matches.size() << endl;
	/*cout << matches << endl;
	cout << good_matches << endl;*/

	return good_matches;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to sort matches by their distance.
	Inputs: - (pointer) 1st vector of matches
			- (pointer) 2nd vector of matches
	Output: Boolean value indicating if the first match is more precise (1)
			or if the second match is more precise (0)
----------------------------------------------------------------------------- */
bool sort_by_distance(const my_match &left_match, const my_match &right_match)
{ 
	return left_match.distance < right_match.distance; 
}
/* -------------------------------------------------------------------------- */

/*	----------------------------------------------------------------------------
	Function to sort features in decreasing order of precision.
	Inputs: - First set of features
			- Second set of features
			- Matches vector linking the two sets of features
	Output: Matches vector with sorted 2d points (the most precise ones first)
----------------------------------------------------------------------------- */
vector<my_match> get_most_precise_matches(FEATURES feat1, FEATURES feat2, 
										  vector<DMatch> matches)
{
	int number_matches = (int)matches.size();

	vector<double> distances(number_matches);

	vector<my_match> vec_matches(number_matches);
	for(int i = 0; i < number_matches; i++)
	{
	    //	Save the distance of the matches
	    distances[i] = matches[i].distance;

	    //	Save these informations into the structure
	    vec_matches[i].point1 = feat1.keypoints[matches[i].queryIdx].pt;
	    vec_matches[i].point2 = feat2.keypoints[matches[i].trainIdx].pt;
	    vec_matches[i].distance = matches[i].distance;
	}
	//	Sort these matches to get the smallest distances first
    sort(vec_matches.begin(), vec_matches.end(), sort_by_distance);

    return vec_matches;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to compute the error from the fundamental matrix and given matches.
	Inputs: - Fundamental matrix
			- Number of matches (Only an error message is displayed if wrong)
			- First vector of points (The points need to be sorted by pairs)
			- Second vector of points
			- Message to be output on terminal indicating nature of F matrix.
	Output: Matches vector with sorted 2d points (the most precise ones first)
----------------------------------------------------------------------------- */
double test_fund_mat_error(Mat fund_mat, int num_matches,
						   vector<Point2f> pts1, vector<Point2f> pts2,
						   string details_msg = " ")
{
	/*if(num_matches != length(pts1) || num_matches != length(pts2))
	{
		cerr << "Error computing F matrix error: wrong vector sizes" << endl;
	}*/

	Mat_<double> error = (Mat_<double>(1,1) << 0);
	for (int i=0; i<num_matches; i++){
	 	Mat_<double> p1 = (Mat_<double>(3,1) << pts1[i].x, pts1[i].y, 1);
	 	Mat_<double> p2 = (Mat_<double>(1,3) << pts2[i].x, pts2[i].y, 1);
		error += abs(p2*fund_mat*p1);
	}
	cout << "Error for F computation (" << details_msg << "): " << error << endl;
	//	The error should be smaller than approximately 1

	double final_error = error(0,0);

	return final_error;
}
/* -------------------------------------------------------------------------- */

/*	----------------------------------------------------------------------------
	Computes the fundamental matrix F using the 8-points algorithm such that
			x2' * F * x1 = 0
	Algorithm from page 282, Chapter 11 of 2nd edition of 
	R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
	Cambridge Univ. Press, 2003. [1]

	The points must be already ordered according to matches
----------------------------------------------------------------------------- */
Mat compute_fund_mat(vector<Point2f> pts1, vector<Point2f> pts2)
{
	int number_matches = (int)pts1.size();
	//  Check if the matched features have the same length
	if (number_matches != (int)pts2.size())
	{
	    cerr << "Error computing F: features dont match" << endl;
		Mat F =  (Mat_<double>(3,3) << 	0, 0, 0,
								   		0, 0, 0,
								   		0, 0, 0);
	    return F;
	}
	//	Check if the matched features are enough to compute F
	if (number_matches < 8)
	{
	    cerr << "Error computing F: not enough features" << endl;
		Mat F =  (Mat_<double>(3,3) << 	0, 0, 0,
								   		0, 0, 0,
								   		0, 0, 0);
	    return F;
	}

	number_matches = 8;	//	use 8 points

	//	------------------------------------------------------------------------
	//	Normalise data

	//	Compute centroids ("center of masses")
	double mean_pts1_x = 0; double mean_pts1_y = 0;
	double mean_pts2_x = 0; double mean_pts2_y = 0;
	for(int i=0; i < number_matches; i++)
	{
		mean_pts1_x += pts1[i].x;
		mean_pts1_y += pts1[i].y;
		mean_pts2_x += pts2[i].x;
		mean_pts2_y += pts2[i].y;
	}
	mean_pts1_x /= number_matches; mean_pts1_y /= number_matches;
	mean_pts2_x /= number_matches; mean_pts2_y /= number_matches;
	Point2f mean_pts1 = Point(mean_pts1_x, mean_pts1_y);
	Point2f mean_pts2 = Point(mean_pts2_x, mean_pts2_y);

	//	Compute current average distance to center
	double variance_pts1 = 0;
	double variance_pts2 = 0;
	for(int i=0; i < number_matches; i++)
	{
	    variance_pts1 += sqrt((pts1[i].x-mean_pts1_x)*(pts1[i].x-mean_pts1_x) + 
	    		  		 	  (pts1[i].y-mean_pts1_y)*(pts1[i].y-mean_pts1_y) );
	    variance_pts2 += sqrt((pts2[i].x-mean_pts2_x)*(pts2[i].x-mean_pts2_x) + 
	    		  		 	  (pts2[i].y-mean_pts2_y)*(pts2[i].y-mean_pts2_y) );
	}
	variance_pts1 /= number_matches;
	variance_pts2 /= number_matches;

	//	Normalise data: center it and scale its average distance to center of
	//	centroid to be equal to one (could be sqrt(2) as in HZ book)
	for(int i=0; i < number_matches; i++)
	{
		pts1[i] = (pts1[i] - mean_pts1) * (sqrt(2)/variance_pts1);
		pts2[i] = (pts2[i] - mean_pts2) * (sqrt(2)/variance_pts2);
	}

	//	Compute the inverse transformation (to recover F later) 
	Mat_<double> T1 = (Mat_<double>(3,3) << 
	 sqrt(2)/variance_pts1,   			0, -mean_pts1_x * sqrt(2)/variance_pts1,
	             0, sqrt(2)/variance_pts1, -mean_pts1_y * sqrt(2)/variance_pts1,
	             0,          		    0,                 			 		 1);
	Mat_<double> T2 = (Mat_<double>(3,3) << 
	 sqrt(2)/variance_pts2,   			0, -mean_pts2_x * sqrt(2)/variance_pts2,
	             0, sqrt(2)/variance_pts2, -mean_pts2_y * sqrt(2)/variance_pts2,
	             0,          			0,                 			 		 1);

	//	Build matrix A from the matches
	Mat_<double> A = (Mat_<double>(number_matches,9));
	for(int i=0; i < number_matches; i++)
	{
	    double x1 = pts1[i].x; double y1 = pts1[i].y;
	    double x2 = pts2[i].x; double y2 = pts2[i].y;
	    Mat_<double> a = (Mat_<double>(1,9) << 
	    						x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1);
	    for(int j=0; j < 9; j++)
	    	A(i,j) = a(0,j);
	}

	//	Perform Singular value decomposition
	Mat_<double> w, U, V, Vt;
	SVD::compute(A, w, U, Vt);
	V = Vt.t();

	Size s = V.size();
	int n_cols = s.width-1;

	Mat_<double> f = (Mat_<double>(9,1) << 	
							V(0,n_cols), V(1,n_cols), V(2,n_cols), V(3,n_cols),
			   V(4,n_cols), V(5,n_cols), V(6,n_cols), V(7,n_cols), V(8,n_cols));

	//	Recompose fundamental matrix F
	Mat_<double> F =  (Mat_<double>(3,3) << 	f(0,0), f(1,0), f(2,0),
								    			f(3,0), f(4,0), f(5,0),
								   				f(6,0), f(7,0), f(8,0));

	//	Enforce singularity (Constraint enforcement) ( rank(F)=2 )
	Mat_<double> w_F, U_F, Vt_F;
	SVD::compute(F, w_F, U_F, Vt_F);
	//w_F(2,2) = 0;         //	Enforce the smallest eigenvalue to be zero
	Mat_<double> D = (Mat_<double>(3,3) <<  w_F(0,0), 		 0, 	0,
								 		  		   0, w_F(1,1),  	0,
								 		  		   0,        0,  	0);
	//   Reform F which is now rank 2
	F = U_F * D * Vt_F;

	//   Undo normalisation to get the fundamental matrix
	F = T2.t() * F * T1;

	//   Rescale to have F(3,3)=1
	F = F / F(2,2);

	return F;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to compute the fundamental matrix from given matched features.
	Inputs: - First set of features
			- Second set of features
			- Vector of matches
	Output: Fundamental matrix
----------------------------------------------------------------------------- */
double max_F_error = 0;
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

	//	Sort these matches to get the smallest distances first
	vector<my_match> vec_matches(number_matches);
	vec_matches = get_most_precise_matches(feat1, feat2, matches);

    //	Use only the 25 best matches for the fundamental matrix computation
    const int num_match_F = min(NB_MAX_FEAT_FOR_FUND_MAT, number_matches);		//--------BIG PARAMETER, how to choose? better than 25
	vector<Point2f> F_points1(num_match_F);
	vector<Point2f> F_points2(num_match_F);
	for(int i = 0; i < num_match_F; i++)
	{
		F_points1[i] = vec_matches[i].point1;
		F_points2[i] = vec_matches[i].point2;
	}

	Mat fund_mat;
	Mat fund_mat1;
	Mat fund_mat2;
	Mat fund_mat3;
	Mat fund_mat4;
	Mat fund_mat_custom;
	if(number_matches == 7){
		cout << "Matches number for F computation: " << number_matches << endl;
		fund_mat = findFundamentalMat(points1, points2, CV_FM_7POINT, 1., 0.99);
	}
	else if(number_matches >= 8){
		cout << "Matches number for F computation: " << number_matches << endl;
		//cout << "Matches number for F computation: " << num_match_F   << endl;

		fund_mat1 = findFundamentalMat(points1, points2, CV_FM_8POINT, 1., 0.99);
		//cout << "Fundamental Matrix 1: " << endl << fund_mat1 << endl;	//*/

		fund_mat2 = findFundamentalMat(F_points1, F_points2, CV_FM_8POINT, 1., 0.99);
		//cout << "Fundamental Matrix 2: " << endl << fund_mat2 << endl;	//*/

		cout << "fund_mat2" << fund_mat2 << endl;
		fund_mat3 = findFundamentalMat(points1, points2, CV_FM_RANSAC, 1., 0.99);
		//cout << "Fundamental Matrix 3: " << endl << fund_mat3 << endl;	//*/

		/*fund_mat4 = findFundamentalMat(F_points1, F_points2, CV_FM_RANSAC, 1., 0.99);
		cout << "Fundamental Matrix: " << fund_mat3 << endl;	//*/
		fund_mat4 = findFundamentalMat(F_points1, F_points2);
		//cout << "Fundamental Matrix 4: " << endl << fund_mat4 << endl;	//*/

		/*fund_mat_custom = compute_fund_mat(F_points1, F_points2);
		cout << "fund_mat_custom" << fund_mat_custom << endl;
		double err_custom = test_fund_mat_error(fund_mat_custom, number_matches, 
							points1, points2, "from points1&2, fund_mat_custom");*/
	}													
	else{
		cout << "Not enough points to compute fundamental matrix." << endl;
		cout << "Fundamental Matrix: " << endl << fund_mat << endl;
	}

	//	Test of the fundamental matrix with matches to compute the error
	double err_1 = test_fund_mat_error(fund_mat1, number_matches, 
							points1, points2, "pts1, pts2, CV_FM_8POINT");
	double err_2 = test_fund_mat_error(fund_mat2, number_matches, 
							points1, points2, "from  F_pts1&2, CV_FM_8POINT");
	double err_3 = test_fund_mat_error(fund_mat3, number_matches, 
							points1, points2, "pts1,   pts2,   CV_FM_RANSAC");
	double err_4 = test_fund_mat_error(fund_mat4, number_matches, 
							points1, points2, "from  F_pts1&2, CV_FM_RANSAC");

	//	----------------------- REFERENCE FOR IMGs 5-6 -----------------------//
	//	Fundamental matrix from ground truth data for comparison in imgs5-6
	Mat F =  (Mat_<double>(3,3) << 5.64028e-10, -3.97032e-08, 3.54125e-05,
								   7.04518e-10, -2.61959e-09, 0.00062845,
								   7.24224e-06, -0.000554166, -0.097117);
	test_fund_mat_error(F, number_matches, points1, points2, 
						"Ground truth (imgs pair 5-6)");
	/*cout << " ++++ !!!! ground truth FUND MAT for imgs 5-6 currently used!!! ++++ !!!! " << endl;
	fund_mat =  (Mat_<double>(3,3) << 5.64028e-10, -3.97032e-08, 3.54125e-05,
								   7.04518e-10, -2.61959e-09, 0.00062845,
								   7.24224e-06, -0.000554166, -0.097117);*/
	//	----------------------- REFERENCE FOR IMGs 5-6 -----------------------//


	//	Return the most precise fundamental matrix
	fund_mat = fund_mat1;
	if(abs(err_1) > abs(err_2))
		fund_mat = fund_mat2;
	if(abs(err_2) > abs(err_3) && abs(err_1) > abs(err_3))
		fund_mat = fund_mat3;
	if(abs(err_3) > abs(err_4) && abs(err_2) > abs(err_4) && abs(err_1) > abs(err_4))
		fund_mat = fund_mat4;


	//	Update the error
	if(min(abs(err_1),min(abs(err_2),min(abs(err_3),abs(err_4)))) > max_F_error)
	{
		max_F_error=min(abs(err_1),min(abs(err_2),min(abs(err_3),abs(err_4))));
		cout << " BIGGER ERROR FOR F: " << max_F_error << endl;
	}

	//	Interestingly, this one works way better for triangulation with images pair 0-1 but not for epipolar lines
	//fund_mat = findFundamentalMat(points1, points2, CV_FM_8POINT, 1., 0.99);
	
	//fund_mat = fund_mat.t();
	return fund_mat;
}
/* -------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
	Draws the Epipolar Lines on the two images

	General line equation  :          a*x +          b*y +          c = 0
	Epipolar lines equation: epiline(0)*x + epiline(1)*y + epiline(3) = 0
	-> solve for x = 0 and x = image_width to get border points on images

	Number of epipolar lines and points drawn per image: NB_EPILINES_TO_DRAW
   -------------------------------------------------------------------------- */
void draw_epipolar_lines(Mat img1, Mat img2, 
	vector<Point2f> points1, vector<Point2f> points2, 
	vector<Vec3f> epilines1, vector<Vec3f> epilines2)
{
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
/* -------------------------------------------------------------------------- */


/*  ----------------------------------------------------------------------------
	Computes and draws the Epipolar Lines on the two images
	
	For reference, see Chapter 9: Epipolar Geometry and the Fundamental Matrix,
	R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
	Cambridge Univ. Press, 2003. [1].

	Number of epipolar lines and points drawn per image: NB_EPILINES_TO_DRAW
   -------------------------------------------------------------------------- */
EPIPOLAR_LINES features_epipolar_lines(FEATURES feat1, FEATURES feat2,
							vector<DMatch> matches, Mat fund_mat, 
							Mat img1, Mat img2)
{
	// Number of matched features used in epipolar lines computation
	int number_matches = (int)matches.size();

	//  Sort, reorder the matched features to get the smallest distances first
	vector<my_match> vec_matches(number_matches);
	vec_matches = get_most_precise_matches(feat1, feat2, matches);

    /*	Save these reordered matched features into vector points
    	Note: Only the first NB_EPILINES_TO_DRAW are actually needed
    		  To change this to get all the matches reorder, 
    			-> change NB_EPILINES_TO_DRAW to number_matches			*/
	vector<Point2f> pts1(NB_EPILINES_TO_DRAW);
	vector<Point2f> pts2(NB_EPILINES_TO_DRAW);
	for(int i = 0; i < NB_EPILINES_TO_DRAW; i++)
	{
		pts1[i] = vec_matches[i].point1;
		pts2[i] = vec_matches[i].point2;
	}

	// Epipolar lines computation
	EPIPOLAR_LINES epi_lines;
	cv::computeCorrespondEpilines(pts1, 1, fund_mat, epi_lines.epilines1);
	cv::computeCorrespondEpilines(pts2, 2, fund_mat, epi_lines.epilines2);

	//	Draws the first NB_EPILINES_TO_DRAW (15) most precise matches & epilines
	draw_epipolar_lines(img1, img2,	pts1, pts2, 
						epi_lines.epilines1, epi_lines.epilines2);

	return epi_lines;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Additional function for triangulation. Original version at, and adapted from
		http://stackoverflow.com/questions/24354889/problems-in-3d-
		reconstruction-by-triangulation-opencv 
----------------------------------------------------------------------------- */
Mat triangulate_Linear_LS(Mat_<double> P1, Mat_<double> P2, 
							Mat_<double> pt1, Mat_<double>  pt2)
{
    Mat A(4,3,CV_64FC1), b(4,1,CV_64FC1), X(3,1,CV_64FC1), 
    	X_homogeneous(4,1,CV_64FC1), W(1,1,CV_64FC1);

    W.at<double>(0,0) = 1.0;
    A.at<double>(0,0) = (pt1(0,0)/pt1(2,0))*P1(2,0) - P1(0,0);
    A.at<double>(0,1) = (pt1(0,0)/pt1(2,0))*P1(2,1) - P1(0,1);
    A.at<double>(0,2) = (pt1(0,0)/pt1(2,0))*P1(2,2) - P1(0,2);
    A.at<double>(1,0) = (pt1(1,0)/pt1(2,0))*P1(2,0) - P1(1,0);
    A.at<double>(1,1) = (pt1(1,0)/pt1(2,0))*P1(2,1) - P1(1,1);
    A.at<double>(1,2) = (pt1(1,0)/pt1(2,0))*P1(2,2) - P1(1,2);
    A.at<double>(2,0) = (pt2(0,0)/pt2(2,0))*P2(2,0) - P2(0,0);
    A.at<double>(2,1) = (pt2(0,0)/pt2(2,0))*P2(2,1) - P2(0,1);
    A.at<double>(2,2) = (pt2(0,0)/pt2(2,0))*P2(2,2) - P2(0,2);
    A.at<double>(3,0) = (pt2(1,0)/pt2(2,0))*P2(2,0) - P2(1,0);
    A.at<double>(3,1) = (pt2(1,0)/pt2(2,0))*P2(2,1) - P2(1,1);
    A.at<double>(3,2) = (pt2(1,0)/pt2(2,0))*P2(2,2) - P2(1,2);
    b.at<double>(0,0) = -((pt1(0,0)/pt1(2,0))*P1(2,3) - P1(0,3));
    b.at<double>(1,0) = -((pt1(1,0)/pt1(2,0))*P1(2,3) - P1(1,3));
    b.at<double>(2,0) = -((pt2(0,0)/pt2(2,0))*P2(2,3) - P2(0,3));
    b.at<double>(3,0) = -((pt2(1,0)/pt2(2,0))*P2(2,3) - P2(1,3));

    solve(A,b,X,DECOMP_SVD);
    vconcat(X,W,X_homogeneous);
    return X_homogeneous;
}
/* -------------------------------------------------------------------------- */

/*	--------------------------------------------------------------------------
	Computes the depth of a 3D point given the camera matrix.
    Used to determine whether the point is in front of the camera image plane 

	Inputs:
		pt_3d - 3D point in homogeneous coordinates (x,y,z,w=1) [1x4] (CV_32FC4)
		P     - Camera matrix P = K[R|t]									

	*Assumption:	The matrix from the first 3 columns of P is inversible.	*/
double depth_pt(Mat_<double> pt_3d, Mat_<double> P)
{
	//	Compute camera center c from camera matrix
	Mat A1 = (Mat_<double>(3,3) << P(0,1), P(0,2), P(0,3),
		 						   P(1,1), P(1,2), P(1,3),
		 						   P(2,1), P(2,2), P(2,3));
	Mat A2 = (Mat_<double>(3,3) << P(0,0), P(0,2), P(0,3),
		 						   P(1,0), P(1,2), P(1,3),
		 						   P(2,0), P(2,2), P(2,3));
	Mat A3 = (Mat_<double>(3,3) << P(0,0), P(0,1), P(0,3),
		 						   P(1,0), P(1,1), P(1,3),
		 						   P(2,0), P(2,1), P(2,3));
	Mat A4 = (Mat_<double>(3,3) << P(0,0), P(0,1), P(0,2),
		 						   P(1,0), P(1,1), P(1,2),
		 						   P(2,0), P(2,1), P(2,2));
	double x =  determinant(A1);
	double y = -determinant(A2);
	double z =  determinant(A3);
	double t = -determinant(A4);
	Mat c = (Mat_<double>(3,1) << x/t, y/t, z/t);

	//	Extract the first three columns
	Mat M = (Mat_<double>(3,3) << P(0,0), P(0,1), P(0,2),
		 						  P(1,0), P(1,1), P(1,2),
		 						  P(2,0), P(2,1), P(2,2));
	Mat last_line_M = (Mat_<double>(1,3) << P(2,0), P(2,1), P(2,2));

	//	Projection of the 3d point onto the z-axis of the camera
	Mat_<double> pt = (Mat_<double>(3,1) << pt_3d(0,0), pt_3d(1,0), pt_3d(2,0));
	Mat_<double> w = last_line_M * (pt - c);

	/* The 3rd row of P points towards the positive z-axis if the determinant of
	   P(1,3)>0. Hence, the scalar product of w with this direction vector
	   represents the depth of the 3d point 								*/
	// Simplified sign function. See *Assumption
	double sign_det  = (double)((determinant(M)>0) ? 1 : (-1) );
	double depth = sign_det * w(0,0) / norm(last_line_M);

	return depth;
}
/* -------------------------------------------------------------------------- */


/*	--------------------------------------------------------------------------
	Computes the 2nd normalised extrinsics camera matrix P2 ( [R|t] ) from the 
	essential matrix and the best available pair of matches.
	The intrinsics matrix K, pt1 & pt2 are used to check the solution.
		pt1 & pt2 - Best match [1 x 2] in first and second images (only 1)	
   -------------------------------------------------------------------------- */
Mat_<double> get_Rt_from_essential_mat(Mat_<double> E, Mat_<double> K, 
									   vector<Point2f> pt1, vector<Point2f> pt2)
{
	//	P1 is set at the origin of the world frame, with same orientation
	Mat P1 =  (Mat_<double>(3,4) <<  1,     0,     0,     0,
								     0,     1,     0,     0,
								     0,     0,     1,     0);

	/*cout << "E in RTf(x)" << E <<endl;
	cout << "K in RTf(x)" << K <<endl;
	cout << "pt1 in RTf(x)" << pt1 <<endl;
	cout << "pt2 in RTf(x)" << pt2 <<endl;*/

	//	SVD decomposition of Essential matrix to later compute the second
	//		projection matrix P2.
	Mat w, U, Vt;	//	w is a 3x1 vector, U and Vt are 3x3 matrices
	Mat R1, R2, R3, R4, t1, t2, t3, t4;	// t are 3x1 vectors, R are 3x3 matrices
	SVD::compute(E, w, U, Vt);

	Mat W =   (Mat_<double>(3,3) <<  0,    -1,     0,
								     1,     0,     0,
								     0,     0,     1);
	Mat Wt =  (Mat_<double>(3,3) <<  0,     1,     0,
								    -1,     0,     0,
								     0,     0,     1);

	//	Four solutions for the reconstructed projection matrix P2.
	R1 = U*W*Vt;
	t1 =  U.col(2);
	R2 = U*W*Vt;
	t2 = -U.col(2);
	R3 = U*Wt*Vt;
	t3 =  U.col(2);
	R4 = U*Wt*Vt;
	t4 = -U.col(2);
	vector<Mat> matrices_1 = {R1, t1};
	vector<Mat> matrices_2 = {R2, t2};
	vector<Mat> matrices_3 = {R3, t3};
	vector<Mat> matrices_4 = {R4, t4};
	Mat P2_1;
	Mat P2_2;
	Mat P2_3;
	Mat P2_4;
	hconcat( matrices_1, P2_1 );
	hconcat( matrices_2, P2_2 );
	hconcat( matrices_3, P2_3 );
	hconcat( matrices_4, P2_4 );

	//	Un-normalise the projection matrices
	P1 = K*P1;
	P2_1 = K*P2_1;
	P2_2 = K*P2_2;
	P2_3 = K*P2_3;
	P2_4 = K*P2_4;
	/*cout << "pt1" << pt1 << endl;
	cout << "pt2" << pt2 << endl;
	cout << "R1" << R1 << endl;
	cout << "t1" << t1 << endl;	
	cout << "P1 = K*P1;" << P1 << endl;	//good
	cout << "P2_1 = K*P2_1;" << P2_1 << endl;	//good*/

	//	Triangulate
	cv::Mat homog_3d_pt_1(4, 1, CV_32FC4);
	cv::Mat homog_3d_pt_2(4, 1, CV_32FC4);
	cv::Mat homog_3d_pt_3(4, 1, CV_32FC4);
	cv::Mat homog_3d_pt_4(4, 1, CV_32FC4);
	triangulatePoints(P1, P2_1, pt1, pt2, homog_3d_pt_1);
	triangulatePoints(P1, P2_2, pt1, pt2, homog_3d_pt_2);
	triangulatePoints(P1, P2_3, pt1, pt2, homog_3d_pt_3);
	triangulatePoints(P1, P2_4, pt1, pt2, homog_3d_pt_4);
	//	Set last coordinate of homogeneous coordinates to 1 (w=1)
	float scale = homog_3d_pt_1.at<float>(3,0);
	homog_3d_pt_1.at<float>(0,0) /= scale;
	homog_3d_pt_1.at<float>(1,0) /= scale;
	homog_3d_pt_1.at<float>(2,0) /= scale;
	homog_3d_pt_1.at<float>(3,0) /= scale;
	scale = homog_3d_pt_2.at<float>(3,0);
	homog_3d_pt_2.at<float>(0,0) /= scale;
	homog_3d_pt_2.at<float>(1,0) /= scale;
	homog_3d_pt_2.at<float>(2,0) /= scale;
	homog_3d_pt_2.at<float>(3,0) /= scale;
	scale = homog_3d_pt_3.at<float>(3,0);
	homog_3d_pt_3.at<float>(0,0) /= scale;
	homog_3d_pt_3.at<float>(1,0) /= scale;
	homog_3d_pt_3.at<float>(2,0) /= scale;
	homog_3d_pt_3.at<float>(3,0) /= scale;
	scale = homog_3d_pt_4.at<float>(3,0);
	homog_3d_pt_4.at<float>(0,0) /= scale;
	homog_3d_pt_4.at<float>(1,0) /= scale;
	homog_3d_pt_4.at<float>(2,0) /= scale;
	homog_3d_pt_4.at<float>(3,0) /= scale;
	/*cout << "homog_3d_pt_1" << homog_3d_pt_1 << endl;
	cout << "homog_3d_pt_2" << homog_3d_pt_2 << endl;
	cout << "homog_3d_pt_3" << homog_3d_pt_3 << endl;
	cout << "homog_3d_pt_4" << homog_3d_pt_4 << endl;*/

	// Select the camera matrices such that the point is in front of the cameras
	Mat R, t;	// Final Rotation matrix and translation vector for camera m. P2
	//cout << "depth_pt(homog_3d_pt_1, P1) " << depth_pt(homog_3d_pt_1, P1) << endl;
	if 	   (depth_pt(homog_3d_pt_1, P1)>0 && depth_pt(homog_3d_pt_1, P2_1)>0){
		cout << "First solution is the good one" << endl;
	    R = R1;
	    t = t1;
	}
	else if(depth_pt(homog_3d_pt_2, P1)>0 && depth_pt(homog_3d_pt_2, P2_2)>0){
		cout << "2nd solution is the good one" << endl;
	    R = R2;
	    t = t2;
	}
	else if(depth_pt(homog_3d_pt_3, P1)>0 && depth_pt(homog_3d_pt_3, P2_3)>0){
		cout << "3rd solution is the good one" << endl;
	    R = R3;
	    t = t3;
	}
	else if(depth_pt(homog_3d_pt_4, P1)>0 && depth_pt(homog_3d_pt_4, P2_4)>0){
		cout << "4th solution is the good one" << endl;
	    R = R4;
	    t = t4;
	}
	else{
	    cerr << "Problem: No [R|t] can reconstruct the 3d Points" << endl;
		Mat R =  Mat::zeros(3, 3, CV_64F);
		Mat t =  Mat::zeros(3, 1, CV_64F);
	}

	vector<Mat> matrices = {R, t};
	Mat_<double> Rt12;
	hconcat(matrices, Rt12);

	return Rt12;
}
/* -------------------------------------------------------------------------- */

/*	-----------------------------------------------------------------------
	Triangulation function

	TODO: 	
		Run LM on inliers to refine (R, t)
	----------------------------------------------------------------------- */
//	The intrinics matrix is defined in the header as K for each image
void features_triangulation(Mat fund_mat, 
							Mat img1, Mat img2, FEATURES feat1, 
							FEATURES feat2, vector<DMatch> matches)
{

	/*	Reorder points according to matches 
		(duplicated code from features_epipolar_lines)	*/
	int number_matches = (int)matches.size();
	vector<Point2f> points1(number_matches);
	vector<Point2f> points2(number_matches);
	for(int i = 0; i < number_matches; i++){
	    points1[i] = feat1.keypoints[matches[i].queryIdx].pt;
	    points2[i] = feat2.keypoints[matches[i].trainIdx].pt;
	}

	//	Computation of the Essential matrix
	Mat ess_mat = K.t() * fund_mat * K;

	/*
	//	OPENCV3 FUNCTION
 	double focal = K.at<double>(0, 0); //Note: assuming fx = fy
    cv::Point2d pp(K.at<double>(0, 2), K.at<double>(1, 2));
    Mat ess_mat;
	Mat mask;
	ess_mat = findEssentialMat(points1, points2, focal, pp, RANSAC, 0.999, 1.0, mask);*/


	cout << "Main: Essential matrix computed: " << endl;
	cout << ess_mat << endl;

	//	Computation of normalized projection (camera) matrices
	Mat P1 =  (Mat_<double>(3,4) <<  1,     0,     0,     0,
								     0,     1,     0,     0,
								     0,     0,     1,     0);

	Mat w, U, Vt;	//	w is a 3x1 vector, U and Vt are 3x3 matrices
	Mat R, t;		//	t is a 3x1 vector, R is a 3x3 matrix
	SVD::compute(ess_mat, w, U, Vt);
	//cout << U.size() << Mat::diag(w).size() << Vt.size() << endl;

	Mat W =   (Mat_<double>(3,3) <<  0,    -1,     0,
								     1,     0,     0,
								     0,     0,     1);
	Mat Wt =  (Mat_<double>(3,3) <<  0,     1,     0,
								    -1,     0,     0,
								     0,     0,     1);
	/*R = U*W*Vt;
	t = U.col(2);//*/
	/*R = U*W*Vt;
	t = -U.col(2);//*/
	/*R = U*Wt*Vt;
	t = U.col(2);//*/
	R = U*Wt*Vt;
	t = -U.col(2);//*/
	vector<Mat> matrices = {R, t};
	Mat P2;// =  (Mat_<double>(3,4));
	hconcat( matrices, P2 );
	cout << "Second (normalised) projection matrix P2: " << endl << P2 << endl;

	/*	From EPFL fountain_dense dataset 
		http://cvlabwww.epfl.ch/data/multiview/fountain_dense.html
	Mat P1 =(Mat_<double>(3,4) << -105.345, -3146,   -137.149, -24575.2,
								 -1154.97, -563.355,  2646.3,  -13219.2, 
								 -0.887537,-0.449183,-0.102528,-9.84483 );
	Mat P2 =(Mat_<double>(3,4) << 379.365, -3122.38, -184.594, -16544,
							      -1084.51, -777.117, 2621.87, -14348,
							   -0.807052, -0.577925, -0.121118, -10.3411 );*/

	/*//	Result from Matlab script (see reconstruct.m)
	Mat P2 = (Mat_<double>(3,4) << -0.1373,   -2.7106,   -0.6412,   -0.9978,
	    							7.2929,   -0.6274,   40.7885,   -0.0143,
	  							   -0.5008,  -41.9537,   -0.8486,    0.0647 );*/


	/* -------------FURTHER DEVELOPMENT-----------------------------------------
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): see 
	http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/SfM.html
	------------------------------------------------------------------------- */



/*
 *	The Optimal Triangulation Method (see HZ for details)
 *		For each given point correspondence points1[i] <-> points2[i], and a fundamental matrix F,
 *		computes the corrected correspondences new_points1[i] <-> new_points2[i] that minimize the
 *		geometric error d(points1[i],new_points1[i])^2 + d(points2[i],new_points2[i])^2 (where d(a,b)
 *		is the geometric distance between points a and b) subject to the epipolar constraint
 *		new_points2' * F * new_points1 = 0.*/
	vector<Point2f> new_points1(number_matches);
	vector<Point2f> new_points2(number_matches);
	cv::correctMatches( fund_mat, points1, points2,
						new_points1, new_points2 );

	//	Un-normalise the projection matrices
	//P1 = K*P1;
	P2 = K*P2;


	//	Get most precise match
	vector<my_match> vec_matches(number_matches);
	vec_matches = get_most_precise_matches(feat1, feat2, matches);
	vector<Point2f> best_pt1(1); best_pt1[0] = vec_matches[0].point1;
	vector<Point2f> best_pt2(1); best_pt2[0] = vec_matches[0].point2;



//-------------------------------REMOVE WHAT IS ABOVE HERE
P2 = K*get_Rt_from_essential_mat(ess_mat, K, best_pt1, best_pt2);

P1 = K*P1;
	/*vector<Point2f> undistorded_pts1(number_matches);
	vector<Point2f> undistorded_pts2(number_matches);
	//Mat dist_coeffs =  (Mat_<double>(8,1) <<  	NULL, NULL, NULL, NULL, 
	//											NULL, NULL, NULL, NULL);
	undistort(new_points1, undistorded_pts1, K, Mat());
	undistort(new_points2, undistorded_pts2, K, Mat());*/


	//	Triangulation of the points
	cv::Mat reconstructed_points(4, number_matches, CV_32FC4);	//1st parameter can be set to 1

	//triangulatePoints(P1, P2, undistorded_pts1, undistorded_pts2, reconstructed_points);

	triangulatePoints(P1, P2, new_points1, new_points2, reconstructed_points);
	//triangulatePoints(P1, P2, new_points2, new_points1, reconstructed_points);

	/*  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		it might be necessary to swap points 1 and points 2 in this function
		++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  */


	
	for (int i=0; i<number_matches; i++)
	{
		//	Set last coordinate of homogeneous coordinates to 1 (w=1)
		float scale = reconstructed_points.at<float>(3,i);
		reconstructed_points.at<float>(0,i) /= scale;
		reconstructed_points.at<float>(1,i) /= scale;
		reconstructed_points.at<float>(2,i) /= scale;
		reconstructed_points.at<float>(3,i) /= scale;

		//	Set last 3d coordinate of 3d point to positive values 1 (z=1)
		/*if(reconstructed_points.at<float>(2,i) < 0){
			reconstructed_points.at<float>(0,i) *= -1;
			reconstructed_points.at<float>(1,i) *= -1;
			reconstructed_points.at<float>(2,i) *= -1;
		}*/
	}

	//cout << "Reconstructed_points: " << endl << reconstructed_points << endl;
	cout << "size of reconstructed pts: " << (reconstructed_points.size()) <<endl;
	cout << "1st point: x: " << reconstructed_points.at<float>(0,0) 
		<< ", y: " << reconstructed_points.at<float>(1,0)
		<< ", z: " << reconstructed_points.at<float>(2,0) 
		<< ", w: " << reconstructed_points.at<float>(3,0) << endl;

	//	Other triangulation method, see triangulate_Linear_LS()
	/*Mat results_homogeneous(4, number_matches ,CV_64FC4);
	for (int i=0; i<number_matches; i++)
	{
		Mat matpts1 = (Mat_<double>(3,1) << (double)new_points1[i].x, (double)new_points1[i].y, 1);
		Mat matpts2 = (Mat_<double>(3,1) << (double)new_points2[i].x, (double)new_points2[i].y, 1);

		Mat result_homogeneous = triangulate_Linear_LS(P1, P2, matpts1, matpts2);
		//cout <<result_homogeneous<< endl;

		results_homogeneous.at<double>(0,i) = result_homogeneous.at<double>(0,0);
		results_homogeneous.at<double>(1,i) = result_homogeneous.at<double>(1,0);
		results_homogeneous.at<double>(2,i) = result_homogeneous.at<double>(2,0);
		results_homogeneous.at<double>(3,i) = result_homogeneous.at<double>(3,0);
	}

	cout << "size of reconstructed pts: " << (results_homogeneous.size()) <<endl;
	cout << "1st point: x: " << results_homogeneous.at<double>(0,0) 
		<< ", y: " << results_homogeneous.at<double>(1,0)
		<< ", z: " << results_homogeneous.at<double>(2,0) 
		<< ", w: " << results_homogeneous.at<double>(3,0) << endl;//*/



	/*	--------------------------------------------------------------------
		Save Triangulation results
		-------------------------------------------------------------------- */

	TriangleVec triangleVec;

	/*vector< my_point > vec_3d_ptsCUSTOM(number_matches);
	for(int i=0; i<number_matches; i++){
        my_point single_3d_ptCUSTOM;
		for(int j=0;j<3;j++){
			single_3d_ptCUSTOM._p[j] = results_homogeneous.at<double>(j,i);
		}

		vec_3d_ptsCUSTOM[i] = single_3d_ptCUSTOM;
	}
	const string ply_fileCUSTOM = "reconstructCUSTOM.ply";
	writeMeshToPLYFile(vec_3d_ptsCUSTOM,triangleVec,ply_fileCUSTOM);//*/

	//	Save the triangulated points into a vector
	vector<my_point> vec_3d_pts(number_matches);
	for(int i=0; i<number_matches; i++){
        my_point single_3d_pt;
		for(int j=0;j<3;j++){
			//single_3d_pt._p[j] = reconstructed_points.at<float>(0,i*4 + j);
			single_3d_pt._p[j] = reconstructed_points.at<float>(j,i);
		}

		vec_3d_pts[i] = single_3d_pt;
	}


	//	Write the triangulated points to a PLY file to visualize the results
	const string ply_file = "reconstruct.ply";
	writeMeshToPLYFile(vec_3d_pts,triangleVec,ply_file);

	//	Write the matches and fundamental matrix for further matlab analysis
	const string matches_file = "matches.txt";
	write_matched_features_to_file(matches_file, feat1, feat2, matches);
	const string fund_matches_file = "Fund_matches.txt";
	write_matched_features_to_file(fund_matches_file, feat1, feat2, matches,
								   25, true);
	const string fund_mat_file = "fundMat.txt";
	write_fundamental_matrix(fund_mat_file, fund_mat);
	//cout << "Fundamental Matrix: " << fund_mat << endl;
}
/* -------------------------------------------------------------------------- */

/*	----------------------------------------------------------------------------
	Compute new [rotation|translation] matrix compared to world frame
	Input:	-	Relative [R,t] between the two images
			-	[R,t] of first image of the pair compared to global frame 	  
	Output: Final rotation and translation matrix [R,t] in first frame 0
----------------------------------------------------------------------------- */
Mat_<double> get_RT_global_frame(Mat_<double> Rt12, Mat_<double> Rt01)
{
	//	Change Rt01 to a SE(3) [ Rotation (3x3) | translation (3,1)] matrix
	//						   [  0     0     0 |         1        ]
	Mat Adj01 = (Mat_<double>(4,4)<< Rt01(0,0), Rt01(0,1), Rt01(0,2), Rt01(0,3),
									 Rt01(1,0), Rt01(1,1), Rt01(1,2), Rt01(1,3),
									 Rt01(2,0), Rt01(2,1), Rt01(2,2), Rt01(2,3),
										     0,		    0,         0,        1);
	//	Extend Rt12 to SE(3)
	Mat_<double> Adj12 = (Mat_<double>(4,4) << 
									Rt12(0,0), Rt12(0,1), Rt12(0,2), Rt12(0,3),
									Rt12(1,0), Rt12(1,1), Rt12(1,2), Rt12(1,3),
									Rt12(2,0), Rt12(2,1), Rt12(2,2), Rt12(2,3),
											0,		   0,         0,        1);

	Mat_<double> Adj02 = Adj01 * Adj12;
	//Adj12 = Adj12 * Adj01;

	Mat_<double> Rt02 = (Mat_<double>(3,4) << 
						Adj02(0,0), Adj02(0,1), Adj02(0,2), Adj02(0,3),
						Adj02(1,0), Adj02(1,1), Adj02(1,2), Adj02(1,3),
						Adj02(2,0), Adj02(2,1), Adj02(2,2), Adj02(2,3));
	return Rt02;
}
/* -------------------------------------------------------------------------- */

/*	----------------------------------------------------------------------------
	For two pairs of images, computes similar matches between the three images
----------------------------------------------------------------------------- */
typedef struct ids_match
{
	int id_matched_01_1;
	int id_matched_12_1;
}IDS_MATCH;
IDS_MATCH get_3views_match(vector<my_match> vec_matches_01, vector<Point2f> best_pts1_12)
{
	int iter = 0;
	int nb_matches_01 = (int)vec_matches_01.size();
	int nb_matches_12 = (int)best_pts1_12.size();
	for(int i=0; i<nb_matches_01; i++)
	{
		//	See if it appears in second pair of images
		for(int j=0; j<nb_matches_12; j++)
		{
			iter++;
			// Check if the point in image1 appears in second pair of images
			if(vec_matches_01[i].point2 == best_pts1_12[j])
			{
				IDS_MATCH ids;
				ids.id_matched_01_1 = i;
				ids.id_matched_12_1 = j;
				cout << "nb iterations before finding 3 match" << iter << endl;
				return ids;
			}
		}
	}

	cout << "nb iterations without finding 3 match" << iter << endl;
	cerr << "No good matches are common between two pairs of images" << endl;
	IDS_MATCH ids;

	//	Returns too big identifiers which will crash the program
	ids.id_matched_01_1 = nb_matches_01;
	ids.id_matched_12_1 = nb_matches_12;
	return ids;
}
/* -------------------------------------------------------------------------- */


/*	---------------------- NOT USED IN ANY FUNCTION ----------------------------
	Function supposed to return the scale of 3d points. Does not currently work.
-------------------------- NOT USED IN ANY FUNCTION ------------------------- */
double scale_pts_checking(SFM sfm_previous, vector<Point2f> ordered_2d_pts_12_1, 
										   Mat_<double> my_reconstructed_points)
{
	//	Get a good match between the two pairs of images
	IDS_MATCH ids;
	ids = get_3views_match(sfm_previous.vec_matches, ordered_2d_pts_12_1);
	cout << "2d pt from 1st pair: (x,y)" << endl << "(" 
		 << sfm_previous.vec_matches[ids.id_matched_01_1].point2.x << "," 
		 << sfm_previous.vec_matches[ids.id_matched_01_1].point2.y << ")" 
		 << endl;
	cout << "1st pt from 2nd pair: (x,y)"<< endl << "(" 
		 << ordered_2d_pts_12_1[ids.id_matched_12_1].x << "," 
		 << ordered_2d_pts_12_1[ids.id_matched_12_1].y << ")" 
		 << endl;
	
	//	Get the 3d point common to all three images from first pair of images
	Mat_<double> matched_3d_pt_01(4, 1); 
	matched_3d_pt_01 = (Mat_<double>(4,1) <<
							sfm_previous.homog_3d_pts(0,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(1,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(2,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(3,ids.id_matched_01_1));
							/*sfm_previous.homog_3d_pts(ids.id_matched_12_1,0), 
							sfm_previous.homog_3d_pts(ids.id_matched_12_1,1), 
							sfm_previous.homog_3d_pts(ids.id_matched_12_1,2), 
							sfm_previous.homog_3d_pts(ids.id_matched_12_1,3));*/
	//	Get the 3d point common to all three images from second pair of images
	Mat_<double> matched_3d_pt_12(4, 1); 
	matched_3d_pt_12 = (Mat_<double>(4,1) << 
							my_reconstructed_points(0,ids.id_matched_12_1), 
							my_reconstructed_points(1,ids.id_matched_12_1), 
							my_reconstructed_points(2,ids.id_matched_12_1), 
							my_reconstructed_points(3,ids.id_matched_12_1));
	cout << "3dpt01 (x,y,z): (" << matched_3d_pt_01(0,0) << ", " 
							    << matched_3d_pt_01(1,0) << ", " 
							    << matched_3d_pt_01(2,0) << ") " << endl;
	cout << "3dpt12 (x,y,z): (" << matched_3d_pt_12(0,0) << ", " 
								<< matched_3d_pt_12(1,0) << ", " 
								<< matched_3d_pt_12(2,0) << ") " << endl;

	//	Compute the ratio in depth between the two 3d features
	double depth_scale = matched_3d_pt_01(2,0) / matched_3d_pt_12(2,0);

	return depth_scale;
}
/* -------------------------------------------------------------------------- */

/*	---------------------- NOT USED IN ANY FUNCTION ----------------------------
	Function supposed to return the scale of ẗranslation. Does not work.
-------------------------- NOT USED IN ANY FUNCTION ------------------------- */
double scale_translation(SFM sfm_previous, 
						vector<Point2f> ordered_2d_pts_12_1, 
						Mat_<double> my_reconstructed_points, 
						Mat_<double> Rt12)
{
	//	Get a good match between the two pairs of images
	IDS_MATCH ids;
	ids = get_3views_match(sfm_previous.vec_matches, ordered_2d_pts_12_1);
	
	//	Get the 3d point common to all three images from first pair of images
	Mat_<double> matched_3d_pt_01(4, 1); 
	matched_3d_pt_01 = (Mat_<double>(4,1) <<
							sfm_previous.homog_3d_pts(0,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(1,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(2,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(3,ids.id_matched_01_1));
	//	Get the 3d point common to all three images from second pair of images
	Mat_<double> matched_3d_pt_12(4, 1); 
	matched_3d_pt_12 = (Mat_<double>(4,1) << 
							my_reconstructed_points(0,ids.id_matched_12_1), 
							my_reconstructed_points(1,ids.id_matched_12_1), 
							my_reconstructed_points(2,ids.id_matched_12_1), 
							my_reconstructed_points(3,ids.id_matched_12_1));

	//	Compute the ratio in depth between the two 3d features
	Mat_<double> R12 = (Mat_<double>(4,4) << Rt12(0,0), Rt12(0,1), Rt12(0,2), 0,
											 Rt12(1,0), Rt12(1,1), Rt12(1,2), 0,
											 Rt12(2,0), Rt12(2,1), Rt12(2,2), 0,
											 		 0,         0,         0,1);
	Mat_<double> translated_3d_pt(4, 1);
	translated_3d_pt = (matched_3d_pt_01 - R12 * matched_3d_pt_12);
	cout << "translated_3d_pt" << translated_3d_pt << endl;

	double translation_scale = translated_3d_pt.at<double>(2,0) / Rt12(2,3);
	cout << "translation_scale" << translation_scale << endl;

	return translation_scale;
}
/* -------------------------------------------------------------------------- */


/*	---------------------- NOT USED IN ANY FUNCTION ----------------------------
	Function similar to scale_translation(). Does not work.
-------------------------- NOT USED IN ANY FUNCTION ------------------------- */
double my_scale_translation(SFM sfm_previous, 
							vector<Point2f> ordered_2d_pts_12_1, 
							Mat_<double> my_reconstructed_points, 
							Mat_<double> Rt12)
{
	//	1st pair of images
	Mat_<double> Rt01 = sfm_previous.Rt;
	Mat_<double> R01 = (Mat_<double>(3,3) << Rt01(0,0), Rt01(0,1), Rt01(0,2),
											 Rt01(1,0), Rt01(1,1), Rt01(1,2),
											 Rt01(2,0), Rt01(2,1), Rt01(2,2));
	Mat_<double> t01 = (Mat_<double>(3,1) << 	Rt01(0,3),
												Rt01(1,3),
												Rt01(2,3));
	Mat_<double> t01_inv = (Mat_<double>(3,1) << 
					-(R01(0,0)*t01(0,0)+R01(1,0)*t01(1,0)+R01(2,0)*t01(2,0)),
					-(R01(0,1)*t01(0,0)+R01(1,1)*t01(1,0)+R01(2,1)*t01(2,0)),
					-(R01(0,2)*t01(0,0)+R01(1,2)*t01(1,0)+R01(2,2)*t01(2,0)));
	Mat Adj01 = (Mat_<double>(4,4) << 	R01(0,0), R01(0,1), R01(0,2), t01(0,0),
										R01(1,0), R01(1,1), R01(1,2), t01(1,0),
										R01(2,0), R01(2,1), R01(2,2), t01(2,0),
											   0,	  	 0,        0,       1);
	Mat Adj01_inv = (Mat_<double>(4,4) << 
									R01(0,0), R01(1,0), R01(2,0), t01_inv(0,0),
									R01(0,1), R01(1,1), R01(2,1), t01_inv(1,0),
									R01(0,2), R01(1,2), R01(2,2), t01_inv(2,0),
			 							   0,	  	 0,        0,           1);
	//	2nd pair of images
	Mat_<double> R12 = (Mat_<double>(3,3) << Rt12(0,0), Rt12(0,1), Rt12(0,2),
											 Rt12(1,0), Rt12(1,1), Rt12(1,2),
											 Rt12(2,0), Rt12(2,1), Rt12(2,2));
	Mat_<double> t12 = (Mat_<double>(3,1) << 	Rt12(0,3),
												Rt12(1,3),
												Rt12(2,3));
	Mat_<double> t12_inv = (Mat_<double>(3,1) << 
					-(R12(0,0)*t12(0,0)+R12(1,0)*t12(1,0)+R12(2,0)*t12(2,0)),
					-(R12(0,1)*t12(0,0)+R12(1,1)*t12(1,0)+R12(2,1)*t12(2,0)),
					-(R12(0,2)*t12(0,0)+R12(1,2)*t12(1,0)+R12(2,2)*t12(2,0)));
	Mat Adj12 = (Mat_<double>(4,4) << 	R12(0,0), R12(0,1), R12(0,2), t12(0,0),
										R12(1,0), R12(1,1), R12(1,2), t12(1,0),
										R12(2,0), R12(2,1), R12(2,2), t12(2,0),
											   0,	  	 0,        0,       1);
	Mat Adj12_inv = (Mat_<double>(4,4) << 
									R12(0,0), R12(1,0), R12(2,0), t12_inv(0,0),
									R12(0,1), R12(1,1), R12(2,1), t12_inv(1,0),
									R12(0,2), R12(1,2), R12(2,2), t12_inv(2,0),
			 							   0,	  	 0,        0,           1);
	Mat MEGA_R12_inv = (Mat_<double>(4,4) << R12(0,0), R12(1,0), R12(2,0), 0,
											 R12(0,1), R12(1,1), R12(2,1), 0,
											 R12(0,2), R12(1,2), R12(2,2), 0,
			 							   			0,	  	  0,        0, 1);

	//	Get a good match between the two pairs of images
	IDS_MATCH ids;
	ids = get_3views_match(sfm_previous.vec_matches, ordered_2d_pts_12_1);
	
	//	Get the 3d point common to all three images from first pair of images
	Mat_<double> matched_3d_pt_01(4, 1); 
	matched_3d_pt_01 = (Mat_<double>(4,1) <<
							sfm_previous.homog_3d_pts(0,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(1,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(2,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(3,ids.id_matched_01_1));
	//	Get the 3d point common to all three images from second pair of images
	Mat_<double> matched_3d_pt_12(4, 1); 
	matched_3d_pt_12 = (Mat_<double>(4,1) << 
							my_reconstructed_points(0,ids.id_matched_12_1), 
							my_reconstructed_points(1,ids.id_matched_12_1), 
							my_reconstructed_points(2,ids.id_matched_12_1), 
							my_reconstructed_points(3,ids.id_matched_12_1));

	Mat_<double> translated_3d_pt(4, 1);
	translated_3d_pt = (Adj01 * matched_3d_pt_01 - 
						MEGA_R12_inv * matched_3d_pt_12);
	cout << "translated_3d_pt" << translated_3d_pt << endl;

	double translation_scale = translated_3d_pt.at<double>(2,0) / Rt12(2,3);
	cout << "translation_scale" << translation_scale << endl;

	return translation_scale;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to compute a scalar for the translation between two pairs of images
	Input:	- sfm structure of previous pair of images
			- Extracted 2D points in first image of second pair of images.
			Note: 	This points need to be ordered by precision with 
					get_most_precise_matches  
	Output: Scaler to scale the translation vector between the two pairs of imgs
----------------------------------------------------------------------------- */
double get_t_scaler(SFM sfm_previous, vector<Point2f> ordered_2d_pts_12_1, 
					Mat_<double> my_reconstructed_points)
{
	//	1st pair of images
	Mat_<double> Rt01 = sfm_previous.Rt;
	Mat_<double> R01 = (Mat_<double>(3,3) << Rt01(0,0), Rt01(0,1), Rt01(0,2),
											 Rt01(1,0), Rt01(1,1), Rt01(1,2),
											 Rt01(2,0), Rt01(2,1), Rt01(2,2));
	Mat_<double> t01 = (Mat_<double>(3,1) << 	Rt01(0,3),
												Rt01(1,3),
												Rt01(2,3));

	//	Get a good match between the two pairs of images
	IDS_MATCH ids;
	ids = get_3views_match(sfm_previous.vec_matches, ordered_2d_pts_12_1);
	
	//	Get the 3d point common to all three images from first pair of images
	Mat_<double> p_01(3, 1); 
	p_01 = (Mat_<double>(3,1) <<
							sfm_previous.homog_3d_pts(0,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(1,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(2,ids.id_matched_01_1)); 
							//sfm_previous.homog_3d_pts(3,ids.id_matched_01_1));
	//	Get the 3d point common to all three images from second pair of images
	Mat_<double> p_12(3, 1); 
	p_12 = (Mat_<double>(3,1) << 
							my_reconstructed_points(0,ids.id_matched_12_1), 
							my_reconstructed_points(1,ids.id_matched_12_1), 
							my_reconstructed_points(2,ids.id_matched_12_1));
							//my_reconstructed_points(3,ids.id_matched_12_1));
	Mat_<double> Rp = R01 * p_01; //(Mat_<double>(3,1) 

	//	See final report pdf on Github for full explanation
	double beta = (Rp(0,0)*p_12(1,0)/(p_12(0,0)*t01(1,0))-(Rp(1,0)/t01(1,0))) /
					(1 - ((t01(0,0)*p_12(1,0))/(p_12(0,0)*t01(1,0))));
	cout << "beta: " << beta << endl;

	return beta;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to compute a scalar for the reconstructed 3d points in
	second pair of images.
	Input:	- sfm structure of previous pair of images
			- Extracted 2D points in first image of second pair of images.
			- Reconstructed 3d points in second pair of images
			- beta: scaler for translation vector computer with get_t_scaler(..)
			Note: 	This points need to be ordered by precision with 
					get_most_precise_matches  
	Output: Scaler to scale the reconstructed 3d points in second pair of images
----------------------------------------------------------------------------- */
double get_pts_scaler(SFM sfm_previous, vector<Point2f> ordered_2d_pts_12_1, 
					  Mat_<double> my_reconstructed_points, double beta)
{
	//	1st pair of images
	Mat_<double> Rt01 = sfm_previous.Rt;
	Mat_<double> R01 = (Mat_<double>(3,3) << Rt01(0,0), Rt01(0,1), Rt01(0,2),
											 Rt01(1,0), Rt01(1,1), Rt01(1,2),
											 Rt01(2,0), Rt01(2,1), Rt01(2,2));
	Mat_<double> t01 = (Mat_<double>(3,1) << 	Rt01(0,3),
												Rt01(1,3),
												Rt01(2,3));


	//	Get a good match between the two pairs of images
	IDS_MATCH ids;
	ids = get_3views_match(sfm_previous.vec_matches, ordered_2d_pts_12_1);
	
	//	Get the 3d point common to all three images from first pair of images
	Mat_<double> p_01(3, 1); 
	p_01 = (Mat_<double>(3,1) <<
							sfm_previous.homog_3d_pts(0,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(1,ids.id_matched_01_1), 
							sfm_previous.homog_3d_pts(2,ids.id_matched_01_1)); 
							//sfm_previous.homog_3d_pts(3,ids.id_matched_01_1));
	//	Get the 3d point common to all three images from second pair of images
	Mat_<double> p_12(3, 1); 
	p_12 = (Mat_<double>(3,1) << 
							my_reconstructed_points(0,ids.id_matched_12_1), 
							my_reconstructed_points(1,ids.id_matched_12_1), 
							my_reconstructed_points(2,ids.id_matched_12_1));
							//my_reconstructed_points(3,ids.id_matched_12_1));
	Mat_<double> Rp = R01 * p_01;

	double alpha = (Rp(0,0) + beta * t01(0,0)) / p_12(0,0);
	cout << "alpha: " << alpha << endl;

	return alpha;
}
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
sfm features_twoview_triangulation(Mat fund_mat,
								   Mat img1, Mat img2, 
								   FEATURES feat1, FEATURES feat2,
								   vector<DMatch> matches, SFM sfm_previous)
{
	int number_matches = (int)matches.size();

	/*	In general, frame 0 is defined as the world frame, or frame of first cam
		, frame 1 corresponds to the first camera in new image pair and
				2 					 second camera   new image pair
		R01 corresponds to global   Rotation between world frame to first image
		t01 			   global   Translation
		R12 corresponds to relative Rotation    between 2 images in image pair
		t12 			   relative Translation between 2 images 			*/

	//	Extract previous rotation and translation matrices
	Mat_<double> Rt01 = sfm_previous.Rt;
	Mat_<double> R01 = (Mat_<double>(3,3) << Rt01(0,0), Rt01(0,1), Rt01(0,2),
											 Rt01(1,0), Rt01(1,1), Rt01(1,2),
											 Rt01(2,0), Rt01(2,1), Rt01(2,2));
	Mat_<double> t01 = (Mat_<double>(3,1) << 	Rt01(0,3),
												Rt01(1,3),
												Rt01(2,3));
	Mat_<double> t01_inv = (Mat_<double>(3,1) << 
		-(R01(0,0)*t01(0,0)+R01(1,0)*t01(1,0)+R01(2,0)*t01(2,0)),
		-(R01(0,1)*t01(0,0)+R01(1,1)*t01(1,0)+R01(2,1)*t01(2,0)),
		-(R01(0,2)*t01(0,0)+R01(1,2)*t01(1,0)+R01(2,2)*t01(2,0)));
	Mat Adj01 = (Mat_<double>(4,4) << 	R01(0,0), R01(0,1), R01(0,2), t01(0,0),
										R01(1,0), R01(1,1), R01(1,2), t01(1,0),
										R01(2,0), R01(2,1), R01(2,2), t01(2,0),
											   0,	  	 0,        0,       1);
	//	Adj^-1 = [Rt,-Rt*t]		is used since the reconstructed points are
	// 			 [000,   1]		expressed in the body frame (2nd imgs pair)
	Mat_<double> Adj01_inv = (Mat_<double>(4,4) << 
									R01(0,0), R01(1,0), R01(2,0), t01_inv(0,0),
									R01(0,1), R01(1,1), R01(2,1), t01_inv(1,0),
									R01(0,2), R01(1,2), R01(2,2), t01_inv(2,0),
			 							   0,	  	 0,        0,           1);

	//--------------------------------------------------------------------------
	//	Computation of relative rotation and translation between the two images

	//	Computation of the Essential Matrix (K is defined in the header)
	Mat ess_mat = K.t() * fund_mat * K;


	/* -------------FURTHER DEVELOPMENT-----------------------------------------
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): 
	Run LM on inliers to refine (R, t): see (Last visited 2 May 2017)
		http://www.cad.zju.edu.cn/home/gfzhang/training/SFM/SfM.html
	------------------------------------------------------------------------- */

	
	//	Get most precise match to check P2 (since there are 4 solutions given E)
	vector<my_match> vec_matches(number_matches);
	vector<Point2f> ordered_2d_pts1(number_matches);	//	Ordered by precision
	vector<Point2f> ordered_2d_pts2(number_matches);
	vector<Point2f> best_pt1(1);
	vector<Point2f> best_pt2(1);

	vec_matches = get_most_precise_matches(feat1, feat2, matches);
	best_pt1[0] = vec_matches[0].point1;
	best_pt2[0] = vec_matches[0].point2;
	for(int i=0; i<number_matches; i++){
		ordered_2d_pts1[i] = vec_matches[i].point1;
		ordered_2d_pts2[i] = vec_matches[i].point2;
	}

	//	Refine matches
	vector<Point2f> corrected_2d_pts1(number_matches);
	vector<Point2f> corrected_2d_pts2(number_matches);
	cv::correctMatches( fund_mat, ordered_2d_pts1, ordered_2d_pts2,
						corrected_2d_pts1, corrected_2d_pts2 );

	//	Get [R|t] from most precise match, essential and intrinsics matrices
	Mat_<double> Rt12=get_Rt_from_essential_mat(ess_mat, K, best_pt1, best_pt2);
	cout << "fresh new Rt12: " << Rt12 << endl;

	//	Triangulation of 3d points
	cv::Mat reconstructed_3d_pts(1, number_matches ,CV_32FC4);
	Mat P1 =  K * (Mat_<double>(3,4) <<  1,     0,     0,     0,
								       	 0,     1,     0,     0,
								       	 0,     0,     1,     0);
	triangulatePoints(P1, K*Rt12, corrected_2d_pts1, 
					  corrected_2d_pts2, reconstructed_3d_pts);

	//	Ensure that the reconstruction is in front for the complete SFM
	if(Rt12(0,0) < 0)
	{
		Rt12(0,0) *= -1; Rt12(0,1) *= -1; Rt12(0,2) *= -1;
		Rt12(1,0) *= -1; Rt12(1,1) *= -1; Rt12(1,2) *= -1;
		Rt12(2,0) *= -1; Rt12(2,1) *= -1; Rt12(2,2) *= -1;
		cout << "bad sign for Rt12 (rotation part), multiply R by (-1)" << endl;
	}
	if(Rt12(0,3) < 0)
	{
		Rt12(0,3) *= -1;
		Rt12(1,3) *= -1;
		Rt12(2,3) *= -1;
		cout << "bad sign for Rt12 (translation), multiply rotation part by *(-1)" << endl;
	}	//not necessary: beta scaler solves this issue as well 

	//	Set last coordinate of homogeneous representation to 1 (w=1)
	Mat_<double> pts_3d(4, number_matches);
	for (int i=0; i<number_matches; i++)
	{
		//	Set last coordinate of homogeneous coordinates to 1 (w=1)
		float scale = reconstructed_3d_pts.at<float>(3,i);
		reconstructed_3d_pts.at<float>(0,i) /= scale;
		reconstructed_3d_pts.at<float>(1,i) /= scale;
		reconstructed_3d_pts.at<float>(2,i) /= scale;
		reconstructed_3d_pts.at<float>(3,i) /= scale;

		pts_3d(0,i) = (double)reconstructed_3d_pts.at<float>(0,i);
		pts_3d(1,i) = (double)reconstructed_3d_pts.at<float>(1,i);
		pts_3d(2,i) = (double)reconstructed_3d_pts.at<float>(2,i);
		pts_3d(3,i) = (double)reconstructed_3d_pts.at<float>(3,i);
	}

	//	Return good structure
	SFM sfm_current;
	sfm_current.vec_matches = vec_matches;	//	ordered matches (my_match) according with most precise first
	//	Add rotation and translation compared to previous images
	Mat_<double> Rt02 = get_RT_global_frame(Rt12, Rt01);
	sfm_current.Rt = Rt02;

	//	Do not try this if there was no previous image pair (first pair)
	if (Rt01(0,0)==1 && Rt01(0,1)==0 && Rt01(0,2)==0 && Rt01(0,3)==0 &&
		Rt01(1,0)==0 && Rt01(1,1)==1 && Rt01(1,2)==0 && Rt01(1,3)==0 &&
		Rt01(2,0)==0 && Rt01(2,1)==0 && Rt01(2,2)==1 && Rt01(2,3)==0)
		sfm_current.homog_3d_pts = pts_3d; 	//no scale correction
	else
	{
		/*	Since t12 (translation between 2 new images) is normalized to 1,
			It is necessary to recover good scale compared to last pair.
			Hence, we compare the best recovered 3d points in two pairs to
			recover the scale of both the 3d points and rotation
			Hence,				p3d = alpha * p3d
								t12 = beta  * t12 							*/

		double beta =  get_t_scaler  (sfm_previous, ordered_2d_pts1, pts_3d);
		double alpha = get_pts_scaler(sfm_previous, ordered_2d_pts1, pts_3d, 
									  beta);
		//	apply scalers to translation and points and change to world origin
		for (int i=0; i<number_matches; i++)
		{
			pts_3d(0,i) *= alpha;
			pts_3d(1,i) *= alpha;
			pts_3d(2,i) *= alpha;
		}
		Mat_<double> Adj01_inv = (Mat_<double>(4,4) << 
							R01(0,0), R01(1,0), R01(2,0), beta * t01_inv(0,0),
							R01(0,1), R01(1,1), R01(2,1), beta * t01_inv(1,0),
							R01(0,2), R01(1,2), R01(2,2), beta * t01_inv(2,0),
	 							   0,	  	 0,        0,           1);
		pts_3d = Adj01_inv * pts_3d;

		Mat_<double> R01 = (Mat_<double>(3,4) << 
							Rt01(0,0), Rt01(0,1), Rt01(0,2), beta * Rt01(0,3),
							Rt01(1,0), Rt01(1,1), Rt01(1,2), beta * Rt01(1,3),
							Rt01(2,0), Rt01(2,1), Rt01(2,2), beta * Rt01(2,3));

													
		//	Refine the depth of 3D points
		/*double depth_scale = scale_pts_checking(sfm_previous, ordered_2d_pts1, 
														pts_3d);
		cout << "depth_scale" << depth_scale << endl;
		for(int i=0; i<number_matches; i++)
		{
			pts_3d(0,i) *= abs(depth_scale);
			pts_3d(1,i) *= abs(depth_scale);
			pts_3d(2,i) *= abs(depth_scale);
			//pts_3d(3,i) *= abs(depth_scale);
		}*/

		sfm_current.homog_3d_pts = pts_3d;

		Rt02 = get_RT_global_frame(Rt12, Rt01);
		sfm_current.Rt = Rt02;
	}

	cout << "Adj01_inv" << Adj01_inv << endl;
	cout << "Rt01: " << Rt01 << endl << 
			"Rt12: " << Rt12 << endl << 
			"Rt02: " << Rt02 << endl;
	cout << "(t.x, t.y, t.z) " << endl << "(" << sfm_current.Rt(0,3) << "," <<
			sfm_current.Rt(1,3) << "," << sfm_current.Rt(2,3) << ")" << endl;


	//	------------------------------------------------------------------------
	// 	Output to .ply for visualization
	TriangleVec triangleVec;

	vector< my_point > vec_3d_pts(number_matches);
	for(int i=0; i<number_matches; i++){
        my_point single_3d_pt;
		for(int j=0;j<3;j++){
			single_3d_pt._p[j] = pts_3d.at<double>(j,i);
		}

		vec_3d_pts[i] = single_3d_pt;
	}

	//	Write the triangulated points to a PLY file to visualize the results
	const string ply_file = "reconstruct.ply";
	//	If first pair of feature (no previous rotation), create new PLY file
	if (Rt01(0,0)==1 && Rt01(0,1)==0 && Rt01(0,2)==0 && Rt01(0,3)==0 &&
		Rt01(1,0)==0 && Rt01(1,1)==1 && Rt01(1,2)==0 && Rt01(1,3)==0 &&
		Rt01(2,0)==0 && Rt01(2,1)==0 && Rt01(2,2)==1 && Rt01(2,3)==0)
		writeMeshToPLYFile(vec_3d_pts,triangleVec,ply_file);
	else
		add_MeshToPLYFile(vec_3d_pts,triangleVec,ply_file);

	//	Write the matches and fundamental matrix for further matlab analysis
	const string matches_file = "matches.txt";
	write_matched_features_to_file(matches_file, feat1, feat2, matches);
	const string fund_matches_file = "Fund_matches.txt";
	write_matched_features_to_file(fund_matches_file, feat1, feat2, matches,
								   25, true);
	const string fund_mat_file = "fundMat.txt";
	write_fundamental_matrix(fund_mat_file, fund_mat);

	return sfm_current;
}
/* -------------------------------------------------------------------------- */
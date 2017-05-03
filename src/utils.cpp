/*	----------------------------------------------------------------------------
	Comp5421 Computer Vision project, Spring 2017, Thomas Lew
	Last update: 3 May 2017
	
	Implements functions for display and utilities.
    ------------------------------------------------------------------------- */
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>		//	For imshow and similar functions
#include <opencv2/nonfree/features2d.hpp>	//	For <Dmatch> structure
#include <stack>							//	For tic(), toc().
#include <ctime>							//	For tic(), toc().

#include "utils.h"

using namespace std;
using namespace cv;

/*	----------------------------------------------------------------------------
	Function to write 3D points into a ply file.		
----------------------------------------------------------------------------- */
void writeMeshToPLYFile(const PointVec& pointVec,
						const TriangleVec& triangleVec,
						const string& outFilename)
{
	ofstream outFile(outFilename.c_str());
    	//ofstream outFile(outFilename.c_str(), ofstream::app);

    if (!outFile)
    {
        cerr << "Error opening output file: " << outFilename << "!" << endl;
        exit(1);	//	Using exit is bad, should be removed
    }

    const int pointNum    = (int) pointVec.size();
    const int triangleNum = (int) triangleVec.size();

    ////
    // Header
    ////
	outFile << "ply" << endl;
	outFile << "format ascii 1.0" << endl;
	outFile << "element vertex " << pointNum << endl;
	outFile << "property float x" << endl;
	outFile << "property float y" << endl;
	outFile << "property float z" << endl;
	outFile << "element face " << triangleNum << endl;
	outFile << "property list uchar int vertex_index" << endl;
	outFile << "end_header" << endl;

    ////
    // Points
    ////
    for (int pi = 0; pi < pointNum; ++pi)
    {
        const my_point& point = pointVec[pi];

        for(int vi = 0; vi < 3; ++vi)
            outFile << point._p[vi] << " ";

        outFile << endl;
    }

    ////
    // Triangles
    ////
    for (int ti = 0; ti < triangleNum; ++ti)
    {
        const my_triangle& triangle = triangleVec[ti];
        outFile << "3 ";

        for(int vi = 0; vi < 3; ++vi)
            outFile << triangle._v[vi] << " ";

        outFile << endl;
    }

    return;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to add 3D points into a ply file.		
----------------------------------------------------------------------------- */
void  add_MeshToPLYFile(const PointVec& pointVec,
						const TriangleVec& triangleVec,
						const string& outFilename)
{
	//	----------------------------------------------------------------------
	//	First, recopy file into temporary file with changed number of vertices
	//	----------------------------------------------------------------------
	string temp_string = "temp";
	ofstream outFile(temp_string.c_str());

    if (!outFile)
    {
        cerr << "Error opening output file: " << outFilename << "!" << endl;
        exit(1);	//	Using exit is bad, should be removed
    }

    const int pointNum    = (int) pointVec.size();
    const int triangleNum = (int) triangleVec.size();

    // 	No need for header (recopied), we only add 3d points to the ply file
    //
	//	Recopy file and change the number of points (we add more points)
	string x;
	ifstream file( outFilename.c_str() );
	//ofstream ofile( "test2.txt" );
	while (!file.eof())
	{
	    getline(file,x);
	    if (x.find("element vertex ") != std::string::npos)	//contains 
	    {
	    	//	Extract number of current poins
	    	string element;
	    	string vertex;
			int previous_num_pts;
			stringstream ss(x);
			ss >> element >> vertex >> previous_num_pts;

	        outFile << "element vertex " + 
	        			to_string(previous_num_pts + pointNum) << endl;
	    }
	    else
	    {	//	Do not add an additionnal line at the end
	    	if(!file.eof())
	    		outFile << x << endl;
	    	else
	    		outFile << x;
	    }
	}
	file.close();
	outFile.close();


	//	----------------------------------------------
	//	Recopy this file into the original one (erase)
	//	----------------------------------------------
	ifstream  in_originalFile(temp_string.c_str());
	ofstream out_originalFile(outFilename.c_str());
	while (!in_originalFile.eof())
	{
		getline(in_originalFile,x);
		//	Do not add an additionnal line at the end
		if(!in_originalFile.eof())
			out_originalFile << x << endl;
		else
			out_originalFile << x;
	}
	in_originalFile.close();
	out_originalFile.close();


	//	----------------------------------------------------
	//	Add new features into the final file (original file)
	//	----------------------------------------------------
    ofstream appoutFile(outFilename.c_str(), ofstream::app);
    // 	Points
    for (int pi = 0; pi < pointNum; ++pi)
    {
        const my_point& point = pointVec[pi];

        for(int vi = 0; vi < 3; ++vi)
            appoutFile << point._p[vi] << " ";

        appoutFile << endl;
    }

    // 	Triangles
    for (int ti = 0; ti < triangleNum; ++ti)
    {
        const my_triangle& triangle = triangleVec[ti];
        appoutFile << "3 ";

        for(int vi = 0; vi < 3; ++vi)
            appoutFile << triangle._v[vi] << " ";

        appoutFile << endl;
    }

    return;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to save the matched features into an external file
	Currently used for further analysis with Matlab
----------------------------------------------------------------------------- */
void write_matched_features_to_file(const string& outFilename, FEATURES feat1, 
								   	FEATURES feat2, vector<DMatch> matches,
								   	int nb_out_matches, bool sorted)
{
	//	Check and open the file if it exists. Exits the function if not.
	ofstream outFile(outFilename.c_str() );
    if (!outFile)
    {
        cerr << "ERROR opening " << outFilename << " to save matches!" << endl;
        return;
    }

    if(nb_out_matches < 0)
    {
    	cerr << "Wrong number of matches to output to file!" << endl;
    	return;
    }

    /*	Retrieve and output matched features to the file

    	Structure of the file:
		1st match: 		point1.x, point1.y;
		2nd match: 		point1.x, point1.y;
		3rd .....		.........

		1st match:		point2.x, point2.y;	
		2nd match:		point2.x, point2.y;	
		3rd .....		.........			 	
															*/
	int number_matches = (int)matches.size();

	if(sorted)
	{
		//	Sort these matches to get the smallest distances first
		vector<my_match> vec_matches(number_matches);
		vec_matches = get_most_precise_matches(feat1, feat2, matches);

	    //	Use only the 50 best matches for the fundamental matrix computation
	    if(nb_out_matches == 0)
	    	nb_out_matches = number_matches;
		vector<Point2f> points1(nb_out_matches);
		vector<Point2f> points2(nb_out_matches);

		for (int i = 0; i < nb_out_matches; i++)
	    {
	    	//	Retrieve matched features
		    points1[i] = vec_matches[i].point1;

	        //	Output the feature coordinates to the file 
	    	outFile << points1[i].x << " " << points1[i].y << " ";

	        //	Add \n
	        outFile << endl;
	    }
	    for (int i = 0; i < nb_out_matches; i++)
	    {
	    	//	Retrieve matched features
		    points2[i] = vec_matches[i].point2;

	        //	Output the feature coordinates to the file 
	    	outFile << points2[i].x << " " << points2[i].y << " ";

	        //	Add \n
	        outFile << endl;
	    }
	}
	else	//	Case without sorting the matches
	{
		vector<Point2f> points1(number_matches);
		vector<Point2f> points2(number_matches);

	    for (int i = 0; i < number_matches; i++)
	    {
	    	//	Retrieve matched features
		    points1[i] = feat1.keypoints[matches[i].queryIdx].pt;

	        //	Output the feature coordinates to the file 
	    	outFile << points1[i].x << " " << points1[i].y << " ";

	        //	Add \n
	        outFile << endl;
	    }
	    for (int i = 0; i < number_matches; i++)
	    {
	    	//	Retrieve matched features
		    points2[i] = feat2.keypoints[matches[i].trainIdx].pt;

	        //	Output the feature coordinates to the file 
	    	outFile << points2[i].x << " " << points2[i].y << " ";

	        //	Add \n
	        outFile << endl;
	    }
	}
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to save the fundamental matrix into an external file
	Currently used for further analysis with Matlab
----------------------------------------------------------------------------- */
void write_fundamental_matrix(const string& outFilename, Mat _F)
{
	//	Check and open the file if it exists. Exits the function if not.
	ofstream outFile(outFilename.c_str());
    if (!outFile)
    {
        cerr << "ERROR opening " << outFilename << 
        		" to save fundamental matrix!" << endl;
        return;
    }

    //	Check if the fundamental matrix is indeed 3 x 3
    if (_F.size() != Size(3,3))
    {
        cerr << "ERROR Copying the fundamental matrix: it isn't 3 x 3!" << endl;
        return;
    }

    //	Copy the fundamental matrix (3x3) into the file
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
	    {
	        //	Output the fundamental matrix to the file 
	    	outFile << _F.at<double>(i,j) << " ";
	    }

        //	Add \n
        outFile << endl;
	}

	return;
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to display an image with good dimensions		
----------------------------------------------------------------------------- */
void opencv_show_image(char* image_name, Mat image_content){
    namedWindow(image_name, WINDOW_NORMAL);
	imshow(image_name, image_content);
	resizeWindow(image_name, WIDTH, HEIGHT);
}
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
//	Tic-Toc implementation similar to Matlab functions. From
//	http://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
----------------------------------------------------------------------------- */
std::stack<clock_t> tictoc_stack;
void tic() {
    tictoc_stack.push(clock());
}
void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}
//	----------------------------------------------------------------------------

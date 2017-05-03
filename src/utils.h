/*	----------------------------------------------------------------------------
	Comp5421 Computer Vision project, Spring 2017, Thomas Lew
	Last update: 3 May 2017
	
	This module implements matrices and features output, image display and timer
    ------------------------------------------------------------------------- */


#ifndef utils_h
#define utils_h


#include "features.h"	//	For structures definitions

/*	----------------------------------------------------------------------------
	Functions and structures used to output triangulated points to a PLY file. 
	Retrieved from	https://codeyarns.com/2011/08/18/output-3d-mesh-to-ply-file/ 
	Note TODO: Use of "exit(1)" in writeMeshToPLYFile should be removed		
----------------------------------------------------------------------------- */
struct my_point
{
    float _p[3];
};
typedef vector<my_point> PointVec;

struct my_triangle
{
    float _v[3];
};
typedef vector<my_triangle> TriangleVec;


/*	----------------------------------------------------------------------------
	Function to write and add 3D points into a ply file.		
----------------------------------------------------------------------------- */
void writeMeshToPLYFile(const PointVec& pointVec,
						const TriangleVec& triangleVec,
						const string& outFilename);
/* -------------------------------------------------------------------------- */
void  add_MeshToPLYFile(const PointVec& pointVec,
						const TriangleVec& triangleVec,
						const string& outFilename);
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to save the matched features into an external file
	Currently used for further analysis with Matlab
----------------------------------------------------------------------------- */
void write_matched_features_to_file(const string& outFilename, FEATURES feat1, 
								   	FEATURES feat2, vector<DMatch> matches,
								   	int nb_out_matches = 0, bool sorted =false);
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to save the fundamental matrix into an external file
	Currently used for further analysis with Matlab
----------------------------------------------------------------------------- */
void write_fundamental_matrix(const string& outFilename, Mat _F);
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
	Function to display an image with good dimensions		
----------------------------------------------------------------------------- */
void opencv_show_image(char* image_name, Mat image_content);
/* -------------------------------------------------------------------------- */


/*	----------------------------------------------------------------------------
//	Tic-Toc implementation similar to Matlab functions
----------------------------------------------------------------------------- */
void tic();
void toc();
/* -------------------------------------------------------------------------- */


#endif
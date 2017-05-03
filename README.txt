/*	----------------------------------------------------------------------------
	Comp5421 Computer Vision project, Spring 2017, Thomas Lew
	Last update: 3 May 2017
    ------------------------------------------------------------------------- */

To compile and run on Ubuntu 14.04
	(~/build/comp5421$ cmake ..)
	~/build/comp5421$ make
	~/build/comp5421$ ./bin/cvision geometry ../fountain_dense/urd/0000.png ../fountain_dense/urd/0001.png


How to execute the program:

-> For features detection:
~/comp5421/build/bin$ ./cvision detection ../../fountain_dense/urd/0002.png

-> For features detection and matching:
~/comp5421/build/bin$ ./cvision matching ../../fountain_dense/urd/0000.png ../../fountain_dense/urd/0001.png

-> For features triangulation:
~/comp5421/build/bin$ ./cvision triangulation ../../fountain_dense/urd/0000.png ../../fountain_dense/urd/0001.png

-> For multiple 3d scenes reconstruction, structure from motion:
./bin/cvision sfm ../fountain_dense/urd/0000.png ../fountain_dense/urd/0001.png ../fountain_dense/urd/0002.png ../fountain_dense/urd/0003.png ../fountain_dense/urd/0004.png ../fountain_dense/urd/0005.png ../fountain_dense/urd/0006.png ../fountain_dense/urd/0007.png ../fountain_dense/urd/0008.png ../fountain_dense/urd/0009.png ../fountain_dense/urd/0010.png


How to use the Matlab code:

To vizualize the reconstructed features produced by "sfm" or "triangulation", run:
>> display_cpp_result_ply

To reproduce a 3D reconstruction based on the last two images used in "sfm" or "triangulation", run:
>> reconstruct

To test reconstruction algorithms and code on a simple model, run:
>> TestFile


For more information about the project, see the final report (final_report.pdf)
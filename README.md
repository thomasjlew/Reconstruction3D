# Reconstruction3D
HKUST Computer Vision course (Spring 2017 - COMP5421) project. 
<br />
Recovers a 3D scene from multiple uncalibrated images. 
<br />
<br />

To compile and run on Ubuntu 14.04<br />
```
~/build/comp5421$ make
```
<br />
<br />

How to execute the program:<br />
<br />
-> For features detection:<br />
```
~/comp5421/build/bin$ ./cvision detection ../../fountain_dense/urd/0002.png
```
-> For features detection and matching:<br />
```
~/comp5421/build/bin$ ./cvision matching ../../fountain_dense/urd/0000.png ../../fountain_dense/urd/0001.png
```
-> For features detection, matching, fundamental matrix computation and epipolar lines drawing:<br />
```
~/build/comp5421$ ./bin/cvision geometry ../fountain_dense/urd/0000.png ../fountain_dense/urd/0001.png
```
-> For features triangulation:<br />
```
~/comp5421/build/bin$ ./cvision triangulation ../../fountain_dense/urd/0000.png ../../fountain_dense/urd/0001.png
```
-> For multiple 3d scenes reconstruction, structure from motion:<br />
```
./bin/cvision sfm ../fountain_dense/urd/0000.png ../fountain_dense/urd/0001.png ../fountain_dense/urd/0002.png ../fountain_dense/urd/0003.png ../fountain_dense/urd/0004.png ../fountain_dense/urd/0005.png ../fountain_dense/urd/0006.png ../fountain_dense/urd/0007.png ../fountain_dense/urd/0008.png ../fountain_dense/urd/0009.png ../fountain_dense/urd/0010.png
```

<br />
How to use the Matlab code in Matlab:<br />
<br />
To vizualize the reconstructed features produced by "sfm" or "triangulation", run:<br />
```
display_cpp_result_ply
```
To reproduce a 3D reconstruction based on the last two images used in "sfm" or "triangulation", run:<br />
```
reconstruct
```
To test reconstruction algorithms and code on a simple model, run:<br />
```
TestFile
```

<br />
For more information about the project, see the final report (final_report.pdf)

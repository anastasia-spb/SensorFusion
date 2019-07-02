Extended Kalman Filter Project
==============================

Self-Driving Car Engineer Nanodegree Program

Overview
--------

The goal of this project is to understand and implement an Extended
Kalman Filter. This filter will be used to implement the state of a
moving object based on simulated lidar and radar measurements. Data from
sensors are preprocessed and as an input we receive object position. As
a performance metric the root mean square error (RMSE) is used.

Details About Project Files
---------------------------

The source files were copied from Udacity repository:
<a href="https://github.com/udacity/CarND-Extended-Kalman-Filter-Project" class="uri">https://github.com/udacity/CarND-Extended-Kalman-Filter-Project</a>

The main file was modified in order to read data from
`obj_pose-laser-radar-synthetic-input.txt` file. The reading
functionality was overtaken from Kalman Filter lab.

Implementation of methods was added into following files: \*
`FusionEKF.cpp` \* `kalman_filter.cpp` \* `Tools.cpp`

After build on local machine, files were uploaded into Udacity workspace
and tested using Simulation.

### `Docs`

This folder contains input data format description.

### `data`

Contains test data.

Each row represents a sensor measurement. Detailed description is placed
in `Docs\Data_Flow_Doc.txt`.

### `CMakeLists.txt`

Contains the input to the CMake build system.

### `generate_VS_15_2_10.bat`

Script for generating Visual Studio 2015 Project setting v140 build
tools.

### `src`

Contains project source files.

#### `Eigen`

C++ template library for linear algebra:
<a href="http://eigen.tuxfamily.org/index.php?title=Main_Page#Overview" class="uri">http://eigen.tuxfamily.org/index.php?title=Main\_Page\#Overview</a>

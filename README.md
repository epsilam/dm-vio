<h1 align="center">DM-VIO: Delayed Marginalization<br/> Visual-Inertial Odometry </h1>

<p align="center">
<a href="https://vision.in.tum.de/_media/research/vslam/dm-vio/dm-vio.pdf">Paper</a> | <a href="https://youtu.be/7iep3BvcJPU">Video</a> | <a href="http://vision.in.tum.de/dm-vio">Project Page</a>
</p>

When using this project in academic work, please consider citing:

    @article{stumberg22dmvio,
      author = {L. von Stumberg and D. Cremers},
      title = {{DM-VIO}: Delayed Marginalization Visual-Inertial Odometry},
      journal = {{IEEE} Robotics and Automation Letters ({RA-L})},
      year = {2022},
      volume = {7},
      number = {2},
      pages = {1408-1415},
      doi = {10.1109/LRA.2021.3140129}
    }

## New: ROS version and Live demo for Realsense cameras
* **Update May 23, 2023**: The ROS wrapper now supports running DM-VIO directly on rosbags, see [here](https://github.com/lukasvst/dm-vio-ros#new-now-you-can-also-run-in-non-realtime-mode-on-rosbags). This will run in non-realtime mode, but in practice it is usually much faster than real-time.
* **Update Jun 22, 2022**: There is a ROS wrapper for DM-VIO, available at https://github.com/lukasvst/dm-vio-ros
* **Update Jun 15, 2022**: Now there is a live demo for Realsense cameras. See [doc/RealsenseLiveVersion.md](doc/RealsenseLiveVersion.md) for details. The page also contains interesting tips for improving performance on custom datasets.
  * Note that it's not possible anymore to pass IMU noise values with the `camchain.yaml`, you need to use the `settings.yaml` file or commandline args.

### 1. Related Papers
* **[DM-VIO: Delayed Marginalization Visual-Inertial Odometry](https://vision.in.tum.de/dm-vio)**, L. von Stumberg and D. Cremers, In IEEE Robotics and Automation Letters (RA-L), volume 7, 2022
* **[Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization](https://vision.in.tum.de/vi-dso)**, L. von Stumberg, V. Usenko and D. Cremers, In International Conference on Robotics and Automation (ICRA), 2018
* **[Direct Sparse Odometry](https://vision.in.tum.de/dso)**, *J. Engel, V. Koltun, D. Cremers*, In  TPAMI, vol. 40, 2018

### 2. Installation

	git clone https://github.com/lukasvst/dm-vio.git

The following instructions have been tested with Ubuntu 20.04.
The system is also known to work well on Ubuntu 16.04, 18.04 and MacOS Big Sur (only Intel Macs have been tested so far).

#### 2.1 Required Dependencies

##### Suitesparse, Eigen3, Boost, yaml-cpp (required).
Required, install with

    sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libboost-all-dev libyaml-cpp-dev

On MacOS we recommend Homebrew to install the dependencies. It might be necessary
to install boost@1.60 instead of the newest boost, in order for the used GTSAM version to work.


##### GTSAM (required).
Build from source with

    sudo apt install libtbb-dev
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    git checkout 4.2a6          # newer gtsam versions might not work.
    mkdir build && cd build
    cmake -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
    make -j
    sudo make install

(Note: It seems like the keyframe operations are 2-3% slower with this GTSAM version compared to an older commit. To
reproduce the realtime paper results you should use commit `a738529af9754c7a085903f90ae8559bbaa82e75` of GTSAM).

##### OpenCV.
Used to read, write and display images.
Install with

	sudo apt-get install libopencv-dev


##### Pangolin.
Like for DSO, this is used for the GUI. You should install v0.6.
Install from [https://github.com/stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin)


	sudo apt install libgl1-mesa-dev libglew-dev pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
	git clone https://github.com/stevenlovegrove/Pangolin.git
	cd Pangolin
	git checkout v0.6
	mkdir build
	cd build
	cmake ..
	cmake --build .
	sudo make install



#### 2.2 Recommended Dependencies

##### Librealsense
This is necessary for the live demo for Realsense cameras. See
[doc/RealsenseLiveVersion.md](doc/RealsenseLiveVersion.md) for details

##### GTest (optional).
For running tests, install with `git submodule update --init`.

##### ziplib (optional).
Used to read datasets with images as .zip.
See [src/dso/README.md](src/dso/README.md) for instructions.

##### sse2neon (required for ARM builds).
After cloning, run `git submodule update --init` to include this.

#### 2.3 Build

    cd dm-vio
    mkdir build
    cd build
    cmake ..
    make -j

This compiles `dmvio_dataset` to run DM-VIO on datasets (needs both OpenCV and Pangolin installed).
It also compiles the library `libdmvio.a`, which other projects can link to.

#### Trouble-Shooting
The project is based on DSO and only has two additional dependencies with GTSAM and yaml-cpp.
In case of problems with compilation we recommend trying to compile https://github.com/JakobEngel/dso
first and seeing if it works.

### 3 Running
Download a TUM-VI sequence (download in the format `Euroc / DSO 512x512`) at https://vision.in.tum.de/data/datasets/visual-inertial-dataset

    bin/dmvio_dataset
        files=XXXX/datasetXXXX/dso/cam0/images
        vignette=XXXX/datasetXXXX/dso/cam0/vignette.png
        imuFile=XXXX/datasetXXXX/dso/imu.txt
        gtFile=XXXX/datasetXXXX/dso/gt_imu.csv
        tsFile=XXXX/datasetXXXX/dso/cam0/times.txt
        calib=PATH_TO_DMVIO/configs/tumvi_calib/camera02.txt
        gamma=PATH_TO_DMVIO/configs/tumvi_calib/pcalib.txt
        imuCalib=PATH_TO_DMVIO/configs/tumvi_calib/camchain.yaml
        mode=0
        use16Bit=1
        preset=0                                                        # use 1 for realtime
        nogui=0                                                         # use 1 to enable GUI
        resultsPrefix=/PATH_TO_RESULTS/
        settingsFile=PATH_TO_DMVIO/configs/tumvi.yaml
        start=2

Instead of typing this long command you can use the [python tools](https://github.com/lukasvst/dm-vio-python-tools).

#### Running on EuRoC, 4Seasons, reproducing paper results, etc.
We **strongly recommend** using the python-dm-vio tools published at: https://github.com/lukasvst/dm-vio-python-tools

They can be used to
* prepare the EuRoC and 4Seasons sequences for usage with DM-VIO.
* run on all (or some) sequences of EuRoC, TUM-VI and 4Seasons and gather the results.
* create a Python evaluation script for inspecting the results and generating the plots shown in the paper.

#### Commandline arguments
There are two types of commandline arguments:
1. Main arguments defined `in util/MainSettings.cpp` (see `parseArgument` and `registerArgs`). Most of these are derived from
DSO, so you can read [src/dso/README.md](src/dso/README.md) for documentation on them.
2. Lots of additional settings are defined using the `SettingsUtil`. They can be set either using comandline
or by placing them in the yaml file defined with the commandline argument `settingsFile`.
All of them are printed to commandline when the program starts (and also into the file `usedSettingsdso.txt`).
Most of these are documented in the header file they are defined in
(see `src/IMU/IMUSettings.h`, `src/IMUInitialization/IMUInitSettings.h`).

### 4 Running the live demo
See [doc/RealsenseLiveVersion.md](doc/RealsenseLiveVersion.md)

### 5 Running on your own datasets
To run on your own dataset you need
* to pass the folder containing files with `files=...`
* an accurate camera calibration! For tips on calibration and the format of camera.txt see
[src/dso/README.md](src/dso/README.md).
* to set the `mode=1` unless you have a photometric calibration (vignette.png and pcalib.txt).
* a file times.txt which contains **exactly** one timestamp for each image in
  the image folder, or specified through the `tsFile` parameter. Note that this
  file contains the timestamp twice, first in nanoseconds and then in seconds.

When enabling IMU data you also need

* IMU calibration (transformation between camera and IMU) as a `camchain.yaml`. Note that only the field `cam0/T_cam_imu`
and optionally the noise values are read from this file.
* a file containing synchronized IMU data. For each image it **must** contain an IMU 'measurement' with exactly the same timestamp.
If the sensor does not output this, a fake measurement with this timestamp has to be interpolated in advance.
    The [DM-VIO python tools](https://github.com/lukasvst/dm-vio-python-tools) contain a script to do this (see Notes on IMU-camera synchronization below).
* You should also set the IMU noise values (see `configs/tumvi.yaml`, `configs/euroc.yaml`, and `configs/4seasons.yaml`).
You can read them from an Allan-Variance plot (either computed yourself or taken from datasheet of IMU).
Note that often times these values are too small in practice and should be inflated by a large factor for optimal results.
We recommend first trying the sample noise values (e.g. the one for TUM-VI) and only using your own if they improve the performance.

**Notes on IMU-camera synchronization:** There are two "levels" of IMU-camera synchronization:
* The first one is that IMU and camera timestamps are recorded with the same device or otherwise made consistent. This is a prerequisite for running DM-VIO.
* The second level is that the IMU is triggered manually and always records an IMU sample exactly during the timestamp (middle of exposure) of each image. E.g. the VI-Sensor used in the EuRoC dataset does this, but most other visual-inertial sensors don't. For these sensors you can still run DM-VIO, but first you need to add a "fake IMU measurement" for each camera timestamp, by interpolating the neighboring IMU samples. You can do this by running

      python3 interpolate_imu_file --input imu.txt --times times.txt --output pass_this_imu_file_to_dmvio.txt

You can first set `useimu=0` to try the visual-only system (basically DSO). If this does not work well for
comparably slow motions, there is likely a problem with camera calibration which should be addressed first.

**For adjusting your config you might also find the tips [given on this page](doc/RealsenseLiveVersion.md#adjusting-the-config-file) interesting.**

### 6 License
DM-VIO is based on Direct Sparse Odometry (DSO), which was developed by Jakob Engel
at the Technical University of Munich and Intel.
Like DSO, DM-VIO is licensed under the GNU General Public License
Version 3 (GPLv3).

# ROS-Wrapper for DM-VIO: Delayed Marginalization Visual-Inertial Odometry

For more information see https://vision.in.tum.de/dm-vio and https://github.com/lukasvst/dm-vio.

This is a ROS-Wrapper for DM-VIO, inspired by the [ROS-Wrapper for DSO](https://github.com/JakobEngel/dso_ros).

It interfaces with ROS topics to input images and IMU data and to output poses.
Alternatively, it can read images and IMU data directly from a rosbag dataset.

#### Related Papers
* **[DM-VIO: Delayed Marginalization Visual-Inertial Odometry](https://vision.in.tum.de/dm-vio)**, L. von Stumberg and D. Cremers, In IEEE Robotics and Automation Letters (RA-L), volume 7, 2022
* **[Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization](https://vision.in.tum.de/vi-dso)**, L. von Stumberg, V. Usenko and D. Cremers, In International Conference on Robotics and Automation (ICRA), 2018
* **[Direct Sparse Odometry](https://vision.in.tum.de/dso)**, *J. Engel, V. Koltun, D. Cremers*, In  TPAMI, vol. 40, 2018

### Installation
First install DM-VIO as described [here](https://github.com/lukasvst/dm-vio) and make sure it works.
This version of the ROS wrapper was tested with commit `d18fa15ba086043561361941b8e5298074b34b47` of DM-VIO.

You need to set the environment variable `DMVIO_BUILD` to point to the folder where you built DM-VIO (which is expected to be
a direct subfolder of DM-VIO).
The easiest way is to put the following into your `.bashrc`

    export DMVIO_BUILD=/PATH/TO/dm-vio/cmake-build-relwithdebinfo

Then you can clone this project into your workspace and build normally with `catkin_make`.

### Running

You can run with

    rosrun dmvio_ros node calib=/PATH/TO/camera.txt \
                          imuCalib=/PATH_TO/camchain.yaml \
                          settingsFile=/PATH_TO/dm-vio-public/configs/t265_noise_tumvi.yaml \
                          mode=1 nogui=0 preset=1 quiet=1
                          # vignette=PATH_TO/vignette.png # highly recommended, especially for fisheye cameras (use mode=3 if vignette is set).

Most commandline arguments are the same as for [DM-VIO](https://github.com/lukasvst/dm-vio#3-running), you will at least need to pass camera calibration, IMU camchain, preset and mode.

The node will listen for images at `cam0/image_raw` and for IMU messages at `/imu0`.

Settings file: I recommend starting with `t265_noise_tumvi`, and in general with TUM-VI noise values. For more details and tips see [here](https://github.com/lukasvst/dm-vio/blob/master/doc/RealsenseLiveVersion.md#adjusting-the-config-file).

Notes on exposures and photometric calibration: The standard ROS image messages do not include exposure times. If you have a custom message with exposures you can modify the method `vidCb` and pass the exposure to `undistorter->undistort`.

If you do not have exposure times you cannot use `mode=0`!
Note that `mode=1` completely disables photometric calibration, including the vignette, even if it is passed.
Hence, I have added `mode=3`, which uses photometric calibration, but does not assume that there are exposures.

#### Example 1: Using EuRoC Bags
To test your installation you can download this EuRoC bag: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.bag

You need to create the `camera.txt` file using

    echo -e "458.654 457.296 367.215 248.375 -0.28340811 0.07395907 0.00019359 1.76187114e-05\n752 480\ncrop\n640 480\n" > camera.txt


Then you can run the following commands in separate terminals

    rosrun dmvio_ros node calib=/PATH/TO/camera.txt settingsFile=/PATH/TO/dm-vio/configs/euroc.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1

    rosbag play V2_01_easy.bag


#### New: Now you can also run in non-realtime mode on rosbags

For EuRoC, simply run the following command in a terminal. (Make sure that roscore is running in a separate terminal.)

    rosrun dmvio_ros node calib=/PATH/TO/camera.txt settingsFile=/PATH/TO/dm-vio/configs/euroc.yaml mode=1 nogui=0 preset=1 useimu=1 quiet=1 init_requestFullResetNormalizedErrorThreshold=0.8 init_pgba_skipFirstKFs=1 rosbag=/PATH/TO/V2_01_easy.bag loadRosbagThread=1


This command will run as fast as possible and exit once execution is complete, which makes it useful to run on rosbag
datasets. If a dataset is available as a rosbag, this is more convenient than extracting the rosbag and running the non-ros
version of DM-VIO. However, keep in mind that ROS does not support exposure times out of the box, which can result in
slightly worse accuracy compared to a dataset with full photometric calibration (including exposure times).

#### Example 2: Running on Realense T265
You can run on the Realsense T265 with their provided ROS driver. Install it with

    sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

Then you can run the following commands in separate terminals

    rosrun dmvio_ros node nogui=0 useimu=1 quiet=1 mode=3 `#we use mode 3 because we have vignette but no expusures` \
                          calib=/PATH_TO/RealsenseCalibration/camera.txt \
                          imuCalib=/PATH_TO/RealsenseCalibration/factory_camchain.yaml \
                          gamma=PATH_TO/dm-vio/configs/pcalib_linear_8bit.txt \
                          vignette=PATH_TO/dm-vio/configs/realsense/vignette_t265.png \
                          settingsFile=/PATH_TO/dm-vio/configs/t265_noise_tumvi.yaml \
                          resultsPrefix=/PATH_TO_RESULTS/ \
                          cam0/image_raw:=/camera/fisheye1/image_raw imu0:=/camera/imu # Remap topics

    roslaunch realsense2_camera rs_t265.launch unite_imu_method:=linear_interpolation enable_fisheye1:=true enable_fisheye2:=true enable_pose:=false

The `camera.txt` and `factory_camchain.txt` can be obtained by calibrating yourself or by first running the normal [Realsense demo](https://github.com/lukasvst/dm-vio/blob/master/doc/RealsenseLiveVersion.md) which will save the factory calibration.

Note that the ROS driver does **not** read out exposures, hence we need to use `mode=3` instead of `mode=1` and the accuracy can be a bit worse than with the normal Realsense demo.

### Published topics
When using this ROS wrapper you might want to change `src/ROSOutputWrapper.cpp` to publish information in a format suited to your application.

We publish the following topics by default:

### `dmvio/frame_tracked`: DMVIOPoseMsg
Published for every tracked frame (except the inital two frames).
It includes
* `geometry_msgs/Pose` pose: Transformation from camera to world in DSO frame (meaning it can have an arbitrary scale).

and all fields in `transformDSOToIMU`, which enables you to convert the pose (and all published poses prior to this) to the metric frame:
* `float64` scale
* `geometry_msgs/Quaternion` rotationMetricToDSO: Gravity direction encoded as the rotation to convert from metric world to DSO world
* `geometry_msgs/Pose` imuToCam: Transformation between camera and IMU

### `dmvio/unscaled_pose`: PoseStamped
This is just a copy of `/dmvio/frame_tracked/pose`. It can be useful for visualizing in Rviz as PoseStamped is a standard message.
**Note** that the used coordinate system is camera-based (see below), which is why it can look strange in Rviz.

### `dmvio/metric_pose`: PoseStamped
These poses are the transformation from IMU to world (in metric scale).
We provide this for convenience but for many applications you should **not** directly subscribe to this topic, but rather convert the poses yourself using the newest scale.

The reason is that these metric poses are all converted with the scale estimated at the time they were published.
But when you perform computations on the entire trajectory you want to **convert all poses with the newest scale** instead.

### `dmvio/system_status`: Int32
This is a number, indicating the status the system is in (0 means visual initializer, 1 means visual-only mode and 2 means visual-inertial mode).
For details see the enum [`dmvio::SystemStatus`](https://github.com/lukasvst/dm-vio/blob/master/src/dso/IOWrapper/Output3DWrapper.h#L46).
Whenever a full reset happens, 0 is sent to this topic again.

#### Coordinate system
As mentioned the poses are either in metric or in DSO frame, for converting between the two see [TransformDSOToIMU::transformPose](https://github.com/lukasvst/dm-vio/blob/master/src/GTSAMIntegration/PoseTransformationIMU.cpp) or equation (5) in the [DM-VIO paper](https://vision.in.tum.de/_media/research/vslam/dm-vio/dm-vio.pdf).

We use the typical camera-based coordinate frame, where x points to the right of the image, y points down and z points forward (into the camera).

#### What is not published / implemented and what you might want to implement yourself:
* The **keyframes** after bundle adjustment: These will be more accurate than the initially tracked frame poses. You can access them as well as the **pointcloud** by overriding `publishKeyframes` in `ROSOutputWrapper.cpp`. You could take inspiration from how it was implemented in [LSD-SLAM](https://github.com/tum-vision/lsd_slam/blob/master/lsd_slam_core/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp).
* TF frames
* Dynamic reconfigure

### License
This ROS wrapper is licensed under the GNU General Public License 3 (GPLv3).
The main.cpp file reuses code from the main file of DSO.

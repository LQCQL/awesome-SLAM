# awesome-SLAM
This is a list of awesome open sources  about SLAM. 

## Sparse map
[ORB SLAM v2（Monocular, Stereo and RGB-D Cameras）](https://github.com/raulmur/ORB_SLAM2)  
ORB-SLAM2 is a real-time SLAM library for Monocular, Stereo and RGB-D cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). 

## Semi-dense map
[LSD SLAM(Monocular, Stereo and RGB-D Cameras)](https://github.com/tum-vision/lsd_slam)  
LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct (i.e. does not use keypoints / features) and creates large-scale, semi-dense maps in real-time on a laptop.  

[DSO(Monocular)](https://github.com/JakobEngel/dso)  
Direct Sparse Odometry is a novel direct and sparse formulation for Visual Odometry. It combines a fully direct probabilistic model (minimizing a photometric error) with consistent, joint optimization of all model parameters, including geometry - represented as inverse depth in a reference frame - and camera motion.   

[SVO(Monocular)](https://github.com/uzh-rpg/rpg_svo)  
This code implements a semi-direct monocular visual odometry pipeline.  

## Dense map
[RGB-D SLAM V2(RGB-D)](https://github.com/felixendres/rgbdslam_v2)  
RGBDSLAMv2 is a state-of-the-art SLAM system for RGB-D cameras, e.g., the Microsoft Kinect or the Asus Xtion Pro Live. You can use it to create 3D point clouds or OctoMaps.

[Kintinuous (RGB-D)](https://github.com/mp3guy/Kintinuous)
Real-time dense visual SLAM system capable of producing high quality globally consistent point and mesh reconstructions over hundreds of metres in real-time with only a low-cost commodity RGB-D sensor.  

[Bundle Fusion (RGB-D)](https://github.com/niessner/BundleFusion)  
Real-time Globally Consistent 3D Reconstruction using Online Surface Re-integration.

[InfiniTAM (RGB-D)](https://github.com/victorprad/InfiniTAM)  
InfiniTAM an open source, multi-platform framework for real-time, large-scale depth fusion and tracking, support sparse volumes, using an implementation of our ISMAR 2015 paper, optionally with loop closure, using an implementation of our ECCV 2016 paper. A prelimiary surfel-based version of the pipeline is also included, as detailed in our 2017 Technical Report.

[RTAB-Map(RGB-D, Stereo and Lidar)](https://github.com/introlab/rtabmap)  
RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. 

## Multi-Sensor Fusion
[VINS(stereo cameras / stereo cameras + IMU / mono camera + IMU)](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)  
VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR).  [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) is a real-time SLAM framework for Monocular Visual-Inertial Systems.

[OKVIS(stereo cameras + IMU / mono camera + IMU)](https://github.com/ethz-asl/okvis)  
Open Keyframe-based Visual-Inertial SLAM. 

[ROVIO(mono camera + IMU)](https://github.com/ethz-asl/rovio)  
Robust Visual Inertial Odometry .

[RKSLAM(mono camera + IMU)](http://www.zjucvg.net/rkslam/rkslam.html)    
RKSLAM is a real-time monocular simultaneous localization and mapping system which can robustly work in challenging cases, such as fast motion and strong rotation.

[Cartographer (LIDAR + IMU)](https://github.com/googlecartographer/cartographer)  
Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations. 

[V-LOAM()](https://github.com/laboshinl/loam_velodyne)  
Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar.

## Deep Learning
[CNN-SLAM](https://github.com/iitmcvg/CNN_SLAM)  
this project investigates how predicted depth maps from a deep neural network can be deployed for accurate and dense monocular reconstruction, propose a method where CNN-predicted dense depth maps are naturally fused together with depth measurements obtained from direct monocular SLAM. 

[VINet](https://github.com/HTLife/VINet)  
Visual-Inertial Odometry as a Sequence-to-Sequence Learning Problem 

[3DMV](https://github.com/angeladai/3DMV)  
3DMV: Joint 3D-Multi-View Prediction for 3D Semantic Scene Segmentation. 3DMV jointly combines RGB color and geometric information to perform 3D semantic segmentation of RGB-D scans.
![](https://raw.githubusercontent.com/angeladai/3DMV/master/images/teaser.jpg)

[ScanComplete](https://github.com/angeladai/ScanComplete)  
ScanComplete: Large-Scale Scene Completion and Semantic Segmentation for 3D Scans. ScanComplete is a data-driven approach which takes an incomplete 3D scan of a scene as input and predicts a complete 3D model, along with per-voxel semantic labels.  
![](https://raw.githubusercontent.com/angeladai/ScanComplete/master/images/teaser_mesh.jpg)

[DeepVO](https://github.com/ildoonet/deepvo)  
DeepVO for Visual Odometry Implementation using Tensorflow.


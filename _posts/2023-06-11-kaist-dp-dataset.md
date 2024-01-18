---
title: "[KAIST Divergent Path (KAIST-DP) Dataset]"
classes: wide
---

## Github Code 
[![Github_Page]({{ site.url }}{{ site.baseurl }}/assets/images/github_logo.png)](https://github.com/SungJaeShin/KAIST-DP.git)*Click Image!*

---
<figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/kaist_dp/HW_configuration.png" alt="">
</figure>
---

**Publish Paper** 

If you use KAIST-DP dataset for your academic research, please cite our paper.

- **Sungjae Shin**, Yeeun Kim, Byeongho Yu, Eungchang Mason Lee, and Hyun Myung, "PanoNetVLAD: Visual Loop Closure Detection in Continuous Space Represented With Panoramic View Using Multiple Cameras," _in Proc International Conference on Control, Automation and Systems (ICCAS)_, 2023, [DOI:10.23919/ICCAS59377.2023.10316996](https://doi.org/10.23919/ICCAS59377.2023.10316996).

---

**Description**
- Intel Realsense D455 x3
- Xsens MTI-300 IMU
- Velodyne LiDAR VLP 16


**Message Topic Name**
- Image Topic (__sensor_msgs/Image__):
    - Front Camera: "/front/camera/infra1/image_rect_raw" & "/front/camera/infra2/image_rect_raw"
    - Left Camera: "/left/camera/infra1/image_rect_raw" & "/left/camera/infra2/image_rect_raw"
    - Right Camera: "/right/camera/infra1/image_rect_raw" & "/right/camera/infra2/image_rect_raw"

- IMU Topic (__sensor_msgs/Imu__):
    - "/imu/data"

- LiDAR Topic (__velodyne_msgs/VelodyneScan__):
    - "/velodyne_packets"

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Realsense SDK**
Follow [Realsense SDK](https://github.com/IntelRealSense/realsense-ros)



### 1.3 **Xsense Driver**
Follow [Xsens Driver](https://github.com/ethz-asl/ethzasl_xsens_driver)


## 2. Build KAIST-DP dataset
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/SungJaeShin/KAIST-DP.git
    cd ../../
    catkin config -DCMAKE_BUILD_TYPE=Release
    catkin build KAIST-DP
    source ~/catkin_ws/devel/setup.bash 
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)


## 3. Get KAIST-DP dataset
Open two terminals, run multi camera and IMU and get the bag file respectively:
```
    roslaunch KAIST-DP three_cams.launch 
    
    roscd KAIST-DP
    cd sh_files/
    sh get_kaist_dp_bag.sh
```
(then, bag file save at bag folder !!)

## 4. Provided Dataset (ROSBAG Data)
- KAIST-DP Outdoor Dataset (Library): 
    - [E9 Building](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/EgFowFsfCnJOrnf8YNZT4_ABsuKom2Fc5WmXYDZKLxbHWw?e=cAQULA) & [E9 Building Tilt](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/EoIrnHe06QNPpVh3wZXYsO8BuT7ao6k5xnyLYq34oiYDGw?e=VCMF23)
    <figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/kaist_dp/library_path.png" alt="">
    </figure>

- KAIST-DP EE-Building:
    - [EE-Building](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/ErDl1w2n56VOi7I-0XTpV1EB0WMorX6Aa3_CKpHn79b9RQ?e=JNhl6l)
    <figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/kaist_dp/ee_building_path.png" alt="">
    </figure>


- KAIST-DP Indoor Dataset (EE):
    - [E3 Building](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/Eiawqsudb51KhiSLEA1H3mgBruKdMMPxR3aWvmmdqQVR9w?e=p5tBdC)
    <figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/kaist_dp/ee_building_indoor_path.png" alt="">
    </figure>


## 5. Example of sensor data applying LIO-SAM and VINS-Fusion
- For LiDAR data applied [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git)
    <figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/kaist_dp/lio_sam.png" alt="">
    </figure>


- For Camera data applied [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git)
    <figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/kaist_dp/vins_fusion.png" alt="">
    </figure>


## 6. Acknowledgements
These cameras and IMU are provided URL of KAIST.

Thank you for Byeongho Yu who made zig to get the KAIST-DP dataset.

For commercial inquiries, please contact **Sungjae Shin (pootti@kaist.ac.kr)**
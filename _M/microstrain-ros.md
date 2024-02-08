---
title: "[Microstrain Inertial Package] How to use including ROS node?"
excerpt: "Microstrain IMU Installation"
---
# [Microstrain Inertial Package] How to use including ROS node?

---

- **[Goal]**
    - Microstrain에서 제공하는 IMU 센서를 ROS driver를 통해서 데이터를 얻을 수 있다.

---

- **[Github]**
    
    [GitHub - LORD-MicroStrain/microstrain_inertial: ROS driver for all of MicroStrain's current G and C series products. To learn more visit](https://github.com/LORD-MicroStrain/microstrain_inertial.git)
    

---

- **[How to install?]**
    - 먼저 공식 github page에 ros branch로 git clone 하고 하위 모듈까지 같이 받을 수 있도록 clone 진행 후 build
        
        ```bash
        $ cd ~/catkin_ws/
        $ catkin config -DCMAKE_BUILD_TYPE=Release
        $ cd ./src/
        $ git clone --recursive --branch ros https://github.com/LORD-MicroStrain/microstrain_inertial.git
        $ cd ..
        $ catkin build
        $ source devel/setup.bash
        ```
        
    - 다음 launch file을 실행하면 IMU data를 쉽게 취득할 수 있음 !!
        
        ```bash
        $ roslaunch microstrain_inertial_driver microstrain.launch
        ```
        

---

- **[Error Related]**
    - 본 저자가 사용한 IMU 기종은 “3DM-GV7-AHRS” 이라서 문제가 발생하는 경우들이 존재하였음
        - 작성일은 2023.11.07 이고 이 시기에 가장 최신 IMU 기종이기 때문에 생기는 문제일듯 함
            - **[Error 1]**
                - `[ERROR] Failed to configure PPS source`
                - `[ERROR] Error(3): Invalid Parameter`
            - **[Ref 1]**
                
                - [3dm-cv7 - failed to configure PPS source · Issue #273 · LORD-MicroStrain/microstrain_inertial](https://github.com/LORD-MicroStrain/microstrain_inertial/issues/273#issuecomment-1697980522)
                
            - **[Solution 1]**
                - GV7 기종이기 때문에 GQ7/CV7 기종만 사용하는 PPS source를 disable 해주어야함 **microstrain_inertial_driver_common/config/params.yml** 파일안에 `filter_pps_source` 을 `0`으로 set 해주어야 함
                    - **[params.yml](https://github.com/LORD-MicroStrain/microstrain_inertial_driver_common/blob/3d3d678c0e2562151464d741094c75fd37705bf0/config/params.yml#L425)**
                        
                        ```yaml
                        # [Before]
                        filter_pps_source : 1
                        
                        # [After]
                        filter_pps_source : 0
                        ```
                        
            
            ---
            
            - **[Error 2]**
                - `[ERROR] Failed to read configuration for node`
                - `[ERROR] Unable to configure node base`
            - **[Ref 2]**
                
                - [an instance of 'mscl::Error_MipCmdFailed'   what():  The DeclinationSource command has failed. · Issue #168 · LORD-MicroStrain/microstrain_inertial](https://github.com/LORD-MicroStrain/microstrain_inertial/issues/168#issuecomment-1192737248)
                
            - **[Solution 2]**
                - GV7 기종도 filter_declination_source 를 사용하지 않아서 **microstrain_inertial_driver_common/config/params.yml** 파일안에  `filter_declination_source` 를 `1` 으로 set 해주어야 함
                    - [**params.yml**](https://github.com/LORD-MicroStrain/microstrain_inertial_driver_common/blob/3d3d678c0e2562151464d741094c75fd37705bf0/config/params.yml#L236C19-L236C19)
                        
                        ```yaml
                        # [Before]
                        filter_declination_source : 2
                        
                        # [After]
                        filter_declination_source : 1
                        ```
                        

---

- **[Results]**
    - 최종 결과
        - rostopic list
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/microstrain/Untitled.png" alt="">
            </figure>   
            
            (1) /gps_corr (microstrain_inertial_msgs/GPSCorrelationTimestampStamped)
            
            → IMU GPS correlation timestamp
            
            ---
            
            (2) /imu/data (sensor_msgs/Imu)
            
            → Raw IMU data
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/microstrain/Untitled 1.png" alt="">
            </figure>   
                        
            ---
            
            (3) /nav/filtered_imu/data (sensor_msgs/Imu)
            
            → Provides filtered IMU data based on the output of the EKF running on the device

            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/microstrain/Untitled 2.png" alt="">
            </figure>   
            
            ---
            
            (4) /nav/heading (microstrain_inertial_msgs/FilterHeading)
            
            → Provides the heading based on the output of the EKF running on the device
            
            ---
            
            (5) /nav/odom (nav_msgs/Odometry)
            
            → Provides odometry based on the output of the EKF running on the device

            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/microstrain/Untitled 3.png" alt="">
            </figure>   
            
            ---
            
            (6) /nav/status (microstrain_inertial_msgs/FilterStatus)
            
            → Provides status of Extended Kalman Filter running on the device

            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/microstrain/Untitled 4.png" alt="">
            </figure>   
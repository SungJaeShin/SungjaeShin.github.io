---
title: "[RealSense package] How to use?"
excerpt: "RealSense Installation"
---
# [RealSense package] How to use?

- Reference Site: [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)
    

---

- **[Installation]**
    - (1) Install realsense2_camera related with ROS distribution
        - **설치 전 주의할 사항**
            - **(1) ROS에 설치되는 ROS realsense version과 SDK와 연관되어 있는 버전을 설치**
        
        ```bash
        # Example of ROS "melodic" version
        $ sudo apt-get install ros-melodic-realsense2-camera 
        ```
        
    
    ---
    
    - (2) Install librealsense SDK
        - Reference Site: [https://github.com/IntelRealSense/realsense-ros/releases](https://github.com/IntelRealSense/realsense-ros/releases)
            
            - **설치 전 주의할 사항**
                - **(1) Supported ROS Distributions 확인**
                - **(2) Supported Platforms 확인**
                - **(3) Supported Devices 확인**
        
        ```bash
        # Dependencies
        $ sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
        $ sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
        
        # Unzip librealsense.zip
        $ cd librealsense
        $ mkdir build
        $ cd build
        $ cmake .. -DCMAKE_BUILD_TYPE=Release
        $ sudo make uninstall 
        $ make clean
        $ time make -j8 
        $ sudo make install
        ```
        
    
    ---
    
    - (3) Example of installation
        - Assume Setting
            - Ubuntu 18.04 LTS
            - ROS Melodic
            - D455 realsense camera
        - **1) install ROS realsense version → 2.3.1**
        - **2) install SDK librealsense version → 2.48.0**

---

- **[Topics]**
    - Intel T265 Tracking Camera related launch file
        
        ```bash
        $ roscd realsense2_camera
        $ cd launch
        $ roslaunch rs_t265.launch
        ```
        
        - T265 camera topic
            - Topic name → /camera/odom/sample
            - Message type → nav_msgs/Odometry
    
    ---
    
    - Intel D435i & D455i RGB-D Camera
        
        ```bash
        $ roscd realsense2_camera
        $ cd launch
        $ roslaunch rs_camera.launch
        ```
        
        - D435 & D455 camera topic
            - Image topic name → /camera/color/image_raw
            - Message type → sensor_msgs/Image
            
            ---
            
            - Compressed topic name → /camera/color/image_rect_raw/compressed
            - Message type → sensor_msgs/CompressedImage
            
            ---
            
            - Left Infra topic name → /camera/infra1/image_rect_raw
            - Right Infra topic name → /camera/infra2/image_rect_raw
            - Message type → sensor_msgs/Image
            
            ---
            
            - Camera built-in IMU topic name → /camera/imu
            - Message type → sensor_msgs/Imu
            - **내장되어 있는 IMU data 값은 상당히 좋지 않고 hz가 계속 변해서 일정한 값을 주지 않아 사용하기에 문제가 많음**

---

- **[Configuration]**
    - Settings of camera
        
        ```bash
        $ roscd realsense2_camera
        $ cd launch/includes
        $ gedit nodelet.launch.xml
        
        ##### Default settings in nodelet.launch.xml file #####
        # Depth related
        <arg name="depth_width"         default="640"/>
        <arg name="depth_height"        default="480"/>
        <arg name="enable_depth"        default="true"/>
        
        # Infra related
        <arg name="infra_width"         default="640"/>
        <arg name="infra_height"        default="480"/>
        <arg name="enable_infra"        default="false"/>
        <arg name="enable_infra1"       default="false"/>
        <arg name="enable_infra2"       default="false"/>
        <arg name="infra_rgb"           default="false"/>
        
        # Color related
        <arg name="color_width"         default="640"/>
        <arg name="color_height"        default="480"/>
        <arg name="enable_color"        default="true"/>
        
        # Built-in IMU related
        <arg name="gyro_fps"            default="200"/>
        <arg name="accel_fps"           default="63"/>
        <arg name="enable_gyro"         default="false"/>
        <arg name="enable_accel"        default="false"/>
        <arg name="unite_imu_method"    default="linear_interpolation"/> \
        <!-- Options are: [none, copy, linear_interpolation] -->
        
        # High ACC emitter off
        <arg name="json_file_path"      default="$(find ~)/high_acc_emitter_off.json"/>
        
        # Reset prior camera when restart launch file
        <arg name="initial_reset"       default="false"/>
        ```
        

---

- **[High_acc_emitter_off.json]**
    
    ```json
    {
        "aux-param-autoexposure-setpoint": "1536",
        "aux-param-colorcorrection1": "0.298828",
        "aux-param-colorcorrection10": "-0",
        "aux-param-colorcorrection11": "-0",
        "aux-param-colorcorrection12": "-0",
        "aux-param-colorcorrection2": "0.293945",
        "aux-param-colorcorrection3": "0.293945",
        "aux-param-colorcorrection4": "0.114258",
        "aux-param-colorcorrection5": "-0",
        "aux-param-colorcorrection6": "-0",
        "aux-param-colorcorrection7": "-0",
        "aux-param-colorcorrection8": "-0",
        "aux-param-colorcorrection9": "-0",
        "aux-param-depthclampmax": "65536",
        "aux-param-depthclampmin": "0",
        "aux-param-disparityshift": "0",
        "controls-autoexposure-auto": "True",
        "controls-autoexposure-manual": "8500",
        "controls-color-autoexposure-auto": "True",
        "controls-color-autoexposure-manual": "166",
        "controls-color-backlight-compensation": "0",
        "controls-color-brightness": "0",
        "controls-color-contrast": "50",
        "controls-color-gain": "64",
        "controls-color-gamma": "300",
        "controls-color-hue": "0",
        "controls-color-power-line-frequency": "3",
        "controls-color-saturation": "64",
        "controls-color-sharpness": "50",
        "controls-color-white-balance-auto": "True",
        "controls-color-white-balance-manual": "4600",
        "controls-depth-gain": "16",
        "controls-laserpower": "150",
        "controls-laserstate": "off",
        "ignoreSAD": "0",
        "param-amplitude-factor": "0",
        "param-autoexposure-setpoint": "1536",
        "param-censusenablereg-udiameter": "6",
        "param-censusenablereg-vdiameter": "7",
        "param-censususize": "6",
        "param-censusvsize": "7",
        "param-depthclampmax": "65536",
        "param-depthclampmin": "0",
        "param-depthunits": "1000",
        "param-disableraucolor": "0",
        "param-disablesadcolor": "0",
        "param-disablesadnormalize": "0",
        "param-disablesloleftcolor": "0",
        "param-disableslorightcolor": "1",
        "param-disparitymode": "0",
        "param-disparityshift": "0",
        "param-lambdaad": "751",
        "param-lambdacensus": "6",
        "param-leftrightthreshold": "10",
        "param-maxscorethreshb": "2893",
        "param-medianthreshold": "796",
        "param-minscorethresha": "4",
        "param-neighborthresh": "108",
        "param-raumine": "6",
        "param-rauminn": "3",
        "param-rauminnssum": "7",
        "param-raumins": "2",
        "param-rauminw": "2",
        "param-rauminwesum": "12",
        "param-regioncolorthresholdb": "0.785714",
        "param-regioncolorthresholdg": "0.565558",
        "param-regioncolorthresholdr": "0.985323",
        "param-regionshrinku": "3",
        "param-regionshrinkv": "0",
        "param-robbinsmonrodecrement": "25",
        "param-robbinsmonroincrement": "2",
        "param-rsmdiffthreshold": "1.65625",
        "param-rsmrauslodiffthreshold": "0.71875",
        "param-rsmremovethreshold": "0.809524",
        "param-scanlineedgetaub": "13",
        "param-scanlineedgetaug": "15",
        "param-scanlineedgetaur": "30",
        "param-scanlinep1": "155",
        "param-scanlinep1onediscon": "160",
        "param-scanlinep1twodiscon": "59",
        "param-scanlinep2": "190",
        "param-scanlinep2onediscon": "507",
        "param-scanlinep2twodiscon": "493",
        "param-secondpeakdelta": "647",
        "param-texturecountthresh": "0",
        "param-texturedifferencethresh": "1722",
        "param-usersm": "1",
        "param-zunits": "1000",
        "stream-depth-format": "Z16",
        "stream-fps": "30",
        "stream-height": "480",
        "stream-ir-format": "R8L8",
        "stream-width": "640"
    }
    ```
    

---

- **[Others]**
    - **Auto Exposure:**
        - 카메라에서는 자동적으로 노출을 조절하게 되는데 자동으로 설정하게 되면 야외에서 위치 추정시 **자동 노출 조절 때문에 이미지들 간의 matching 에 어려움이 생기게 됨**
        - 따라서, **realsense-viewer를 통해서 auto exposure를 disable 로 설정**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/realsense/Untitled.png" alt="">
            </figure>
            
    
    ---
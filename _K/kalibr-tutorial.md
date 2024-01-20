---
title: "[Kalibr] Camera Calibration"
excerpt: "Camera intrinsic & extrinsic parameter calculation tool"
---
# [Kalibr] Camera Calibration

- **[Goal]**
    - Kalibr package를 사용하여 Camera intrinsic & extrinsic parameter를 찾는 것을 목적으로 한다.

---

- **[Github]**
    - [**https://github.com/ethz-asl/kalibr**](https://github.com/ethz-asl/kalibr)   

---

- **[Calibration 순서]**
    - (1) 실제로 Aprilgrid의 영상을 딴 Camera data와 IMU data가 담겨 있는 bag file record
        - 실제 영상을 딸 때, **Aprilgrid가 다 보이게 영상을 따고 주로 가장자리의 선과 다양한 각도에서 영상을 촬영**한다. 그리고 **로봇도 마음대로 기울여서 촬영** 해주어야 한다.
        - **[Bag file] 다운로드**
        
            - [Sign in to your account](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/EuEW-ueknMhKn8rWmjlx-jcBnpCdjLGktaSEe-zfT7ydhg?e=l2tS2k)
        
        ---
        
    - (2) Camera intrinsic parameter를 구하기 위해 Aprilgrid의 yaml file을 얻는다.
        - Reference Site: [**https://github.com/ethz-asl/kalibr/wiki/calibration-targets**](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)
                
        - Download Site: [**https://github.com/ethz-asl/kalibr/wiki/downloads**](https://github.com/ethz-asl/kalibr/wiki/downloads)
        
        - **[주의]**
            - Calibration을 위한 target은 여러 종류가 있는데 그 중에 Aprilgrid로 하였기 때문에 이를 참조하는 것이다.
    
    ---
    
    - (3) **/kalibr/aslam_offline_calibration/kalibr/python/** 로 들어가서 camera data로만 camera intrinsic parameter를 구한다.
        
        ```bash
        # In kalibr/aslam_offline_calibration/kalibr/python
        
        # --bag   : Directory where the camera and imu bag data are containing Aprilgrid images 
        # --topics: Camera topics in the bag data (If the bag data has stereo, then write two topics)
        # --models: Write camera models (If the used camera is stereo, then write two times same model name)
        # --target: Directory where the aprilgrid configuration is 
        
        $ kalibr_calibrate_cameras --bag ($ Camera-IMU).bag --topics /zed/left/image_rect_color /zed/right/image_rect_color --models pinhole-radtan pinhole-radtan --target ($ aprilgrid).yaml
        ```
        
        - kalibr_calibrate_camera 실행 → 이때 결과는 pdf 및 yaml file이 생성
            - yaml file안의 **이미지 size의 절반 정도가 대략적으로 projection_parameter값으로 나타난다면 올바르게 intrinsic parameter를 구했다고 생각**할 수 있다. **(bag data image는 640 * 480 기준)**
        - ****[주의]****
            - 구하는 과정에서 --models pinhole-equi 로 작성하는 경우를 볼 것이다. 그런데 camera model을 **pinhole_equi → pinhole_radtan**으로 해야한다.
            
            ---
            
    - (4) IMU intrinsic parameter 들이 담겨 있는 yaml file 을 준비한다.
        - Yaml file 안에 bag file이 담고 있는 IMU topic 명을 작성해준다.
        - **일반적으로 VINS package**에 있는 config안 yaml file에 담겨 있는 imu parameter들을 사용한다.
        - 보통은 **잘 안 맞는 경우가 있어 추후 실험을 하면서 tuning을 진행**한다.
        - **[IMU Yaml] 다운로드**
            
            - [Sign in to your account](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/EsAGJRxUxUhOmw4mB2ShqW4BxBsh4q246OXLc_3XpNVIKQ?e=i5ygSJ)
            
        
        ---
        
    - (5) **/kalibr/aslam_offline_calibration/kalibr/python/** 로 들어가서 앞서 얻은 **camera intrinsic yaml & aprilgrid yaml & imu intrinsic yaml & bag file을 모두 이용**하여 camera와 imu간의 extrinsic parameter를 구한다.
        
        ```bash
        # In kalibr/aslam_offline_calibration/kalibr/python
        
        # --bag   : Directory where the camera and imu bag data are containing Aprilgrid images 
        # --topics: Camera topics in the bag data (If the bag data has stereo, then write two topics)
        # --models: Write camera models (If the used camera is stereo, then write two times same model name)
        # --target: Directory where the aprilgrid configuration is 
        # --cam   : Directory where the camera intrinsic configuration is 
        # --imu   : Directory where the imu configuration is 
        
        $ kalibr_calibrate_imu_camera --bag ($ Camera-IMU).bag --cam camchain-Kalibr_data.yaml --imu ($ imu).yaml --target ($ aprilgrid).yaml
        ```
        
        - kalibr_calibrate_imu_camera 실행 → 이때 결과는 pdf 및 yaml file이 생성된다.
            - **이 결과를 이용해서 실제 VINS의 configuration을 만들 수 있다.**
            - **[ROSBOT 2.0 기준 결과] 다운로드**
                
                - [Sign in to your account](https://kaistackr-my.sharepoint.com/:f:/g/personal/pootti_kaist_ac_kr/EnaEtxf6lM1NniDjw74PExMB9EYUquuIh14tVFhSmxqjdQ?e=K2mZxJ)
                

---

- **[참고]**
    - 최근 kalibr version pdf 결과에는 camera-to-imu matrix도 있지만 imu-to-camera matrix도 있다.
        - **(Tip) 이들 중에서 4x4 matrix에서 마지막 column에 해당하는 position이 실제로 camera와 imu 사이의 거리가 맞는 것을 사용하면 된다.  **(cam_to_imu 이걸 사용!)****
    
    ---
    
    - Kalibr_allan을 이용하여 IMU intrinsic parameter를 구할 수 있는데 이 역시도 **부정확**해서 일반적으로 VINS에 configuration의 yaml file의 default 값을 사용한다.
        - IMU intrinsic calibration site
            - Reference Site: [**https://suho0515.tistory.com/6**](https://suho0515.tistory.com/6)
            
            - Reference Site: [**https://github.com/rpng/kalibr_allan**](https://github.com/rpng/kalibr_allan)            
    
    ---
    
    - 전체적인 processing을 담은 site 참고
        - Reference Site: [**https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS**](https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS)
                
        - Reference Site: [**https://wsstudynote.tistory.com/19**](https://wsstudynote.tistory.com/19)
                
        - Reference Site: [**https://suho0515.tistory.com/m/12**](https://suho0515.tistory.com/m/12)
        

---

- **[Error Related]**
    - **[1]** `fatal error: libv4l2.h: No such file or directory`
        - Reference Site
            
            - [missing libv4l2.h file during compilation - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/287589/missing-libv4l2h-file-during-compilation/)
            
        - **[Solve]**
            
            ```bash
            $ sudo apt update
            $ sudo apt install libv4l-dev
            ```
            
    
    ---
    
    - **[2]** `ImportError: No module named igraph`
        - **[Solve]**
            
            ```bash
            $ sudo apt update
            $ sudo apt install python-igraph
            ```
            
    
    ---
    
    - **[3]** `ImportError: No module named scipy.optimize`
        - Reference Site
            
            - [No module named scipy.optimize · Issue #66 · ethz-asl/kalibr](https://github.com/ethz-asl/kalibr/issues/66)
            
        - **[Solve]**
            
            ```bash
            $ sudo apt-get install gfortran
            # If the python version is 2, otherwise python version is 3 then use pip3 
            $ sudo -H pip install scipy 
            ```
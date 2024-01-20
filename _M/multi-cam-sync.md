---
title: "[Multiple camera] How to run multiple intel realsense cameras?"
excerpt: "Multiple Camera Configuration"
---
# [Multiple camera] Tips for time sync when running multiple cameras

## [Goal] 여러개의 realsense camera를 구동하기에 hardware 및 software 모두 time sync 를 위한 작업이 필요하여 이를 확인

---

- **[Hardware]**
    - Reference Site
        
        - [Is hardware sync for multi camera configuration needed in ROS? · Issue #984 · IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros/issues/984#issuecomment-553262466)
        
        - [External Synchronization of Intel® RealSense™ Depth cameras](https://dev.intelrealsense.com/docs/external-synchronization-of-intel-realsense-depth-cameras)
        
    
    ---
    
    - Problem definition
        - 카메라는 보통 각각의 clock frequency를 가지고 있어 동시에 여러대의 카메라를 키면 다 다르게 작동을 하게 됨. 그래서 원하는 time stamp에 이미지를 가지고 오려면 제일 근접한 것을 가지고 오게 되는데 각각 clock frequency가 다르다 보니 다른 이미지를 가지고 올 수 있는 경우가 생김
            - (예시) ROS time 기준 1643350626의 사진을 3개의 카메라에서 가지고 오라고 가정할 때, 각 clock frequency 가 다르다 보니 626 timestamp에 제일 근접한 것을 가지고 오려고 함. 이때 Cam1 에서는 622, Cam 2 에서는 625, Cam 3 에서는 620 등으로 다 다르게 가져올수 있는 현상이 발생
                
                그런데, 하드웨어 sync 작업을 하게 되면 3대의 카메라의 clock frequency를 맞추는 효과가 있어서 626의 timestamp는 정확하지 않더라도 카메라끼리는 조금 더 균일한 timestamp로 이미지들을 가지고 옴 (예: Cam1: 622, Cam2: 623, Cam3: 622)
                
    
    ---
    
    - Schematic
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_sync/Untitled.png" alt="">
        </figure>
        

---

- **[Software]**
    - Reference Site
        
        - [VINS-Fusion/rosNodeTest.cpp at be55a937a57436548ddfb1bd324bc1e9a9e828e0 · HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/rosNodeTest.cpp#L71)
        
    
    ---
    
    - Problem definition
        - 카메라를 키고 들어오는 time stamp 시간이 명확하게 똑같지 않기 때문에 reference 카메라의 시간에 가장 근접한 이미지들을 맞춰서 받는 작업을 진행
    
    ---
    
    - Code Example **(PanoNetVLAD Code)**
        
        ```cpp
        // Panorama Image Related
        void Estimator::syncImage(double prev_time, double cur_time)
        {
            if(!frontimgBuf.empty())
            {
                double t = frontimgBuf.front().header.stamp.toSec();
                if(t != cur_time)
                {
                    ROS_WARN("NOT Sync Image !!");
                    return;
                }
        
                while(leftimgBuf.front().header.stamp.toSec() < prev_time)
                    leftimgBuf.pop();
                while(rightimgBuf.front().header.stamp.toSec() < prev_time)
                    rightimgBuf.pop();
        
                sensor_msgs::Image tmp_left_img = leftimgBuf.front(); 
                sensor_msgs::Image tmp_front_img = frontimgBuf.front(); 
                sensor_msgs::Image tmp_right_img = rightimgBuf.front(); 
                
                tmp_left_img.header = tmp_front_img.header;
                tmp_right_img.header = tmp_front_img.header;
        
                left_image = tmp_left_img;
                front_image = tmp_front_img;
                right_image = tmp_right_img;
        
                frontimgBuf.pop();
            }
        }
        ```
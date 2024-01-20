---
title: "[ROS1-ROS2 Bridge] How to use ROS1-ROS2 bridge?"
excerpt: "ROS1-ROS2 Bridge Tutorial"
---
# [ROS1-ROS2 Bridge] How to use ROS1-ROS2 bridge?

---

- **[Github]**
    - [https://github.com/ros2/ros1_bridge](https://github.com/ros2/ros1_bridge)        

---

- **[Cause]**
    - 아직까지 많은 package 들이 ros2 에서 돌아가지 않은 경우가 많아서 ros1 에서 돌아가는 결과를 ros2 로 보내주는 행위를 하거나 ros2 에서 로봇 간 서로 통신된 메세지들을 ros1 으로 전달하여 이를 알고리즘에 돌려 결과를 얻는 방식 때문에 이를 사용하게 되었다.

---

- **[Build ROS1-ROS2 bridge]**
    
    ```bash
    # Go to ROS2 workspace 
    $ cd ros2_ws/src
    
    # Clone ros1_bridge github 
    $ git clone https://github.com/ros2/ros1_bridge.git
    
    # Change branch fit your ROS2 distribution
    $ cd ros1_bridge 
    $ git checkout ${ROS2_DISTRO}
    $ cd ../../
    
    # Build ros1_bridge 
    $ colcon build --symlink-install --packages-select ros1_bridge
    ```
    

---

- **[Process]**
    - ROS1 알고리즘이 돌아가는 terminal 1개 / ROS2 알고리즘이 돌아가는 terminal 1개 / ROS1-ROS2 bridge 가 돌아가는 terminal 1개로 **총 3개의 terminal 을 이용**해야 한다.
    - Example related with realsense camera
        - **Assume case: publish ros1 node and subscribe ros2 node**
            - [Step 1] ROS1 only
                
                ```bash
                # Terminal 1
                $ source /opt/ros/${ROS1_DISTRO}/setup.bash     
                # ${ROS1_DISTRO} example is noetic 
                $ roslaunch realsense2_camera rs_camera.launch
                ```
                
                ---
                
            - [Step 2] ROS2 only
                
                ```bash
                # Terminal 2
                $ source /opt/ros/${ROS2_DISTRO}/setup.bash
                # ${ROS2_DISTRO} example is foxy
                $ ros2 run sub_ros1_cam subscriber
                ```
                
                ---
                
            - [Step 3] ROS1-ROS2 bridge
                
                ```bash
                # Terminal 3
                $ source /opt/ros/${ROS1_DISTRO}/setup.bash 
                $ source /opt/ros/${ROS2_DISTRO}/setup.bash
                $ ros2 run ros1_bridge dynamic_bridge    
                ```
                

---

- **[Result & Analyzsis]**
    - ROS1-ROS2 bridge terminal 의 결과 창은 다음과 같다.
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros1_ros2_bridge/Untitled.png" alt="">
        </figure>
        
        - (1) Publisher 와 subscriber 의 node 가 **모두 알맞게 활성화**가 되면 위의 assume 과 같이 ROS1 의 topic 이 ROS2 로 이동하는 것이기 때문에 **created 1to2 bridge for topic ~** 으로 나오게 된다.
        - (2) 그리고 어떤 topic 이 이동해서 전달하고 있는지 확인할 수 있다.
            - 예를 들면, 위의 terminal 에서 sensor_msgs/CompressedImage & sensor_msgs/Image 가 변환되었음을 알 수 있다.
        - (3) Publisher 와 subscriber 중 **하나라도 node 가 활성화가 되지 않으면** 마지막처럼 r**emoved 1to2 bridge for topic ~** 으로 나오게 된다.
    
    ---
    
    - ROS1-ROS2 communication Hz 비교
        - ROS master 를 사용하지 않기 때문에 다중 로봇의 데이터 통신에 용이하므로 3대의 NUC 를 이용하여 Hz delay 비교
            - [Case 1] ROS1 → ROS2 / Compressed Image (30Hz) / delay check
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros1_ros2_bridge/ros1_to_ros2_compressed_30Hz_delay.png" alt="">
                </figure>
            
            ---
            
            - [Case 2] ROS2 → ROS1 / Compressed Image (30 Hz) / delay check
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros1_ros2_bridge/ros2_to_ros1_compressed_30Hz_delay.png" alt="">
                </figure>                
            
            ---
            
            - 3대의 로봇의 ROS1 topic 을 bridge 를 거쳐 ROS2 topic 으로 변환하여 1대의 로봇에서 모두 delay check 한 결과는 성능이 매우 안좋았음 **(Bad result ROS1 → ROS2)**
            - 반면, 3대의 로봇의 ROS2 topic 을 bridge 를 거쳐 ROS1 topic 으로 변환하여 1대의 로봇에서 모두 delay check 한 결과는 성능이 매우 좋았음 **(Good result ROS2 → ROS1)**
            - **이 차이가 왜 그런지 아직 잘 모르겠음…;;**
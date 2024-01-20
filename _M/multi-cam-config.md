---
title: "[Multiple camera] How to run multiple intel realsense cameras?"
excerpt: "Multiple Camera Configuration"
---
# [Multiple camera] How to run multiple intel realsense cameras?

- **[Install RealSense package]**
    
    - [[RealSense package] How to use?](https://heathered-freon-621.notion.site/RealSense-package-How-to-use-b20b9d7fc3954dfdbaef48a549be653b)
    

---

- **[Find two connected cameras]**
    - 카메라가 올바르게 연결되어 있는지 확인
        
        ```bash
        $ lsusb
        ```
        
    
    ---
    
    - 다음과 같이 2개의 카메라가 연결되어 있음을 확인
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled.png" alt="">
        </figure>
        

---

- **[Find serial_no in realsense camera]**
    - Realsense viewer 실행
        
        ```bash
        $ realsense-viewer
        ```
        
    
    ---
    
    - 카메라가 연결되어 있으면 다음과 같은 상태를 나타냄
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled 1.png" alt="">
        </figure>


    ---
    
    - 카메라가 여러대가 연결되어 있다면 다음과 같이 추가
        - Add Source → Intel RealSense D455
        - 그러면 다음과 같이 2개의 창이 나타남
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled 2.png" alt="">
          </figure>

    
    ---
    
    - 각 창마다 보이는 Info를 들어가 Serial Number를 확인
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled 3.png" alt="">
        </figure>
        

---

- **[Make launch file]**
    - 카메라를 킬 수 있는 launch file을 group화 하여 각각 틀어주는 새로운 launch file을 생성
        
        ```bash
        <launch>
        	<group ns="camera1">
            <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        			<arg name="serial_no" default="046322250705"/>
        		</include>
        	</group>
        
        	<group ns="camera2">
            <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        			<arg name="serial_no" default="117222250987"/>
        		</include>
        	</group>
        </launch>
        ```
        

---

- **[Result]**
    - 위의 launch file을 실행하면 다음과 같은 결과가 나타나 2개의 연결된 카메라를 사용할 수 있고 2개의 topic으로 나뉘어 있음을 확인
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled 4.png" alt="">
        </figure>

        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled 5.png" alt="">
        </figure>

        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/multi_cam_config/Untitled 6.png" alt="">
        </figure>
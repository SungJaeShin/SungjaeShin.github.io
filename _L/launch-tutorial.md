---
title: "[Launch file] How to make easy configuration"
excerpt: "roslaunch tutorial"
---
# [Launch file] How to make easy configuration

- **[group ns & $ find & arg name]**
    - **group ns:**
        - 하나의 그룹을 묶어 사용하고자 할 때, 따로 build 할 수 있도록 해주는 역할도 한다.
        - 하나의 로봇에서 나온 결과들을 묶을 때 사용하기에도 좋다.
    - **$(find PACKAGE_NAME):**
        - CMakeList.txt 에 있는 package name을 작성해주면 되고 해당 패키지 이름의 directory에 바로 들어갈 수 있도록 만들어 준다.
    - **arg name:**
        - launch file에 설정해주어 해당 이름만 넣어주면 launch file 어디든지 default 값을 적용 시킬 수 있다.
    - **<node name & pkg & type>:**
        - name → 실제 실행할 때 설정하는 이름 → rqt_graph 속에 표현된 이름
        - pkg    → 노드를 포함하고 있는 package 이름
        - type   → 노드를 정의하는 cpp에서 정의한 이름 → ros::init(argc, argv, "vins_estimator");
    - 예를 들면 다음과 같이 설정할 수 있다.
        
        ```bash
        <launch>
        	<arg name="config" default="$(find vins_estimator)/../config/realsense_d435i/realsense.yaml"/>
        
        	<group ns="robot1">
        		<param name="agent_num" type="int" value="1" />
        		<node name="vins_estimator" pkg="vins_estimator" type="vins_estimator" args="$(arg config)" output="screen"/>
        	</group>
        </launch> 
        ```
        
    - 확장된 예시
        
        ```bash
        <launch>
          <arg name="device_type_camera2"    		default="d4.5"/>		
          <arg name="serial_no_camera2"    			default=""/>
          <arg name="camera2"              			default="d400"/>
          <arg name="tf_prefix_camera2"         default="$(arg camera2)"/>
          <arg name="initial_reset"             default="false"/>
          <arg name="clip_distance"             default="-2"/>
          <arg name="topic_odom_in"             default="odom_in"/>
          <arg name="calib_odom_file"           default=""/>
          <arg name="config_file"               default=""/>
        
          <group ns="$(arg camera2)">
            <include file="$(find realsense2_camera)/launch/includes/nodelet.launch.xml">
              <arg name="device_type"           value="$(arg device_type_camera2)"/>
              <arg name="serial_no"             value="$(arg serial_no_camera2)"/>
              <arg name="tf_prefix"		          value="$(arg tf_prefix_camera2)"/>
              <arg name="initial_reset"         value="$(arg initial_reset)"/>
              <arg name="align_depth"           value="true"/>
              <arg name="filters"               value="pointcloud,disparity,spatial,temporal"/>
              <arg name="enable_color"          default="false"/>
              <arg name="color_width"           value="640"/>
              <arg name="color_height"          value="480"/>
              <arg name="depth_width"           value="640"/>
              <arg name="depth_height"          value="480"/>
              <arg name="clip_distance"         value="$(arg clip_distance)"/>
              <arg name="json_file_path"        value="$(arg config_file)"/>
              <arg name="enable_fisheye"        value="false"/>
              <arg name="enable_depth"          value="true"/>
              <arg name="infra_width"           value="640"/>
              <arg name="infra_height"          value="480"/>
              <arg name="enable_infra1"         value="true"/>
              <arg name="enable_infra2"         value="true"/>
              <arg name="color_fps"             value="5"/>
              <arg name="depth_fps"             value="5"/>
              <arg name="infra_fps"             value="30"/>
            </include>
          </group>
        </launch>
        ```
        

---
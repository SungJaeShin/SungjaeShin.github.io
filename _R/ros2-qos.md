---
title: "[ROS2 QoS] Did you know ROS2 communication can improve only changing QoS(Quality of Service) value?"
excerpt: "ROS2 Communication Quality"
---
# [ROS2 QoS] Did you know ROS2 communication can improve only changing QoS(Quality of Service) value?

---

- **[Reference]**
    - [Book] ROS2로 시작하는 로봇 프로그래밍, 표윤석 지음

---

- **[Communication between ROS1 and ROS2]**
    - [ROS1]
        - (1) 한 대의 master 에서 roscore 가 동작
        - (2) master 를 꼭 지정해주어야 하고 이 master 를 통해서 꼭 메세지 publish & subscribe 가 이루어짐
        - (3) ROS1 의 기본적인 communication 방식은 TCP 기반이어서 TCPROS 통신 라이브러리를 사용
    
    ---
    
    - [ROS2]
        - (1) 별도의 master 가 필요가 없음
            - 해당 terminal 에서 terminal 에 **ROS_DOMAIN_ID** 만 동일하게 설정해주면 여러 로봇들간의 데이터 전송이 가능해짐
                
                ```bash
                # Terminal
                $ export ROS_DOMAIN_ID=1
                # you can easily change ROS_DOMAIN_ID ! 
                ```
                
        
        ---
        
        - (2) DDS (Data Distribution Service) 방식의 통신이 이루어지고 RMW (ROS Middleware) 로 추상화하였음
            - 4가지 DDS 방식이 존재
                - [1] rmw_fastrtps_cpp **(ROS2 foxy default)**
                - [2] rmw_cyclonedds_cpp
                - [3] rmw_connext_cpp
                - [4] rmw_gurumdds_cpp
            - bashrc 에서 또는 terminal 에서 **RMW_IMPLEMENTATION** 만 설정해주면 DDS 방식이 변경
                
                ```bash
                $ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
                ```
                
        
        ---
        
        - (3) 노드 간의 데이터 통신을 목적에 맞춰서 세부적으로 조정하는 QoS 를 설정할 수 있어서 다양한 방식으로 DDS 통신을 할 수 있음
            - Reference: [https://docs.ros.org/en/foxy/Concepts/About-Internal-Interfaces.html](https://docs.ros.org/en/foxy/Concepts/About-Internal-Interfaces.html)
                
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros2_qos/Untitled.png" alt="">
                </figure>                 
        
        ---
        
        - ROS1 & ROS2 architecture
            - Reference: [https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7743223](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7743223)
                
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros2_qos/Untitled 1.png" alt="">
                </figure> 
                

---

- **[DDS QoS]**
    - DDS 의 품질을 담당하는 QoS, 쉽게 말하면 데이터 통신 옵션이다.
        - (1) Reliability
            - TCP 처럼 데이터 손실을 방지함으로써 신뢰도를 우선시하는 것에 대한 설정
        - (2) Best effort
            - UDP 처럼 통신 속도를 최우선시하여 사용하는 신뢰성 기능
        - (3) History
            - 통신 상태에 따라 정해진 크기만큼의 데이터를 보관하는 것에 대한 설정
        - (4) Durability
            - 데이터를 수신하는 subscriber 가 생성되기 전의 데이터를 사용할지 폐기할지에 대한 설정
        - (5) Deadline
            - 정해진 주기 안에 데이터가 발신 및 수신되지 않을 경우 이벤트 함수를 실행
        - (6) Lifespan
            - 정해진 주기 안에서 수신되는 데이터만 유효 판정하고 그렇지 않은 데이터는 삭제
        - (7) Liveliness
            - 정해진 주기 안에서 노드 혹은 토픽의 생사를 확인
    
    ---
    
    - rmw_qos_profile
        - ROS 2 의 RMW 에서 QoS 설정을 쉽게 사용할 수 있도록 가장 많이 사용하는 QoS 설정을 세트로 표현해둔 것이 있는데 이를 RMW QoS Profile 이라 한다.
            - Reference: [https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h](https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h)
            - Reference: [https://velog.io/@leesy970906/019-DDS의-QoSQuality-of-Service](https://velog.io/@leesy970906/019-DDS%EC%9D%98-QoSQuality-of-Service)
                
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros2_qos/Untitled 2.png" alt="">
                </figure>                 

---

- **[QoS Profile Example Realsense Camera]**
    - Reference: [https://github.com/zinuok/VINS-Fusion-ROS2/blob/main/realsense_install.sh](https://github.com/zinuok/VINS-Fusion-ROS2/blob/main/realsense_install.sh)
    - Reference: [https://github.com/IntelRealSense/realsense-ros/blob/8285d84813ff7c08bd58698800299f8a50ed36f5/realsense2_camera/src/ros_utils.cpp](https://github.com/IntelRealSense/realsense-ros/blob/8285d84813ff7c08bd58698800299f8a50ed36f5/realsense2_camera/src/ros_utils.cpp)
        
        ```cpp
        static const rmw_qos_profile_t rmw_qos_profile_latched =
        {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };
        ```
        

---

- **[Result]**
    - QoS 방식 중 **rmw_fastrtps_cpp** 을 사용한 rqt_image_view 결과 **(default setting)**
        - rqt_image_view 를 통해서 확인할 수 있듯이, delay 가 매우 커져서 나중에는 안보이는 것으로 확인되었다. 이는 reliable 으로 설정하였기 때문에 (TCP 방식) 해당 topic 이 다시 돌아올 때까지 아얘 기다리므로 화면이 멈춰버리게 된다.
        
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros2_qos/Untitled.gif" alt="">
        </figure> 
        
    - QoS 방식 중 **rmw_cyclonedds_cpp** **(+ Best effort)** 을 사용한 rqt_image_view 결과
        - rqt_image_view 를 통해서 확인할 수 있듯이, 돌아오지 않은 메세지에 대해서는 기다리지 않고 메세지를 받기 때문에 delay 가 나타나지 않음을 알 수 있다.
        
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros2_qos/Untitled 1.gif" alt="">
        </figure> 
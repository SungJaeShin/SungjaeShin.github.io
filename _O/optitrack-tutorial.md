---
title: "[Optitrack] How to use?"
excerpt: "Optitrack Motion Capture Tutorial"
---
# [Optitrack] How to use?

# Motive: Tracker Tool과 연결하여 Ground truth 취득방법

---

### [Part. 1] Optitrack과 server를 연결시키기 위해 사용되는 package

- (1) sudo apt install ros-melodic-vrpn-client-ros
- (2) roscd vrpn_client_ros 들어가기
- (3) sample.launch file 수정
    - <arg name="server" default="192.168.0.2"/> <!-- ip address of optitrack computer -->
    - **여기서, "192.168.0.2"는 Router 주소가 아닌 motive tracker 프로그램을 실행하고 있는 컴퓨터의 IP 주소와 연결!**
- (4) roslaunch vrpn_client_ros sample.launch 실행
    - 보통 rostopic list를 통해서 ground truth topic 이름은 **"vrpn_client_ros/~"** 로 표기됨

### [Part. 2] Motive: Tracker 실행

- (1) 우 상단의 Calibrate Layout 실행 or (View → Camera Calibration)
- (2) 좌측의 Camera Calibration → Calibration 들어감
- (3) 설치된 카메라의 개수만큼의 화면(7개의 camera)에서 빨간색 및 하얀색으로 점들이 detect가 되는데 빨간색은 wanding 하는데에 포함되지 않는 point이고 하얀색의 점은 wanding하는데에 포함되는 point가 됨
    - **하얀색의 point들도 Rigid Body에 포함이 되지 않게 하기 위해**서 Mask Visible 눌러줌
- (4) Wanding을 시작하기 전에 빠르게 calculation을 해주기 위해서 camera frame rate를 **"50Hz → 240Hz"**로 변경
- (5) Start Wanding을 해주게 되고 최소 각 카메라당 3000개 이상 정보를 모아줌
    - **이때, wanding 하는 물체 이외의 marker는 detect 되서는 안됨**
- (6) 정보를 충분히 모아주고 나면 Ground Plane에 들어가고 삼각형 모형을 가지고 실제 바닥 중 아무 곳을 맞추고 그 marker를 묶은 뒤 refine ground plane을 해줌
- (7) 그 뒤 set ground plane을 해주면 해당 coordinate를 저장하도록 경로가 나타나는데 이걸 저장만 하면 global coordinate environment 저장
- (8) 로봇 하나씩 두고 보이는 marker를 drag → 우클릭 후 rigid body → rigid create or rename(이미 있는 경우) 해서 이름을 변경
    
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/optitrack/Untitled.png" alt="">
    </figure>
    
    
    - 변경된 이름이 추후 로봇에서 받는 topic 이름 vrpn_client_ros/"~" 중 "~"여기에 들어가게 됨
- (9) 상단의 graph view라는 것을 활용하여 x 및 y를 0으로 setting
    - UWB의 경우 z축 위로 setting 되어 있기 때문에 그냥 놔둠
- (10) 좌측의 builder에 location의 x, y를 graph view에서 보이는 것과 같이 (-x, -y)만큼 넣어주면서 0으로 default setting 해주게 된다.
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/optitrack/Untitled 1.png" alt="">
    </figure>

    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/optitrack/Untitled 2.png" alt="">
    </figure>
    
    - 여기서, 중앙에 노란점만 움직이게 되는데 나머지 **하얀색 점들은 mocap으로 marker ball이라 움직이지 않는 것**이고 **노란점이 topic을 쏠 때의 pose 값**이 된다.

---

### [Part. 3] 주의사항

- Motive: Tacker에서 **passive and live setting을 default**로 설정
- Motive: Tacker에서 우 상단의 Create Layout에서 다른 model을 건드리지는 말기
- 로봇에서 rostopic echo /tf → from world to "robot names"
- Motive: Tacker에서 inverse matrix (4x4, coordinate transform e.g. x→y, y→x) 데이터가 잘 뽑히는지 확인이 필요
    - data를 취득할 때, view → data streaming plane → optitrack streaming engine에서 **up Axis → Z-up**으로 설정
    - 가끔 data 취득할 때 Y-up으로 되어있는 경우가 있기 때문에 데이터를 취득하기 전 반드시 확인 필요
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/optitrack/Untitled 3.png" alt="">
        </figure>
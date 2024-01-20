---
title: "[ROSBOT 2.0] Direct connection between NUC and CORE2 controller"
excerpt: "ROSBot 2.0 Motor Connection"
---
# [ROSBOT 2.0] Direct connection between NUC and CORE2 controller

## [Goal] ROSBOT 2.0에서 사용하고 있는 SBC인 ASUS Tinker Board와 로봇 바퀴를 돌리는 CORE2 controller를 NUC에서 직접 제어하기 위해서 변경

---

- **[Preference]**
    - ROS package가 설치되어 있는 Ubuntu OS 환경기반
    - Visual Studio Code IDE로 펌웨어를 빌드하기 때문에 설치 필요
        - Download Site: [https://code.visualstudio.com/](https://code.visualstudio.com/)
        
        - VS Code에 필요한 extension 목록 설치
            - Microsoft C/C++ extension 설치
                - Ctrl + P 를 동시에 누르고 "ext install ms-vscode.cpptools" 입력
            - PlatformIO IDE 설치
                - Ctrl + P를 동시에 누르고 "ext install platformio.platformio-ide" 입력

---

- **[전반적인 Process]**
    - (1) ROSBOT 2.0에서  controller(stm32)로 펌웨어를 업로드하는 github page를 다운
        - Reference Site: [https://github.com/husarion/rosbot-stm32-firmware](https://github.com/husarion/rosbot-stm32-firmware)
  
    
    ---
    
    - (2) github에서 다운 받은 폴더 중 **다른 repository를 사용하기 때문**에 다른 저장소를 하나의 sub-folder로 추가할 수 있도록 진행
        
        ```bash
        $ git submodule update --init --recursive
        ```
        
        - 그렇게 되면 lib folder 안에 다른 저장소에 있는 **"core2-imu-driver", "drv88xx-driver-mbed", "encoder-mbed", "rosserial-mbed", "vl5310x-mbed"안에 파일들이 추가**되는 것을 볼 수 있음!!
    
    ---
    
    - (3) github에서 다운 받은 폴더 중 **"lib → rosserial-mbed"** 폴더를 다른 버전으로 변경
        - Reference Site: [https://github.com/byq77/rosserial-mbed/tree/509e7c191bb45ccd98fba0e69865b55df70b3075](https://github.com/byq77/rosserial-mbed/tree/509e7c191bb45ccd98fba0e69865b55df70b3075)
        
    
    ---
    
    - (4) 변경된 rosserial-mbed 폴더 안에있는 **"mbed_lib.json"** 파일 내부에 들어가서 다음을 변경
        - Assume: **USB로 연결한다는 가정**
        
        ```json
        "CORE2":
        {
            "tx_pin": "USBTX",
            "rx_pin": "USBRX",
            "baudrate": 525000,
            "rtos_kernel_ms_tick": 1
        }
        ```
        
    
    ---
    
    - (5) rosbot-stm32-firmware 안의 **"platformio.ini"** 파일안에 들어가서 다음을 제거
        
        ```json
        // remove this lines
        -D ROS_NOETIC_MSGS=1
        ```
        
    
    ---
    
    - (6) Visual Studio Code IDE에서 좌측에 설치된 PlatformIO IDE 버튼을 누르고 **"default → build all"** 을 진행
        - USB port로 core2를 제어한다는 설정 파일 **"firmware.bin"**이 **".pio/core2/"** 에 생성
    
    ---
    
    - (7) ROSBOT 2.0 SBC (Tinker board)에서 flash_firmware.sh를 다음과 같이 변경 후 실행
        
        ```bash
        $ sudo stm32loader -c tinker -u -W
        $ sudo stm32loader -c tinker -e -v -w firmware.bin
        ```
        
        - ROSBOT 2.0에서 Core2에 펌웨어를 업데이트하는 것이기 때문에 tinker를 변경하지 않음
        - 만일, Raspberry Pi에서 Core2로 펌웨어를 올린다면 **"tinker → rpi"**로 수정해야 함
    
    ---
    
    - (8) Type B cable을 ROSBOT 2.0과 NUC에 연결
        - **이제부터는 ROSBOT의 SBC를 사용하지 않음**
    
    ---
    
    - (9) NUC에서 ROSBOT을 제어할 수 있는 package인 rosbot_ekf github 다운
        - Reference Site: [https://github.com/husarion/rosbot_ekf](https://github.com/husarion/rosbot_ekf)
     
    
    ---
    
    - (10) rosserial communication을 위해서 ros에서 제공하는 package를 설치
        
        ```bash
        $ sudo apt install ros-melodic-rosserial* ros-melodic-serial* 
        ```
        
    
    ---
    
    - (11) workspace에 있는 package들이 필요로 하는 연결된 패키지들을 모두 다운
        
        ```bash
        $ rosdep install --from-paths src --ignore-src -r -y
        ```
        
    
    ---
    
    - (12) rosbot_ekf package build 진행 후, **"rosbot_ekf → launch → all.launch"**에서 USB port name과 mbed_lib.json에서 설정한 baudrate로 변경
        
        ```bash
        <include unless="$(arg rosbot_pro)" file="$(find rosbot_ekf)/launch/rosserial_bridge.launch">
        	<arg name="serial_port" default="/dev/ttyUSB0"/>
        	<arg name="serial_baudrate" default="525000"/>
        </include>
        ```
        
    
    ---
    
    - (13) rosbot_ekf에서 all.launch를 실행하면 이제는 /cmd_vel 의 topic 명을 가진다면 ROSBOT의 SBC 없이 제어 가능
        
        ```bash
        # start rosserial communication
        $ roslaunch rosbot_ekf all.launch
        
        # control robot moving using teleop
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
        ```
        

---

- **[platformio.ini을 build하기 위한 다른 방법론]**
    - rosbot-stm32-firmware 안의 platformio.ini 을 build 해주고 uploading을 해주기 위해서 **"PlatformIO Core (CLI)"**을 설치
        - Reference Site: [https://docs.platformio.org/en/latest/core/installation.html](https://docs.platformio.org/en/latest/core/installation.html)
            
    
    ---
    
    - 다음을 간단하게 진행
        - 설치(get-platformio.py 를 불러오기) 및 terminal에서 platformio run을 사용할 수 있게 만들어주는 과정
        - 꼭 설치할 때, python version 문제가 생길 수 있으므로 **"python3"** 로 실행 (******important****** ***줄 참조***)
            
            ```bash
            # Install using `curl`
            $ curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
            $ python3 get-platformio.py ******important******
            
            # Success result 
            $ python3 get-platformio.py 
            
            Installer version: 0.3.5
            Platform: Linux-4.4.71husarion5-armv7l-with-glibc2.25
            Python version: 3.6.9 (default, Jul 17 2020, 12:50:27) 
            [GCC 8.4.0]
            Python path: /usr/bin/python3
            Creating a virtual environment at /home/husarion/.platformio/penv
            Updating Python package manager (PIP) in a virtual environment
            PIP has been successfully updated!
            Virtual environment has been successfully created!
            Installing PlatformIO Core
            Collecting platformio
            
            PlatformIO Core has been successfully installed into an isolated environment `/home/husarion/.platformio/penv`!
            
            The full path to `platformio.exe` is `/home/husarion/.platformio/penv/bin/platformio`
            
            If you need an access to `platformio.exe` from other applications, please install Shell Commands
            (add PlatformIO Core binary directory `/home/husarion/.platformio/penv/bin` to the system environment PATH variable):
            
            # should be able to run PlatformIO from terminal
            $ sudo ln -s ~/.platformio/penv/bin/platformio /usr/local/bin/platformio
            $ sudo ln -s ~/.platformio/penv/bin/pio /usr/local/bin/pio
            $ sudo ln -s ~/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
            ```
            
    
    ---
    
    - 그 뒤 platformio.ini 이 들어있는 rosbot-stm32-firmware 폴더에서 **"platformio run"** 진행
        - Reference Site: [https://docs.platformio.org/en/latest/core/quickstart.html](https://docs.platformio.org/en/latest/core/quickstart.html)
            
        - 여러 error가 등장하는데 그 종류와 해결 방안에 대해서 설명
            - [Error 1] This system supports the C.UTF-8 locale which is recommended. You might be able to resolve your issue by exporting the following environment variables:
            
                export LC_ALL=C.UTF-8

                export LANG=C.UTF-8
            
            - [Solve 1] 위의 설명대로 그냥 차례대로 export LC_ALL=C.UTF-8 해주고 export LANG=C.UTF-8 이거 해주면 끝
            
            ---
            
            - [Error 2] Error: Could not find the package with 'platformio/toolchain-gccarmnoneeabi @ ~1.90201.0' requirements for your system 'linux_armv7l'
            - **[Solve 2] Tool Manager 의 올바른 버전을 찾기 위해서 다음의 site들을 찾아서 설치하려고 했으나 아직 정확하게 해답을 찾지 못함**
            - [Solve 2] VS code로 진행하기
                - Reference Site: [https://github.com/platformio/platformio-core-installer/releases](https://github.com/platformio/platformio-core-installer/releases)
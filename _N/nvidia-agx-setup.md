---
title: "[NVIDIA Jetson AGX Xavier] Setup NVIDIA Jetson board & Install ROS & Install PyTorch and TorchVision"
excerpt: "Setup NVIDIA Jetson AGX Xavier"
---
# [NVIDIA Jetson AGX Xavier] Setup NVIDIA Jetson board & Install ROS & Install PyTorch and TorchVision

---

- **[Goal]**
    - NVIDIA Jetson Board를 구입하여 초기 셋팅을 마칠 수 있다.

---

- **[Reference Site]**
    - Jetpack version to fit Ubuntu version
        
        - [NVIDIA® Jetson™ L4T and JetPack Support – Stereolabs](https://www.stereolabs.com/blog/nvidia-jetson-l4t-and-jetpack-support/)
        
    - Setting Jetson AGX Xavier
        
        - [NVIDIA Jeston 환경 셋팅 1-1편 (JetPack 설치 On AGX Xavier)](https://ropiens.tistory.com/53)
        
    - Jetpack Install
        
        - [NVIDIA SDK Manager](https://developer.nvidia.com/drive/sdk-manager)
        
    - Jetpack version release
        
        - [JetPack Archive](https://developer.nvidia.com/embedded/jetpack-archive)
        
    - SSD mount
        
        - [Installing an NVMe SSD Drive on Nvidia Jetson Xavier](https://medium.com/@ramin.nabati/installing-an-nvme-ssd-drive-on-nvidia-jetson-xavier-37183c948978)
        
    - Xavier AGX ROS install
        
        - [GitHub - jetsonhacks/installROSXavier: Install Robot Operating System (ROS) on the NVIDIA Jetson AGX Xavier Developer Kit](https://github.com/jetsonhacks/installROSXavier)
        
    - PyTorch to fit Jetpack version
        
        - [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
        
    - TorchVision to fit PyTorch version
        
        - [PyTorch](https://pytorch.org/get-started/previous-versions/)
        
    - YOLOv5 install
        
        - [NVIDIA Jeston 환경 셋팅 번외편 (Pytorch, yolov4, yolov5 설치)](https://ropiens.tistory.com/68)
        

---

- **[Prepare Materials]**
    - 노트북
    - M.2. WIFI module
    - 모니터 연결을 위한 HDMI
    - 키보드 및 마우스
    - C type - USB 연결 cable

---

- **[Setup Procedure]**
    
    (1) SSD와 WIFI module 구입
    
    - **(추천!!)** Jetson Xavier AGX는 구매를 하면 64G or 32G의 메모리를 가지고 있기 때문에 용량이 많이 부족하다. 따라서 외부 SSD를 구매해주는 것이 좋다!
        - 내가 구입한 것은 **NVME M.2. SAMSUNG SSD980 (250G)** 이다!
    - **(필수!!)** Jetson Xavier AGX는 **wifi module이 embed되지 않기 때문에 따로 wifi module을 구입**해주어야 한다!!!!
        - 내가 구입한 것은 **Intel® Dual Band Wireless-AC 8265** 이다!
    - **(참고!!)** **M.2. connector**와 맞는 SSD와 WIFI module 구입 !!
    
    ---
    
    (2) NVIDIA Jetpack을 이용하여 NVIDIA Jetson Xavier AGX 설치
    
    - Jetpack은 Jetson board를 위한 전용 SDK로서 기본 OS부터 Deep learning을 위한 cuda, cuDNN, Tensorflow 등 여러 API들을 설치해준다.
    - 본 저자는 처음에 Jetpack을 이용하지 않고 cuda 및 pytorch를 설치하려고 했으나 Jetpack으로 설치하는게 가장 안전하고 올바르기에 이걸로 설치하였다.
        - Reference Site
            
            - [NVIDIA Jeston 환경 셋팅 1-1편 (JetPack 설치 On AGX Xavier)](https://ropiens.tistory.com/53)
            
    
    ---
    
    - (2-1) Jetpack 설치 → **Jetson 보드에 설치하는게 아니라 노트북에 설치 !!**
        - Install Site (최신거 설치해도 무관)
            
            - [NVIDIA SDK Manager](https://developer.nvidia.com/drive/sdk-manager)
            
    
    ---
    
    - (2-2) Jetpack 실행
        - 노트북 terminal 창에 다음과 같은 명령어를 친다.
            
            ```bash
            $ sdkmanager
            ```
            
        - 그러면 다음과 같은 창이 나타난다.
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nvidia_agx_setup/Untitled.png" alt="">
            </figure> 
    
    ---
    
    - (2-3) Jetson AGX Xavier와 노트북 연결하기
        - Target Hardware에 AGX 연결되어 있어야 하는데 이를 못찾는 경우가 있다.
        - 이 경우에는 AGX 하드웨어 Recovery 버튼을 누름으로서 이를 해결한다.
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nvidia_agx_setup/Untitled 1.png" alt="">
            </figure> 

            - **Recovery 버튼을 누르고 동시에 Power 버튼을 같이 누르면** Jetpack SDK에서 detect될 것이다!
            - 올바르게 연결되어 있는 것을 확인하기 위해서는 host PC에 다음과 같은 명령어를 치면 다음과 같은 결과가 나타나야 잘 인식이 된다.
                
                ```bash
                # HOST PC
                $ lsusb
                
                # OUTPUT
                **Bus 004 Device 001: ID ~ NVidia Corp.**
                ```
                
    
    ---
    
    - (2-4) Step 1에서 Target Operating System 설정하기! **[중요!]**
        - Target Jetpack 설치하기 전에 **원하는 Ubuntu 버전에 맞는 Jetpack 버전을 찾고 설치**한다!
            - Reference Site
                
                - [NVIDIA® Jetson™ L4T and JetPack Support – Stereolabs](https://www.stereolabs.com/blog/nvidia-jetson-l4t-and-jetpack-support/)
                
                - [JetPack Archive](https://developer.nvidia.com/embedded/jetpack-archive)
                
            - 본 저자는 **Ubuntu 18.04를 설치하고 싶기 때문에 Jetpack 4.X 버전을 설치**하였다. (최근 기준 4.6.1 설치)
            - 그리고 CONTINUE를 통해 Step 2로 넘어가기!
    
    ---
    
    - (2-5) Step 2에서는 **“I accept the terms and conditions of the license agreements”** 체크해주기!
    
    ---
    
    - (2-6) Step 3에서는 다운로드 중에 다음과 같은 창이 뜨고 UserName & Password를 원하는 것으로 설정하고 설치를 진행한다.
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nvidia_agx_setup/Untitled 2.png" alt="">
        </figure> 
        
        - 여기서 저자는 **“Manual Setup”으로 진행**하였고 설치를 진행하였다.
        - IP Address는 **HOST PC에서 연결되어 있는 IP를 이용**한다.
            - 해당 IP는 ifconfig를 통해서 넣게 된다. (예시: 192.168.0.15)
    
    ---
    
    - (2-7) 중간에 NVIDIA AGX에서 컴퓨터 이름 및 암호를 설정하라는 **System Setup창이 나타난다.** 사용자가 원하는 이름과 비밀번호를 입력한다.
    
    ---
    
    - (2-8) 그러면 Host PC에서는 나머지 CUDA 및 여러 API들을 설치한다. **(약 1시간 반정도 소요)**
    
    ---
    
    - (2-9) 모두 설치가 끝나면 Step 4로 전환이 되고 FINISH AND EXIT를 눌러 설치를 완료시킨다. **(HOST PC는 여기까지만 사용!)**
    
    ---
    
    - (2-10) NVIDIA AGX에 cuda가 제대로 설치가 되었는지 체크를 진행한다.
        - AGX 보드 terminal 창에 다음과 같은 명령어 작성
            
            ```bash
            # Jetson Xavier AGX board Terminal
            $ nvcc --version
            
            # Output
            **nvcc: NVIDIA (R) Cuda compiler drvier** 
            ```
            
    
    ---
    
    (3) 외장 SSD를 Xavier에 mount 시키기
    
    - Jetson 보드는 메모리가 크지 않기 때문에 외장 SSD를 추가 구매하여 이를 mount 시킨다.
    - Reference Site
        
        - [Installing an NVMe SSD Drive on Nvidia Jetson Xavier](https://medium.com/@ramin.nabati/installing-an-nvme-ssd-drive-on-nvidia-jetson-xavier-37183c948978)
        
    
    ---
    
    - (3-1) “Disks”를 search하기
    
    ---
    
    - (3-2) 해당 외장 메모리 disk 클릭 후 Format Partition을 누르고 이름 설정
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nvidia_agx_setup/Untitled 3.png" alt="">
        </figure> 
        
        - 여기서 Type을 Linux system에서만 사용할 것이기 때문에 다음과 같은 설정으로 setting
            - **“Internal disk for use with Linux systems only (Ext4)”**
    
    ---
    
    - (3-3) 설정 후 원하는 local AGX 경로에 mount
        - 먼저, 생성된 partition의 경로를 찾는다.
            
            ```bash
            $ cd /dev
            $ ls | grep nvme
            ```
            
            - 본 저자는 해당 경로에 **“/dev/nvme0n1”**으로 되어 있는 것을 확인
        
        ---
        
        - 그후, 원하는 경로에 해당 partition path를 mount
            
            ```bash
            # Local AGX 
            $ sudo mkdir ${folder_name}
            $ sudo mount /dev/nvme0n1 ${folder_name}
            $ sudo chmod 755 ${folder_name}
            ```
            
            - 여기서 “755”의 의미는 readable by User, Group and World, writable by User, executable by User, Group and World을 담고 있다.
    
    ---
    
    - (3-4) 새롭게 다시 부팅을 진행하면 mount되어 있는 것이 풀리게 되는데 재부팅을 해도 계속 mount 되도록 설정한다.
        - 먼저,  SSD의 UUID 번호를 확인한다.
            
            ```bash
            $ sudo blkid
            
            # Output
            /dev/nvme0n1: LABEL="${SSD_NAME}" UUID="~" TYPE="ext4"
            ```
            
        
        ---
        
        - 그 후 fstab에 SSD mount 정보 입력하기
            - 해당 기존의 fstab 정보를 수정하다가 해당 파일이 손상될 경우를 방지하여 fstab와 같은 경로에 기존의 파일을 복사해둔다.
                
                ```bash
                $ sudo cp /etc/fstab /etc/fstab.bkup
                ```
                
            
            ---
            
            - 그 뒤 fstab에 정보를 추가한다.
                
                ```bash
                $ sudo gedit /etc/fstab
                ```
                
            
            ---
            
            - fstab 맨 아래에 다음과 같이 추가한다.
                
                ```
                **UUID=~ ${folder_name} ext4 defaults 0 0**
                ```
                
        
        ---
        
        - 저장하면 최종적으로 재부팅을 해도 자동적으로 mount된다!!
    
    ---
    
    (4) ROS 설치
    
    - 일반적으로 desktop에서 설치하던 ROS 방법이랑은 살짝 다르기 때문에 해당 github에 들어가서 설치를 진행해주는 것이 좋다.
        - Reference Site
            
            - [[NVIDIA] Jetson AGX Xavier에 ROS 설치하기](https://m.blog.naver.com/tinz6461/221793599149)
            
            - [GitHub - jetsonhacks/installROSXavier: Install Robot Operating System (ROS) on the NVIDIA Jetson AGX Xavier Developer Kit](https://github.com/jetsonhacks/installROSXavier)
            
            - 이건 **Xavier AGX 용 ROS 설치 shell file**이므로 다른 하드웨어로 ROS를 설치하려면 다른 github을 찾아야 한다!!
    
    ---
    
    - 해당 github 사이트에 들어가서 **“installROS.sh”** 를 다운받고 터미널에 다음과 같이 입력해준다.
        
        ```bash
        $ ./installROS.sh -p ros-melodic-desktop-full
        ```
        
    
    ---
    
    - 설치 완료!
    
    ---
    
    (5) Pytorch 설치
    
    - Jetpack 버전에 맞는 PyTorch를 설치해야 한다.
        - Reference Site
            
            - [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
            
            - 여기서 본 저자는 JetPack 4.6.X 버전을 설치하였으므로 **PyTorch 버전은 1.10.0으로 설치**하였다.
    
    ---
    
    - 위의 사이트에서 맞는 “.whl” 파일을 다운을 받으면 다음과 같이 설치를 진행한다.
        
        ```bash
        $ sudo apt-get install python3-pip libopenblas-base libopenmpi-dev libomp-dev
        $ pip3 install Cython
        $ pip3 install numpy torch-1.10.0-cp36-cp36m-linux_aarch64.whl
        ```
        
    
    ---
    
    - PyTorch가 설치가 되었는지 확인
        
        ```bash
        $ python3
        
        # Python3 in Terminal
        Python 3.6.9
        >>> import torch
        >>> torch.__version__
        '1.10.0'
        ```
        
    
    ---
    
    (6) Torchvision 설치
    
    - Pytorch 버전에 맞는 Torchvision을 설치해야 한다.
        - Reference Site
            
            - [PyTorch](https://pytorch.org/get-started/previous-versions/)
            
            - 여기서 본 저자는 PyTorch 1.10.0 버전을 설치하였으므로 **Torchvision 버전은 0.11.0으로 설치**하였다.
    
    ---
    
    - torchvision은 pytorch 설치와는 다르게 git으로 clone하여 설치를 진행한다. **(시간 소요가 길다!)**
        
        ```bash
        $ sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
        $ git clone --branch **v0.11.0** https://github.com/pytorch/vision torchvision
        cd torchvision
        sudo python3 setup.py install
        ```
        

---

- **[YOLOv5 Application]**
    - Jetson board에 기초적인 설치가 모두 완료되었다면 YOLOv5 모델을 설치하여 확인해본다.
        - Reference Site
            
            - [NVIDIA Jeston 환경 셋팅 번외편 (Pytorch, yolov4, yolov5 설치)](https://ropiens.tistory.com/68)
            
        
        ---
        
        - Error Related
            
            (Error 1) **ModuleNotFoundError: No module named 'numpy.testing.nosetester’**
            
            (Reference Site)
            
            [ModuleNotFoundError: No module named 'numpy.testing.nosetester'](https://stackoverflow.com/questions/59474533/modulenotfounderror-no-module-named-numpy-testing-nosetester)
            
            ---
            
            (Solution) 설치된 scipy 버전과 numpy간의 버전 충돌이 있기 때문에 다음과 같이 변경 해주어야 한다.
            
            - scipy ≥ 1.0.0
            - numpy == 1.17.0
            
            ```bash
            $ sudo pip3 uninstall numpy
            $ sudo apt-get install libatlas-base-dev gfortran
            $ sudo pip3 install numpy==1.17.0
            $ sudo pip3 install -U scipy
            ```
            
            ---
            
            (Error 2) ImportError: cannot import name 'remove_na’
            
            (Reference Site)
            
            [2장 p.97 Seaborn에 관한 문제 · Issue #82 · PinkWink/DataScience](https://github.com/PinkWink/DataScience/issues/82)
            
            [Error importing Seaborn module in Python: "ImportError: cannot import name utils"](https://stackoverflow.com/questions/28828917/error-importing-seaborn-module-in-python-importerror-cannot-import-name-utils)
            
            ---
            
            (Solution) seaborn이 제대로 설치되어 있지 않기 때문에 다시 설치해주어야 한다.
            
            ```bash
            $ pip3 install seaborn
            ```
            
    
    ---
    
    - Results
        - YOLOv5x 모델의 detection 결과는 다음과 같다.
            - bus
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nvidia_agx_setup/Untitled.jpeg" alt="">
                </figure>                 
            
            ---
            
            - zidane
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nvidia_agx_setup/Untitled 1.jpeg" alt="">
                </figure>                 
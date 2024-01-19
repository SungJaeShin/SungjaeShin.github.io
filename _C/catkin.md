---
title: "[Catkin] Tip for development"
excerpt: "Catkin Tips"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [CATKIN] Tip for development

- **[Build]**
    
    ```bash
    # To see catkin configuration 
    $ catkin config
    
    # To setting additional Camke Args
    $ catkin config -DCMAKE_BUILD_TYPE=Release
    
    # To setting additional Make Args
    $ catkin config --make-args -j2
    ```
    
    - "-DCMAKE_BUILD_TYPE=Release"
        - CMake가 실제로 compiler를 호출할 때 (build 하는 경우), 여러 가지 flag가 있는데 대표적으로 **"Release & Debug"** 가 있다.
        - Release flag로 빌드할 때는 **최적화를 수행하고 Debug 정보를 생략**하도록 compiler에게 지시한다.
            - **[장점 1] 더 빠르게 build 할 수 있고 코드가 더 빠르게 돌아갈 수 있다.**
            - **[장점 2] 더 좋은 성능의 결과를 보인다.**
        - 이 명령어를 안사용한다면 **default로 debug 모드로 build**하게 된다.
    - "-j2"
        - CMake가 build를 하는 경우, CPU core를 몇 개 사용할지에 대한 flag가 된다.
        - 기본적으로 catkin build를 하는 경우 하드웨어가 가지고 있는 전체 core 자원을 지유롭게 사용하도록 한다.
            - 만약 제한된 core를 가진다면, 다른 프로세스를 진행할 수 없어 컴퓨터가 멈춰버린다.

---

- **[Catkin Workspace]**
    - 보통 **하나의 workspace에 여러 개의 package를 다 넣는 것은 좋지 않다.**
        - 보통 관련된 것들 끼리 묶어주고 변하지 않는 것들을 묶어주는게 좋다.
    - 만약에 workspace에서 요구하는 **dependencies 들을 자동으로 모두 설치**하고 싶다면 다음과 같은 명령어를 터미널에 작성해준다.
        
        ```bash
        # Example of ROS_DISTRO=melodic 
        $ rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ($ROS_DISTRO) -y
        $ rosdep install --from-paths src --ignore-src -r -y
        ```
        

---

- **[Catkin Module Shared Libraries]**
    - catkin build를 통해서 workspace 에 있는 package 를 만들었는데 이때 이 **package 에 연결되어 있는 library들을 살펴보는 방법**이 있다.
        - 이 package에 연결되어 있는 **(1) library 들이 무엇인지, (2) 해당 library의 version이 무엇인지** 확인하기 좋다는 장점이 있다.
    - workspace 안에 devel 속에 보이지 않지만 해당 package들의 shared library 파일이 있고 이는 **.private** 안에 있다.
        - Shared libraries 들을 보는 방법
            - Terminal에 **“ldd {executive file name}”**을 치면 됨
            
            ```bash
            # In terminal
            **$ ldd ${executable_file_name}**
            ```
            
        - Example of Action Tutorial
            - Code : [GitHub - SungJaeShin/Action_tutorial](https://github.com/SungJaeShin/Action_tutorial.git)

            - 위치: **/${workspace}/devel/.private/Action_tutorial/lib/Action_tutorial**
            
            ```bash
            # In following workspace: ~/devel/.private/Action_tutorial/lib/Action_tutorial
            **client*  server***
            
            # To see shared libraries
            $ ldd client
            
            # Result
            linux-vdso.so.1 (0x00007fffaf956000)
            libactionlib.so => /opt/ros/melodic/lib/libactionlib.so (0x00007fbf3c5e7000)
            libroscpp.so => /opt/ros/melodic/lib/libroscpp.so (0x00007fbf3c252000)
            librosconsole.so => /opt/ros/melodic/lib/librosconsole.so (0x00007fbf3c01c000)
            libroscpp_serialization.so => /opt/ros/melodic/lib/libroscpp_serialization.so (0x00007fbf3be19000)
            librostime.so => /opt/ros/melodic/lib/librostime.so (0x00007fbf3bbf9000)
            libboost_system.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1 (0x00007fbf3b9f4000)
            libboost_thread.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.65.1 (0x00007fbf3b7cf000)
            libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007fbf3b5b0000)
            libstdc++.so.6 => /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007fbf3b227000)
            libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007fbf3b00f000)
            libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fbf3ac1e000)
            libxmlrpcpp.so => /opt/ros/melodic/lib/libxmlrpcpp.so (0x00007fbf3a9ff000)
            libcpp_common.so => /opt/ros/melodic/lib/libcpp_common.so (0x00007fbf3a7f4000)
            libboost_chrono.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.65.1 (0x00007fbf3a5ef000)
            libboost_filesystem.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.65.1 (0x00007fbf3a3d5000)
            libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fbf3a037000)
            librosconsole_log4cxx.so => /opt/ros/melodic/lib/librosconsole_log4cxx.so (0x00007fbf39e1c000)
            librosconsole_backend_interface.so => /opt/ros/melodic/lib/librosconsole_backend_interface.so (0x00007fbf39c1a000)
            liblog4cxx.so.10 => /usr/lib/x86_64-linux-gnu/liblog4cxx.so.10 (0x00007fbf39851000)
            libboost_regex.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.65.1 (0x00007fbf39549000)
            librt.so.1 => /lib/x86_64-linux-gnu/librt.so.1 (0x00007fbf39341000)
            /lib64/ld-linux-x86-64.so.2 (0x00007fbf3ca76000)
            libconsole_bridge.so.0.4 => /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4 (0x00007fbf3913c000)
            libapr-1.so.0 => /usr/lib/x86_64-linux-gnu/libapr-1.so.0 (0x00007fbf38f07000)
            libaprutil-1.so.0 => /usr/lib/x86_64-linux-gnu/libaprutil-1.so.0 (0x00007fbf38cdc000)
            libicui18n.so.60 => /usr/lib/x86_64-linux-gnu/libicui18n.so.60 (0x00007fbf3883b000)
            libicuuc.so.60 => /usr/lib/x86_64-linux-gnu/libicuuc.so.60 (0x00007fbf38483000)
            libuuid.so.1 => /lib/x86_64-linux-gnu/libuuid.so.1 (0x00007fbf3827c000)
            libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007fbf38078000)
            libcrypt.so.1 => /lib/x86_64-linux-gnu/libcrypt.so.1 (0x00007fbf37e40000)
            libexpat.so.1 => /lib/x86_64-linux-gnu/libexpat.so.1 (0x00007fbf37c0e000)
            libicudata.so.60 => /usr/lib/x86_64-linux-gnu/libicudata.so.60 (0x00007fbf36065000)
            ```
            

---
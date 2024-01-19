---
title: "[Boost Error] /usr/bin/ld: warning: libboost_system.so.1.65.1, needed by /opt/ros/melodic/lib/libroscpp.so, not found (try using -rpath or -rpath-link) /opt/ros/melodic/lib/libroscpp.so: undefined reference to boost::system::system_category()' /opt/ros/melodic/lib/librosconsole.so: undefined reference to boost::re_detail_106501::cpp_regex_traits_implementation<char>::transform_primary[abi:cxx11](char const*, char const*) const'"
excerpt: "Boost Error 5"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Boost Error] /usr/bin/ld: warning: libboost_system.so.1.65.1, needed by /opt/ros/melodic/lib/libroscpp.so, not found (try using -rpath or -rpath-link) /opt/ros/melodic/lib/libroscpp.so: undefined reference to boost::system::system_category()' /opt/ros/melodic/lib/librosconsole.so: undefined reference to boost::re_detail_106501::cpp_regex_traits_implementation<char>::transform_primary[abi:cxx11](char const*, char const*) const'

---

- **[Goal]**
    - ROS package와 연관되어 있는 boost version을 확인하고 올바르게 해결할 수 있다.

---

- **[Reference Site]**
    
    - [REP 3 -- Target Platforms (ROS.org)](https://www.ros.org/reps/rep-0003.html#melodic-morenia-may-2018-may-2023:~:text=1.62-,1.65.1!,-1.62)
    

---

- **[Cause]**
    - ROS distribution 마다 최소 package 버전 또는 fit한 version들이 존재하는데 이들이 local에 설치되어 있는 version에 맞춰 설치를 꼭 진행해주어야 한다.
    - 모든 ros-melodic-XXX 페키지들을 매번 git clone하여 install 하기 어렵다면 boost version을 맞춰서 설치를 해주어야 한다.
        - 즉, `sudo apt-get install ros-melodic-rosconsole`을 설치하면 boost 1.65.1 dependency에 맞게 local에 설치가 됨
        - 본인이 boost version이 1.69.0이라면 boost 버전을 바꾸던지 rosconsole git clone을 하여 1.69.0을 install 할지 선택해야함
        - 그런데 git clone을 할꺼면 엄청 많은 package들을 clone해야하는데 너무 귀찮고 많아서 boost version을 변경하기로 선택함

---

- **[Process]**
    - [Step 1] `dpkg -l | grep libboost-dev` 을 통해 local에 설치된 boost version check
        - 저자의 경우 다음과 같이 결과 나타남
            
            ```bash
            (base) sj@sj:~$ dpkg -l libboost-dev
            Desired=Unknown/Install/Remove/Purge/Hold
            | Status=Not/Inst/Conf-files/Unpacked/halF-conf/Half-inst/trig-aWait/Trig-pend
            |/ Err?=(none)/Reinst-required (Status,Err: uppercase=bad)
            ||/ Name                    Version          Architecture     Description
            +++-=======================-================-================-====================================================
            ii  libboost-dev:amd64      1.65.1.0ubuntu1  amd64            Boost C++ Libraries development files (default versi
            ```
            
    - [Step 2] 저자의 경우 ros melodic 을 설치하였는데 1.65.1 버전의 boost를 설치 진행하여야 함
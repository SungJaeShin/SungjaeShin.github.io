---
title: "[Boost Error] CMake Error at /usr/local/share/cmake-3.25/Modules/FindPackageHandleStandardArgs.cmake:230 (message):
Could NOT find Boost (missing: python) (found version "0.0.0")"
excerpt: "Boost Error 3"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Boost Error] CMake Error at /usr/local/share/cmake-3.25/Modules/FindPackageHandleStandardArgs.cmake:230 (message): Could NOT find Boost (missing: python) (found version "0.0.0")

---

- **[Goal]**
    - Boost python version에 관한 lib을 찾지 못하는 경우를 해결할 수 있다.

---

- **[Reference Site]**
    
    - [Cmake doesn't find Boost](https://stackoverflow.com/questions/3808775/cmake-doesnt-find-boost)
    
    - [cv_bridge: boost-python not found (Noetic, Fedora 32) - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/360283/cv_bridge-boost-python-not-found-noetic-fedora-32/)
    
    [](https://github.com/ros-perception/vision_opencv/blob/0a908b0bcf33a4374b26b8572860ff48da19f74a/cv_bridge/CMakeLists.txt)
    

---

- **[Process]**
    - [Step 1] CMake가 해당 python의 경로를 찾지 못하기 때문에 생기는 원인이기 때문에 설치된 경로를 찾고 `CMakeList.txt`에 absolute path를 및 다음과 같은 경로를 추가
        - Boost 설치 경로 확인
            - `sudo find /usr/ -name “*boost*”`
                - 본 저자의 경우 include는 “`/usr/include/boost`” 여기에, library는 “`usr/local/lib`” 여기에 있음을 확인
        - build할 package의 경로를 CMakeList.txt에 다음과 같은 코드 삽입
            
            ```bash
            SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/usr/include/boost")
            SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/usr/local/lib")
            
            FIND_PACKAGE(Boost)
            IF (Boost_FOUND)
                INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
                ADD_DEFINITIONS( "-DHAS_BOOST" )
            ENDIF()
            ```
            
    - [Step 2] 추가로 build할 Boost package의 absolute path를 export
        
        ```bash
        export BOOST_INCLUDE_DIR="/usr/local/include"
        export BOOST_LIBRARYDIR="/usr/local/lib"
        export BOOST_ROOT="/usr/include/boost"
        ```
        
    - [Step 3] `catkin clean` 후 `catkin build` 진행

---

- **[Additional Error in cv_bridge]**
    - 위의 방법을 적용하여도 같은 error가 발생하나 found version이 변경된 경우
        
        ```bash
        **CMake Error at /usr/local/share/cmake-3.25/Modules/FindPackageHandleStandardArgs.cmake:230 (message):
        Could NOT find Boost (missing: python) (found version "1.65.1")**
        ```
        
    
    ---
    
    - [Process]
        - [Step 1] 먼저 libboost_python 라이브러리가 설치된 버전이 어떤 것인지 먼저 확인
            - `sudo find /usr/ -name “libboost_python*”`
                - 본 저자의 경우 다음과 같이 버전이 나타남 (해당 버전은 **python3**임)
                    
                    ```bash
                    /usr/local/lib/libboost_python3.so
                    /usr/local/lib/libboost_python3.so.1.65.1
                    /usr/local/lib/libboost_python3.a
                    ```
                    
                    - 참고로 Boost-1.69.0 의 경우 **python3.8**임
        - [Step 2] cv_bridge 의 CMakeList.txt 중 다음과 같은 Line들을 수정
            - Before
                
                ```bash
                if(NOT ANDROID)
                	find_package(PythonLibs)
                	if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
                	  find_package(Boost REQUIRED python)
                	else()
                		find_package(Boost REQUIRED python3)
                endif()
                ```
                
            - After **(Boost REQUIRED 뒤에 같은 버전으로 모두 변경)**
                
                ```bash
                if(NOT ANDROID)
                	find_package(PythonLibs)
                	if(PYTHONLIBS_VERSION_STRING VERSION_LESS **"3"**)
                	  find_package(Boost REQUIRED **python3**)
                	else()
                		find_package(Boost REQUIRED **python3**)
                endif()
                ```
                
                - Boost-1.69.0 인 경우라면 `python3 → python3.8` 로 작성하면 끝!
        - [Step 3] `catkin clean` 후 `catkin build` 진행
---
title: "[OpenCV Error] libopencv_~.so.3.2: cannot open shared object file: No such file or directory"
excerpt: "OpenCV Error 3"
---
# [OpenCV Error] libopencv_~.so.3.2: cannot open shared object file: No such file or directory

---

- **[Goal]**
    - OpenCV가 설치되어 있음에도 불구하고 g++ 기반 컴파일을 진행할 때 문제가 생기는 것을 막을 수 있다.

---

- **[Reference Site]**
    
    - [libopencv_imgcodecs.so.3.2: cannot open shared object file: No such file or directory](https://stackoverflow.com/questions/43152412/libopencv-imgcodecs-so-3-2-cannot-open-shared-object-file-no-such-file-or-dire)
    

---

- **[How to Solve?]**
    - 기존 방식
        
        ```cpp
        g++ ./src/~.cpp -L/usr/local/include/opencv2 -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_stitching -lopencv_xfeatures2d -lopencv_features2d -lopencv_calib3d -lopencv_flann -o start
        ```
        
    - 해당 문제 해결 방법
        - libopencv_~.so.3.2 가 있는 위치를 먼처 파악
            
            → 해당 저자는 `/usr/local/lib` 에 있는 것을 확인
            
        - 해당 경로를 g++에 넣기 전에 LD_LIBRARY_PATH에 경로 추가
            
            → `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib`
            
        - g++ 안에도 추가 경로 삽입
            
            → -L/usr/lib
            
    - 최종 변경
        
        ```cpp
        **export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib**
        g++ ./src/~.cpp -L/usr/local/include/opencv2 **-L/usr/lib** -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_stitching -lopencv_xfeatures2d -lopencv_features2d -lopencv_calib3d -lopencv_flann -o start
        ```
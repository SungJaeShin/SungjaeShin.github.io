---
title: "[NVIDIA Jetson AGX Xavier] cv_bridge error related"
excerpt: "NVIDIA Jetson AGX Xavier Error"
---
# [NVIDIA Jetson AGX Xavier] cv_bridge error related

---

- **[Goal]**
    - Jetson 보드이기에 생기는 에러를 해결할 수 있다.

---

- **[Error Related]**
    - **[Error 1] Project 'cv_bridge' can not find opencv**
        - [Reference Site]
            
            - [Project 'cv_bridge' can not find opencv · Issue #345 · ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv/issues/345)
            
    
    ---
    
    - **[Solution 1] Jetson board에 설치되어 있는 파일 이름이 “/usr/include/opencv” 아니라 “/usr/include/opencv4”이기 때문에 이를 변경해야한다.**
        - (Step 1)
            
            ```bash
            $ sudo gedit /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
            ```
            
        
        ---
        
        - (Step 2)
            - 해당 문장 수정
                
                ```bash
                # Before 
                set(_include_dirs "include;/usr/include;/usr/include/opencv")
                
                **# After !!!
                set(_include_dirs "include;/usr/include;/usr/include/opencv4")**
                ```
                
    
    ---
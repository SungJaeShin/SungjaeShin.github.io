---
title: "[Boost Error] make[2]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libboost_regex.so', needed by '/home/~/catkin_ws/${Algorithm}/devel/.private/cv_bridge/lib/libcv_bridge.so'. Stop."
excerpt: "Boost Error 4"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Boost Error] make[2]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libboost_regex.so', needed by '/home/~/catkin_ws/${Algorithm}/devel/.private/cv_bridge/lib/libcv_bridge.so'. Stop.

---

- **[Goal]**
    - Build를 진행할 때, Boost 및 cv_bridge와 관련된 문제를 해결할 수 있다.

---

- **[Cause]**
    - Target 경로에 주어진 libboost_regex.so가 없어서 발생하는 문제이기 때문에 설치된 경로에 있는 library들을 target 경로에 복사하여 붙여넣어주면 됨

---

- **[Process]**
    - 설치한 lib을 그대로 target 경로에 복사 및 붙여넣기 진행
        
        ```bash
        $ sudo cp -p ${INSTALL_BOOST_LIB}/libboost_* /usr/lib/x86_64-linux-gnu/
        ```
        
        - 저자의 경우, 다음과 같이 진행
            
            ```bash
            $ sudo cp -p /usr/local/lib/libboost_* /usr/lib/x86_64-linux-gnu/
            ```
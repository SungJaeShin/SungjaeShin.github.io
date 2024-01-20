---
title: "[ROS Master] Is it possible to transmit data between nodes even if the ROS master is disconnected?"
excerpt: "Understand ROS Master"
---
# [ROS Master] Is it possible to transmit data between nodes even if the ROS master is disconnected?

---

- **[Goal]**
    - ROS Master를 중간에 멈추었을 때, 나머지 노드들 간 데이터를 전송할 수 있는지 확인할 수 있다.
    - ROS Master와 데이터 전송간의 관계는 어떤 것인지 확인할 수 있다.
    - 데이터 전송에 가장 중요한 요소는 무엇인지 확인할 수 있다.

---

- **[ROS Network]**
    - Reference Site:
        - [https://kr.mathworks.com/help/ros/ug/ros-network-setup.html](https://kr.mathworks.com/help/ros/ug/ros-network-setup.html)
        - [https://trojrobert.github.io/hands-on-introdution-to-robot-operating-system(ros)/](https://trojrobert.github.io/hands-on-introdution-to-robot-operating-system%28ros%29/)
            
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros_master/Untitled.png" alt="">
            </figure>
            
    - 일반적으로 Node 간 통신을 위해서 ROS1에서는 ROS Master라는 개념을 사용하고 있다.
    - 여기서 **ROS Master는 쉽게 생각하면 Register Node로서 어떤 topic들이 publish되고 어떤 topic들이 subscriber되는지 작성하는 명부같은 역할**을 한다.
    - 그러면 한번 등록된 node들의 정보는 서로 통신이 가능하게 된다.
        - **Master를 거쳐 데이터를 주고 받는것이 아니다!!!!!**
        - **Master는 단지 등록만해주는 node라고 생각하자!!!**

---

- **[Experiment Setup for ROS Master and Communication]**
    - **[Case 1] NUC1 (Master) ↔ NUC2 (Pub Node & Sub Node)**
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros_master/Untitled 1.png" alt="">
        </figure>
        
        - 실험1:
            - (Step 1) NUC1 와 NUC2 위의 그림처럼 구성되어 있음
            - (Step 2) NUC2가 네트워크 통신망을 벗어남
            - (Step 3) NUC2가 다시 같은 통신망으로 들어감
        - 실험2:
            - (Step 1) NUC1 와 NUC2 위의 그림처럼 구성되어 있음
            - (Step 2) NUC1의 Master를 정지시킴
            - (Step 3) NUC2가 네트워크 통신망을 벗어남
            - (Stpe 4) NUC2가 다시 같은 통신망으로 들어감
    
    ---
    
    - **[Case 2] NUC1 (Master & (Pub Node or Sub Node)) ↔ NUC2 (Pub Node or Sub Node)**
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/ros_master/Untitled 2.png" alt="">
        </figure>
        
        - 실험1:
            - (Step 1) NUC1 와 NUC2 위의 그림처럼 구성되어 있음
            - (Step 2) NUC2가 네트워크 통신망을 벗어남
            - (Step 3) NUC2가 다시 같은 통신망으로 들어감
        - 실험2:
            - (Step 1) NUC1 와 NUC2 위의 그림처럼 구성되어 있음
            - (Step 2) NUC1의 Master를 정지시킴
            - (Step 3) NUC2가 네트워크 통신망을 벗어남
            - (Stpe 4) NUC2가 다시 같은 통신망으로 들어감

---

- **[Experiment Results]**
    - [Results of Case 1]
        - 실험1
            - NUC2가 Master와 네트워크 통신망을 벗어나게 되면 pub & sub node가 멈춘다.
                - **(중요!!) 정확하게는 publisher는 계속 진행이되는데 subscriber node가 멈춤!!**
            - NUC2가 다시 동일한 네트워크 통신망에 들어오게 되면 NUC2의 pub & sub node가 다시 진행이 된다.
                - **(중요!!) 통신망을 벗어난 시간의 데이터는 손실이 된다!!**
        - 실험2
            - Case 1 처럼 setting 후 ROS Master를 중단시켜도 NUC2의 pub & sub node는 계속 데이터를 주고 받는다.
            - 실험1 과 같이 네트워크 통신망을 벗어나게 되면 NUC2의 pub & sub node가 멈춘다.
            - 마찬가지로 실험 1 과 같이 NUC2가 다시 동일한 네트워크 통신망에 들어오게 되면 NUC2의 pub & sub node가 다시 진행이 된다..
    
    ---
    
    - [Results of Case 2]
        - 실험 1
            - NUC2가 Master와 네트워크 통신망을 벗어나게 되면 pub인 경우는 계속 Publish를 진행하나 Subscribe node라면 멈춘다.
            - 허나 다시 NUC2가 다시 동일한 네트워크 통신망에 들어오게 되면 NUC2의 pub or sub node가 다시 진행이 된다.
        - 실험 2
            - Case 1의 실험 2처럼 setting 후 ROS Master를 중단시켜도 NUC1의 pub & sub node와 NUC2의 pub & sub node는 계속 데이터를 주고 받는다.
            - NUC2가 Master와 네트워크 통신망을 벗어나게 되면 pub인 경우는 계속 Publish를 진행하나 Subscribe node라면 멈춘다.
            - 허나 다시 NUC2가 다시 동일한 네트워크 통신망에 들어오게 되면 NUC2의 pub or sub node가 다시 진행이 된다.
    
    ---
    
    - [Others]
        
        (1) 공통적인 것인데 다시 같은 통신망으로 들어올 때 초기 성능보다 저하된 결과를 가지고 오는 경우가 있다. 이 경우는 **데이터 사이즈가 크면 성능 저하**가 오는 것으로 보인다. 
        
        (2) ROS 2도 결국 비슷한 결과를 가지고 왔는데 ROS1 보다 대체로 성능 저하가 있다. 정확한 원인은 모르나 ROS 2 communication 이 제대로 구성되어 있지 않아서 그런듯해보인다. 
        

---

- **[Conclusion]**
    - **ROS Master Setting을 처음에만 해주면 노드간 데이터를 주고 받는 것은 Master를 통해서 이동하지 않고 노드끼리 주고 받게된다.**
    - **ROS Master Setting을 처음에만 해주고나면 master가 동작을 하지 않아도 노드끼리 데이터를 주고 받는다.**
    - 데이터를 주고 받는 것은 통신망이 가장 중요하다. 즉 같은 통신망에만 있는 것이 보장이 되면 setting 후 master가 멈추더라도 노드끼리 데이터를 주고 받는다.
    - 같은 통신망에서 벗어나면 publisher 노드만 계속 publish 동작을 하고 subscriber 노드는 멈춰서 동작을 하지 않게 되는 것처럼 보인다.
    - 다시 같은 통신망으로 들어가면 중단되었던 데이터 통신은 정상적으로 동작한다.
    - ROS2도 동일한 결과를 가지고 오는데 ROS1과의 차이는 ROS1은 서로 로봇 간 데이터 주고 받는 통로가 설정이 되어 있어야하고 ROS2는 통신망 안에서 데이터를 뿌리고 받는 형식이 된다.
    - **[결론!!!] ROS 메세지 통신에서 가장 중요한 역할은 Master가 아니라 같은 통신망에 있어야 한다는 점이다!!!!!**

---

- **[Other Reference Site]**
    
    - [ROS publishers & subscribers still transmitting data after roscore closed - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/329385/ros-publishers-subscribers-still-transmitting-data-after-roscore-closed/)
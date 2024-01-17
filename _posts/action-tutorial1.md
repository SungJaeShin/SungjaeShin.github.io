---
title: "[Action Tutorial] Basic concept of ROS Action"
toc: true
---

# [Action Tutorial] Basic concept of ROS Action

## [Goal] ROS에서 사용되는 action에 대해 개념을 알아보기

---

- **[Result Github]**
    
    [GitHub - SungJaeShin/Action_tutorial](https://github.com/SungJaeShin/Action_tutorial.git)
    

---

- **[Reference Site]**
    - Action Summary
        
        [Wiki](http://wiki.ros.org/actionlib#CA-01a90787f036b7f609402261d9a26d106ea379bb_1)
        
        [10. action, action server, action client 설명](https://swimminglab.tistory.com/100)
        
    
    ---
    
    - Action Class
        
        [Wiki](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client)
        
        [Wiki](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
        
    
    ---
    
    - Donguk Seo Github
        
        [GitHub - SeoDU/RRT](https://github.com/SeoDU/RRT.git)
        

---

- **[Problem Definition]**
    - Subscriber 의 경우에는 ros::spin()에 의해 지속적으로 메세지를 queue에 저장을 하는데 이들이 있는 main문에서 주로 사용됨
    - 하지만, 메세지 정보를 주고 받는 함수가 따로 정의한 외부함수(예를 들면, node A의 function B, node C의 function D에서, B와 D에서 메세지를 주고 받는 경우)라면 이때는 subscriber를 사용할 수 없다고 생각이 들었음
        - 해당 정보를 다른 노드에 주고 결과 값이 생길 때 받는 행위를 할 수 있는 action을 공부하게 됨

---

- **[Subscriber & Service & Action]**
    - Subscriber: ros::spin()이 계속 돌면서 queue에 해당 메세지를 넣어주는 작업 진행 (지속적으로 진행)
    - Service: 해당 메세지가 어떤 조건을 만족했을 때 메세지를 받아주는 작업 진행 (특정 조건에 사용)
    - Action: 원하는 정보를 담아 보내면 해당 결과를 받는 작업 진행 (특정 조건에 사용)
    
    ---
    
    - Service vs Action 의 차이
        - Service는 여러 개가 존재하면 순차적으로 조건이 만족하면 다음 service로 넘어감 (현재 진행중인 service의 request가 처리되는 동안, 다른 service는 기다려야함)
        - Action은 여러 개가 존재하면 첫번 째의 action에 대한 결과가 나오지 않았음에도 불구하고 다른 작업이 진행 가능 (여러 개의 topic이 같이 실행 될 수 있음)

---

- **[Action]**
    <figure class="align-center">
    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/action_tutorial.png" alt="">
    <figcaption>Look at 580 x 300 getting some love.</figcaption>
    </figure> 

    <!-- ![Untitled](%5BAction%20Tutorial%5D%20Basic%20concept%20of%20Action%2053e29725788e4bff871bb4e6452f2b52/Untitled.png) -->
    
    ---
    
    - Action에는 **Client와 Server라는 개념**이 존재하고 각각 노드라고 인식하면 훨신 이해하기 편함
        - **Client**: 원하는 결과를 얻기 위한 정보를 server에 보내는 역할 및 server에서 처리하고 원하는 결과를 수신
        - **Server**: client가 원하는 결과를 얻기 위한 정보를 받아 처리 후 원하는 결과를 client에 전송
    
    ---
    
    - Action에는 5가지 topic을 주고 받음
        - **Client →Server**
            - Goal: 원하는 결과를 얻기 위한 정보들이 담겨 있음
            - Cancel: feedback을 지속적으로 받다가 client가 goal을 취소하고 싶은 경우 사용
        - **Server → Client**
            - Status: server가 작업중인 동안 client가 다른 일을 할 수 있도록 server의 상태를 알려줌
            - Result: 요청된 작업이 끝나면 client에게 요청한 결과를 전달
            - Feedback: 원하는 결과에 얼마나 달성했는지 지속적으로 client에게 보내줌

---

- **[Overall Process]**
    - 단순히 subscribe만 진행하여 panorama image로 stitching 해서 결과를 보여주던 코드를 기반하여 action으로 변경
        - **(Reference Code)**
            
            [GitHub - SungJaeShin/Stitching_Image](https://github.com/SungJaeShin/Stitching_Image.git)
            
    
    ---
    
    - 목표:
        - (1) Client에서 2개의 이미지를 server에 전송
        - (2) Server에서 2개의 이미지를 받아 panorama Image로 생성
        - (3) Panorama image가 생성이 완료될 때만 client에 전송
        - (4) Client에서 panorama image를 받아 publish 하여 rqt_image_view로 볼 수 있게 진행
    
    ---
    
    - **(1) Make Action file**
        - “action” 이라는 폴더를 생성하고 action 파일을 생성
            
            ```yaml
            # Define the goal
            int32 index
            sensor_msgs/Image img1  # Specify which dishwasher we want to use
            sensor_msgs/Image img2
            ---
            # Define the result
            int32 pano_index
            sensor_msgs/Image panoImg
            ---
            # Define a feedback message
            bool is_make_pano
            ```
            
    
    ---
    
    - **(2) Include CMakeList.txt & package.xml file**
        - custom으로 생성한 action 파일을 각각 추가시켜줌
            - CMakeList.txt
                
                ```
                find_package(catkin REQUIRED COMPONENTS
                  **actionlib
                  actionlib_msgs
                  genmsg**
                )
                
                ## Generate actions in the 'action' folder
                add_action_files(
                  FILES
                  **image.action**
                )
                
                ## Generate added messages and services with any dependencies listed here
                generate_messages(
                  DEPENDENCIES
                  **actionlib_msgs**
                )
                
                catkin_package(
                 INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
                #  LIBRARIES Action_tutorial
                 CATKIN_DEPENDS **actionlib** 
                #  DEPENDS system_lib
                )
                ```
                
            
            ---
            
            - package.xml
                
                ```xml
                <build_depend>actionlib</build_depend>
                <build_depend>actionlib_msgs</build_depend>
                
                <build_export_depend>actionlib</build_export_depend>
                <build_export_depend>actionlib_msgs</build_export_depend>
                
                <exec_depend>actionlib</exec_depend>
                <exec_depend>actionlib_msgs</exec_depend>
                ```
                
    
    ---
    
    - **(3) Make Client Node**
        - include header
            
            ```cpp
            // Action Client Related 
            #include <actionlib/client/simple_action_client.h>
            #include <actionlib/client/terminal_state.h>
            #include <Action_tutorial/imageAction.h> 
            ```
            
            - **[1]** action file 명이 image.action임에도 불구하고 custom action file을 가지고 오려면 다음과 같이 헤더에 **<${package_name}/${action_name}Action.h>** 이렇게 Action이 추가됨
        
        ---
        
        - main.cpp
            
            ```cpp
            // Publisher initialization
            ros::Publisher pub_pano = nh.advertise<sensor_msgs::Image>("PanoramaImage", 100);
            
            // Client initialization  
            actionlib::SimpleActionClient<Action_tutorial::imageAction> client("imgAction", true);
            
            // Will wait for infinite time
            client.waitForServer(); 
            
            // Goal, Result initialization
            Action_tutorial::imageGoal goal;
            Action_tutorial::imageResultConstPtr result;
            
            // Put in goal information
            goal.index = sequence;
            goal.img1 = img1;
            goal.img2 = img2;
            
            // To send Server 
            client.sendGoal(goal);
            
            // Wait for Server output
            bool finished_before_timeout = client.waitForResult(ros::Duration(5));
            
            if(finished_before_timeout)
            {
                result = client.getResult();
                ROS_WARN("Get Pano Img Index: %i", result -> pano_index);
                pub_pano.publish(result -> panoImg);
            }
            else
                ROS_INFO("Action did not finish before the time out.");
            ```
            
            - **[1]** SimpleActionClient는 **client를 정의할 때 사용**
            - **[2]** client("imgAction", true)에서 **“imgAction”은 서버 이름, true은 계속 server 내용 수신**
            - **[3]**  **waitForServer()는 server가 시작되기 전까지 계속 대기**
            - **[4]** Action_tutorial::imageGoal goal는 목표를 위한 정보를 담는 action 정의
                - 여기서도 image 이름인데 **목표를 위한 변수는 imageGoal로 정의**
            - **[5]** client.sendGoal(goal)는 **server에 전송**
            - **[6]** client.waitForResult(ros::Duration(5)) **server로부터 5초동안 result를 기다림**
            - **[7]** client.getResult()는 action에서 정의한 **result 값을 server로부터 가지고 옴**
                - 여기서는 image에서 sensor_msg::Image & int32 값을 server로부터 받을 수 있음
    
    ---
    
    - **(4) Make Server Node**
        - include header
            
            ```cpp
            // Action Server Related
            #include <actionlib/server/simple_action_server.h>
            #include <Action_tutorial/imageAction.h>
            ```
            
            - **[1]** action file 명이 image.action임에도 불구하고 custom action file을 가지고 오려면 다음과 같이 헤더에 **<${package_name}/${action_name}Action.h>** 이렇게 Action이 추가됨
        
        ---
        
        - main.cpp
            
            ```cpp
            // Server initialization 
            actionlib::SimpleActionServer<Action_tutorial::imageAction> server;
            
            // Feedback, Result initialization 
            Action_tutorial::imageFeedback feedback;
            Action_tutorial::imageResult result;
            
            bool success;
            
            // Consturctor including NodeHandle
            ImgServer(ros::NodeHandle &n)
            : nh(n)
            , server(nh, "imgAction", boost::bind(&ImgServer::execute, this, _1), false)
            {
                server.start();
            }
            
            // Get client message and do execute get final goal 
            void execute(const Action_tutorial::imageGoalConstPtr &goal)
            {
                success = false;
            		
            		// Get client information
                int cur_index = goal -> index;
                sensor_msgs::Image image1 = goal -> img1;
                sensor_msgs::Image image2 = goal -> img2;
            
                cv_img1 = sensorMsg2cvMat(image1);
                cv_img2 = sensorMsg2cvMat(image2);
            
                makePanoImg(cv_img1, cv_img2, cv_pano_img, cur_index);
                
                if(success)
                {
                    sensor_msgs::Image pano = cvMat2sensorMsg(cv_pano_img, image1.header);
                    
            				// Put result of final goal
            				result.pano_index = cur_index;
                    result.panoImg = pano;
                    ROS_WARN("Success Panorama Image !!");
            
            				// To send client using result 
                    server.setSucceeded(result);
                }
            }
            ```
            
            - **[1]** SimpleActionServer는 **server를 정의할 때 사용**
            - **[2]** Action_tutorial::imageFeedback feedback는 **feedback을 정의할 때 사용**
                - 여기서도 image 이름인데 **feedback를 위한 변수는 imageFeedback으로 정의**
            - **[3]** Action_tutorial::imageResult result는 목표에 대한 **결과 값을 정의할 때 사용**
                - 여기서도 image 이름인데 결과**를 위한 변수는 imageResult으로 정의**
            - **[4]** server(nh, "imgAction", boost::bind(&ImgServer::execute, this, _1), false)에서  **"imgAction"은 서버 이름, &ImgServer::execute goal을 받아서 수행할 함수 이름, false는 자동적으로 서버를 킴**
            - **[5]** server.start() **서버를 시작하는 flag**
            - **[6]** server.setSucceeded(result)는 **client에게 목표에 대한 결과 값을 전송**

---

- **[Others]**
    - Action client & server 의 경우, **“무조건 constructor에서 정의를 해두어야 함”**
        - Client의 경우에는 main.cpp에서 정의해도 무관
    
    ---
    
    - actionlib::SimpleClientGoalState query_state의 경우 **class 변수로 미리 생성해두면 constructor에서 추가로 정의 해두어야함**
        - (해당 error 내용) **error**: no matching function for call to ‘actionlib::SimpleClientGoalState::SimpleClientGoalState()’
    
    ---
    
    - ros::init 보다 먼저 ros action 관련 client 또는 server 노드를 먼저 실행시키면 안되고 **항상 먼저 ros::init 뒤에 정의를 해두어야함**
        - (해당 error 내용) **[Fatal]**: You must call ros::init() before creating the first NodeHandle
            - [Reference Site] [https://answers.ros.org/question/100656/problem-with-nodehandle-before-rosinit/](https://answers.ros.org/question/100656/problem-with-nodehandle-before-rosinit/)
                
                [Problem with nodehandle before ros::init - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/100656/problem-with-nodehandle-before-rosinit/)
---
title: "[OpenCV Mat Type] What is meaning INT value in cv::Mat type result?"
excerpt: "OpenCV Mat Type"
---
# [OpenCV Mat Type] What is meaning INT value in cv::Mat type result?

---

- **[Reference Site]**
    
    - [[OpenCV 2.4.3] Mat의 Type 종류와 그 enum 값](https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=iku88&logNo=130153285316)
    

---

- **[Cause]**
    - cv::Mat multiplication 을 할 때면 각 cv::Mat 의 type 들이 달라서 다음과 같은 terminal error 가 발생하는 경우가 생긴다.
        - **Terminal Error**
            
            ```bash
            OpenCV Error: Assertion failed (type == B.type() && 
            (type == CV_32FC1 || type == CV_64FC1 || type == CV_32FC2 || type == CV_64FC2)) in gemm, 
            file /home/sj/opencv-3.2.0/modules/core/src/matmul.cpp, line 1530
            ```
            

---

- **[Solution]**
    - (1) cv::Mat 에 해당하는 matrix 의 type 이 다를 수 있으므로 debugging 을 통해서 각 cv::Mat 의 type 을 출력
        - Using **cv::Mat::type()**
            
            ```bash
            cv::Mat image, TF;
            std::cout << "type of image: " << image.type() << std::endl;
            std::cout << "type of TF: " << TF.type() << std::endl;
            ```
            
    - (2) 결과가 다르다면 이들의 type 을 변경
        - Using **cv::Mat::convertTo(cv::Mat, cv::Type)**
            
            ```bash
            cv::Mat cvt_img; 
            cvt_img.convertTo(image, CV_64F);
            ```
            

---

- **[Append]**
    - **cv::Mat type 에 관한 enum 정보는 위의 reference site 에서 참고하여 변경할 수 있다.**

---

- **[IMPORTANT]**
    - **이미지의 type을 변경하는 것이 이미지의 channel을 변경하는 것과는 다르다.**
        - type 변경하는 것은 **convertTo()를 사용**하는데 **이는 channel 수는 같아야 한다!!!**
        - channel 변경하는 것은 **cvtColor()를 사용**해야 완벽하게 channel 수가 변경된다!!!
        - Reference Site:
            
            - [Mat 클래스](https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=windowsub0406&logNo=220515761001)
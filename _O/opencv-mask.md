---
title: "[OpenCV] How to make image masking using OpenCV in C++?"
excerpt: "OpenCV based Image Masking"
---
# [OpenCV] How to make image masking using OpenCV in C++?

## [Goal] cv::Rect을 이용하여 이미지 masking 통해 특정 이미지 영역을 추출한다.

---

- **[Reference Site]**
    
    - [https://junstar92.tistory.com/402](https://junstar92.tistory.com/402)
    
    - [[OpenCV] cv::Rect 유용한 연산](https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=windrevo&logNo=221721329805)
    
    - [[OpenCV] Image Crop](https://eehoeskrap.tistory.com/419)
    
    - [Intersection and Union of Two Rectangles using OpenCV](https://putuyuwono.wordpress.com/2015/06/26/intersection-and-union-two-rectangles-opencv/)
    
    - [https://diyver.tistory.com/95](https://diyver.tistory.com/95)
    
    - [Behaviour of ORB keypoint detection with masks - OpenCV Q&A Forum](https://answers.opencv.org/question/103087/behaviour-of-orb-keypoint-detection-with-masks/)
    

---

- **[cv::Rect]**
    - Initialization
        - (1) 두 좌표를 이용한 사각형 범위
            
            ```cpp
            cv::Rect(Point(x1, y1), Point(x2, y2));
            ```
            
        - (2) x, y 좌표에서부터 width 와 height 까지의 사각형 범위
            
            ```cpp
            cv::Rect(x, y, width, height);
            ```
            
    
    ---
    
    - Draw Rectangle
        
        ```cpp
        cv::rectangle(img, cv::Rect, cv::Scalar(b,g,r), thickness, lineType, shift);
        ```
        
        - **img**: 사각형을 그려넣을 이미지 입력
        - **scalar(b, g, r)** : 선의 색상을 b, g, r 순으로 입력
        - **thickness** : 선의 굵기를 설정할 수 있음
        - **lineType** : 선의 타입을 설정 가능,
            - FILLED : 안을 채워 넣음 , -1
            - LINE_4 : 4 connected line , 4
            - LINE_8 : 8 connected line , 8
            - LINE_AA : 안티앨리어싱 , 16
        - **IMPORTANT !!!!!**
            - lineType 부분에 filled 를 넣기 위해서는 다음과 같이 thickness에 넣어주어야 한다!
                
                ```cpp
                cv::rectangle(img, cv::Rect, cv::Scalar(b,g,r), cv::FILLED, 8, shift);
                ```
                
    
    ---
    
    - Intersection & Union
        - (1) Initialize two box
            
            ```cpp
            cv::Rect r1(x1, y1, width1, height1);
            cv::Rect r2(x2, y2, width2, height2);
            ```
            
        - (2) Intersection
            
            ```cpp
            cv::Rect r3 = r1 & r2;
            ```
            
        - (3) Union
            
            ```cpp
            cv::Rect r3 = r1 | r2;
            ```
            

---

- **[Example Code]**
    
    ```cpp
    #include <iostream>
    #include <fstream>
    
    // stitching related
    #include "opencv2/imgcodecs/imgcodecs.hpp"
    #include "opencv2/highgui/highgui.hpp"
    #include "opencv2/stitching.hpp"
    
    int main()
    {
    	// Road images
    	cv::Mat left = cv::imread("/home/sj/iros2022/image/left.jpg", 1);
    
      // Check image read correctly
    	if(left.cols != 0)
    		std::cout << "Correctly read left image !!" << std::endl;
    
    	// Combine three images in one vector 
    	std::vector<cv::Mat> imgs;
    	imgs.push_back(left);
    
    	// Add Masking
    	cv::Rect left_rect(int(left.cols/2), 0, int(left.cols/2) , left.rows);
    	cv::Rect right_rect(0, 0, int(right.cols/2), right.rows);
    
    	std::cout << "Left image size1 : " << left.size() << std::endl;
    	// Left image size1 : [4032 x 1908]	
    	std::cout << "Left image size2 : " << left.rows << ", " << left.cols << std::endl;
     	// Left image size2 : 1908, 4032
    	Std::cout << "Left image left_top points : " <<  int(left.rows * 3 / 4) << ", " << int(left.cols / 2) << std::endl;
    	// Left center Points : 1431, 2016
    	std::cout << "Left Mask : " << left_rect << std::endl;
    	// Left Mask : [954 x 4032 from (1431, 2016)]
    	std::cout << "Left Rect area : " << left_rect.area() << std::endl;
    	// Left Rect area : 3846528	
    	
    	cv::Mat copy_left = left.clone(); 
    	cv::Mat left_mask_img = copy_left(left_rect);
    	cv::imshow("Left Rect", left_mask_img);
      cv::waitKey(100);
    
    	cv::Mat copy_right = left.clone(); 
    	cv::Mat right_mask_img = copy_right(right_rect);
    	cv::imshow("Right Rect", right_mask_img);
      cv::waitKey(100);
    
    	return 0;
    }
    ```
    

---

- **[Compile]**
    - C++을 사용하였고 3개의 라이브러리를 포함하고 있으므로 stitching 이라는 이름을 가진 exeutive file을 만들어줌
        
        ```bash
        g++ stitching.cpp -L/usr/local/include/opencv2 -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_stitching -o stitching
        ```
        
        - g++: c++ & gcc: c
        - stitching.cpp: 현재 저장되어 있는 코드의 이름
        - -L: 포함된 라이브러리가 있는 경로
        - -lopencv_~: 코드에 포함된 라이브러리들
        - -o: 실행파일에 대한 정보를 입력할 flag
        - stitching: 실행파일의 이름

---

- **[Run]**
    - 해당 파일에 다음과 같이 작성해주면 컴파일한 코드를 빌드 및 실행
        
        ```bash
        # Example of current path: /home/sj/iros2022/src/Stitching
        ./stitching
        ```
        

---

- **[Result]**
    - Original Image
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_mask/Untitled.jpeg" alt="">
        </figure>
        
    
    ---
    
    - Left Masking
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_mask/Untitled.png" alt="">
        </figure>
        
    
    ---
    
    - Right Masking
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_mask/Untitled 1.png" alt="">
        </figure>
        

---

- **[Error]**
    - **[OpenCV Error]** Assertion failed (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= m.rows) in cv::Mat::Mat
        - **[Answer]** Masking 하는 region 이 기존의 이미지에서 벗어났기 때문에 발생한다. Masking 하는 box를 정의할 때, cv::Rect R(x, y, width, height)로 정의한다. 여기서 **(x, y)는 box의 center point 가 아니라 left & top point 시작점을 의미**한다. 그리고 (width, height)의 경우에는 box의 실제 width와 height 길이이다.
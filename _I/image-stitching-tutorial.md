---
title: "[Image Stitching] How to stitch images using opencv library in c++"
excerpt: "Image Stitching: Tutorial"
---
# [Image Stitching] How to stitch images using opencv library in c++

- **[Github Result]**
    
    - [GitHub - SungJaeShin/Image_Stitching](https://github.com/SungJaeShin/Image_Stitching.git)
    

---

- **[Reference Site]**
    - Stitching related:
        - [https://www.geeksforgeeks.org/stitching-input-images-panorama-using-opencv-c/](https://www.geeksforgeeks.org/stitching-input-images-panorama-using-opencv-c/)
            
        - [https://www.pyimagesearch.com/2018/12/17/image-stitching-with-opencv-and-python/](https://www.pyimagesearch.com/2018/12/17/image-stitching-with-opencv-and-python/)
                
    ---
    
    - Compile C++ related:
        - [https://www.cyberciti.biz/faq/howto-compile-and-run-c-cplusplus-code-in-linux/](https://www.cyberciti.biz/faq/howto-compile-and-run-c-cplusplus-code-in-linux/)
            
        - [https://stackoverflow.com/questions/48687259/undefined-reference-to-cvimreadcvstring-const-int](https://stackoverflow.com/questions/48687259/undefined-reference-to-cvimreadcvstring-const-int)
          
    ---
    
    - Opencv related:
        - [https://diyver.tistory.com/51](https://diyver.tistory.com/51)
            
        - [https://stackoverflow.com/questions/34497099/opencv-undefined-reference-to-imread](https://stackoverflow.com/questions/34497099/opencv-undefined-reference-to-imread)
            
        - [https://answers.opencv.org/question/221603/undefined-reference-to-imread/](https://answers.opencv.org/question/221603/undefined-reference-to-imread/)
                        
        - [https://learnopencv.com/image-resizing-with-opencv/](https://learnopencv.com/image-resizing-with-opencv/)
                
    ---
    
    - Time duration related:
        - [https://www.geeksforgeeks.org/measure-execution-time-function-cpp/](https://www.geeksforgeeks.org/measure-execution-time-function-cpp/)

---

- **[Code]**
    - 다음과 같은 opencv library들을 포함: **imgcodecs, highgui, stitching**
        
        ```cpp
        #include <iostream>
        #include <fstream>
        
        // Time duration related
        #include <chrono>
        
        // stitching related
        #include "opencv2/imgcodecs/imgcodecs.hpp"
        #include "opencv2/highgui/highgui.hpp"
        #include "opencv2/stitching.hpp"
        
        // Original Image Size: width * height = 4032 * 1908
        bool RESIZE = false;
        
        int main()
        {
        	// To check time
        	auto start = std::chrono::high_resolution_clock::now();
        
        	// Define mode for stitching as panorama
        	cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;
        
        	// Road images
        	cv::Mat left = cv::imread("/home/sj/iros2022/image/left.jpg", 1);
        	cv::Mat front = cv::imread("/home/sj/iros2022/image/front.jpg", 1);
        	cv::Mat right = cv::imread("/home/sj/iros2022/image/right.jpg", 1);
        
          // Check image read correctly
        	if(left.cols != 0)
        		std::cout << "Correctly read left image !!" << std::endl;
        	if(front.cols != 0)
        		std::cout << "Correctly read front image !!" << std::endl;
        	if(right.cols != 0)
        		std::cout << "Correctly read right image !!" << std::endl;
        
        	// Resize image 
        	double height = left.cols / 4;
        	double width = left.rows / 4;
        
        	// Combine three images in one vector 
        	std::vector<cv::Mat> imgs;
        	imgs.push_back(left);
        	imgs.push_back(front);
        	imgs.push_back(right);
        
        	// Initialization output image
        	cv::Mat panorama;
        
        	// Make panorama image using cv::Stitcher 
        	cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode, false);
          cv::Stitcher::Status status = stitcher->stitch(imgs, panorama);
        	
        	// If we want to change image size, then please change variable "RESIZE" **false** to **true** !!
        	if(RESIZE)
        		cv::resize(panorama, panorama, cv::Size(height, width), CV_INTER_LINEAR);
        
        	// Check time duration
        	auto finish = std::chrono::high_resolution_clock::now();
        	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
        	std::cout << "Time duration: " << (double)duration.count() / 1000000 << " sec" << std::endl;
        
        	// Write and show result image 
          cv::imwrite("/home/sj/iros2022/image/result.jpg", panorama);
        	cv::imshow("Result", panorama);
          
        	// Wait image not auto cancel
          cv::waitKey(0);
        
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

- **[Output]**
    - 결과는 다음과 같음
        - Input images:
            - Left
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_tutorial/Untitled.jpeg" alt="">
                </figure> 
                
            - Front
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_tutorial/Untitled 1.jpeg" alt="">
                </figure> 
                
            - Right
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_tutorial/Untitled 2.jpeg" alt="">
                </figure>                 
        
        ---
        
        - Output image:
            - RESIZE == true (원래 이미지 크기의 1/4배)
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_tutorial/Untitled 3.jpeg" alt="">
                </figure> 
                
            - RESIZE == False
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_tutorial/Untitled 4.jpeg" alt="">
                </figure> 
                
---

- **[Analyze]**
    - 확실히 resize를 하게 되면 linear interpolation을 하기 때문에 영상의 resolution이 좋아지지 않음
    - 되도록이면 영상의 크기를 보존하는 것이 좋음
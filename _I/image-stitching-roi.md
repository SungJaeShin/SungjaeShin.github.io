---
title: "[Image Stitching] How to stitch images using RoI masking with OpenCV in c++?"
excerpt: "Image Stitching: RoI Masking"
---
# [Image Stitching] How to stitch images using RoI masking with OpenCV in c++?

- **[Github Result]**
    
    - [GitHub - SungJaeShin/Image_Stitching](https://github.com/SungJaeShin/Image_Stitching.git)
    

---

- **[Reference Site]**
    - OpenCV Description:
        - [https://github.com/opencv/opencv/blob/3.2.0/modules/stitching/src/stitcher.cpp](https://github.com/opencv/opencv/blob/3.2.0/modules/stitching/src/stitcher.cpp)
            
        - [https://vovkos.github.io/doxyrest-showcase/opencv/sphinx_rtd_theme/class_cv_Stitcher.html?highlight=stitcher#doxid-d7-d79-classcv-1-1-stitcher-1a34152e67e9b04306236b6d570e9adf30](https://vovkos.github.io/doxyrest-showcase/opencv/sphinx_rtd_theme/class_cv_Stitcher.html?highlight=stitcher#doxid-d7-d79-classcv-1-1-stitcher-1a34152e67e9b04306236b6d570e9adf30)
                       
    ---
    
    - Image Stitching:
        - [https://study.marearts.com/2013/11/opencv-stitching-example-stitcher-class.html](https://study.marearts.com/2013/11/opencv-stitching-example-stitcher-class.html)
              
    ---
    
    - Image Stitching with Masking:
        - [https://github.com/MareArts/Still-Image-Stitching-Test-Using-OpenCV/blob/master/Stitching_CV/main.cpp#L63](https://github.com/MareArts/Still-Image-Stitching-Test-Using-OpenCV/blob/master/Stitching_CV/main.cpp#L63)

---

- **[OpecCV Description]**
    - 간단하게 설명하면 stitching을 위해서 기초적으로 feature extraction과 feature matching을 통해서 이미지간의 homography를 추정하게 되는데 이때, 추출되는 이미지 feature가 이미지 간의 overlap 되지 않은 부분에 focus 되면 stitching의 성능을 저하시킬 수 있기 때문에 masking(RoI)을 적용하여 overlap 되는 부분에 focus를 시킴으로서 성능을 높혀주기 위함이다.
    - OpenCV 3.2.0 기준으로 masking(RoI) setting 하는 부분까지 설명된 코드는 다음과 같다.
        - Stitch Function Arguments:
            - InputArrayOfArrays images = std::vector<cv::Mat> images
            - **std::vector<std::vector<cv::Rect>> &rois**
                - 여기서 std::vector 안에 std::vector가 붙은 이유는 하나의 이미지 안에도 여러 가지 interesting region들이 존재할 수 있기 때문이다.
                - 그래서 각 이미지 마다 std::vector<cv::Rect>로 관심 있는 영역을 Setting 해주고 stitching 하는 총 이미지 개수만큼 rois 에 차례대로 push_back 해주면 된다.
            - OutputArray pano = cv::Mat
                
                ```cpp
                Stitcher::Status Stitcher::stitch(InputArrayOfArrays images, const std::vector<std::vector<Rect> > &rois, OutputArray pano)
                {
                    Status status = estimateTransform(images, rois);
                    if (status != OK)
                        return status;
                    return composePanorama(pano);
                }
                ```
                

---

- **[Code]**
    - 다음과 같은 opencv library들을 포함: **imgcodecs, highgui, stitching**
        
        ```cpp
        #include <iostream>
        #include <fstream>
        
        // stitching related
        #include "opencv2/imgcodecs/imgcodecs.hpp"
        #include "opencv2/highgui/highgui.hpp"
        #include "opencv2/stitching.hpp"
        
        int main()
        {
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
        
        	// Combine three images in one vector 
        	std::vector<cv::Mat> imgs;
        	imgs.push_back(left);
        	imgs.push_back(front);
        	imgs.push_back(right);
        
        	// Initialization output image
        	cv::Mat panorama;
        
        	// Add Masking
          std::vector<std::vector<cv::Rect>> masks;
        	std::vector<cv::Rect> mask1;
        	std::vector<cv::Rect> mask2;
        	std::vector<cv::Rect> mask3;
        
        	// Define cv::Rect what part we focus on !!	
        	cv::Rect left_rect(int(left.cols/2), 0, int(left.cols/2) , left.rows);
        	cv::Rect front_rect(0, 0, front.cols, front.rows);
        	cv::Rect right_rect(0, 0, int(right.cols/2), right.rows);
        
        	// Set the desired RoI for each image
        	mask1.push_back(left_rect);
        	mask2.push_back(front_rect);
        	mask3.push_back(right_rect);
        
        	masks.push_back(mask1);
        	masks.push_back(mask2);
        	masks.push_back(mask3);
        
        	// Make panorama image using cv::Stitcher adding masks
        	cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode, false);
          cv::Stitcher::Status status = stitcher->stitch(imgs, masks, panorama);
        
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
        - Masking Setting:
            - Left Masking
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_roi/Untitled.png" alt="">
                </figure> 
                
            - Front Masking
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_roi/Untitled 1.png" alt="">
                </figure> 
                
            - Right Masking
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_roi/Untitled 2.png" alt="">
                </figure>                 
        
        ---
        
        - Output image:
            - 각 3개의 이미지들에 masking 을 적용한 것을 visualization
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_roi/Untitled 3.png" alt="">
                </figure>    
                
            - 기존에 실험했던 이미지를 가지고 masking 적용 전후 비교
                - Masking 적용 전
                    <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_roi/Untitled.jpeg" alt="">
                    </figure> 
                    
                - Masking 적용 후
                    <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_roi/Untitled 1.jpeg" alt="">
                    </figure> 
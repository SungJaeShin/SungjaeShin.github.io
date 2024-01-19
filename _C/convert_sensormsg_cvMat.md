---
title: "[Convert] sensor_msgs ↔ cv::Mat"
excerpt: "Convert sensor_msgs to cv::Mat"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Convert] sensor_msgs ↔ cv::Mat

## [Goal] sensor_msgs::Image 를 cv::Mat & cv::Mat 을 sensor_msgs::Image 로 변환

- **[Overall Reference Site]**
    
    - [cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages](http://library.isr.ist.utl.pt/docs/roswiki/cv_bridge(2f)Tutorials(2f)UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.html)
    

---

- **[Convert sensor_msgs::Image to cv::Mat]**
    - Reference Site: [https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/rosNodeTest.cpp#L48](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/vins_estimator/src/rosNodeTest.cpp#L48)
        
    
    ---
    
    - Code example
        
        ```cpp
        cv::Mat sensorMsg2cvMat(const sensor_msgs::ImageConstPtr &img_msg)
        {
            cv_bridge::CvImageConstPtr ptr;
            if (img_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = img_msg->header;
                img.height = img_msg->height;
                img.width = img_msg->width;
                img.is_bigendian = img_msg->is_bigendian;
                img.step = img_msg->step;
                img.data = img_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        
            cv::Mat img = ptr->image.clone();
        		cv::cvtColor(img,img,cv::COLOR_BGRA2BGR);
        
            return img;
        }
        ```
        

---

- **[Convert cv::Mat to sensor_msgs::Image]**
    - Reference Site: [https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros](https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros)
        
    
    ---
    
    - Code example
        
        ```cpp
        sensor_msgs::Image cvMat2sensorMsg(cv::Mat image, std_msgs::Header header)
        {
        	cv_bridge::CvImage img_bridge;
        	sensor_msgs::Image img;
        
        	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
        	img_bridge.toImageMsg(img);
        
        	return img;
        }
        ```
        

---

- **[Others]**
    - How to return an empty cv::Mat in a function for c++?
        - Reference Site: [https://stackoverflow.com/questions/59113391/how-to-return-an-empty-mat-in-a-function-for-c](https://stackoverflow.com/questions/59113391/how-to-return-an-empty-mat-in-a-function-for-c)
            
        - To change return following as:
            
            ```cpp
            cv::Mat sensorMsg2cvMat(const sensor_msgs::Image &img_msg)
            {
            	// ***** fill in your code ***** //
            	cv_bridge::CvImageConstPtr ptr;
            	ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
            	cv::Mat img = ptr->image.clone();
            
            	// Case 1. return empty image 
            	return cv::Mat{};
            
            	// Case 2. return black image same size as img_msg
            	return cv::Mat{img.size(), img.type(), cv::Scalar{0,0,0}};
            }
            ```
            

---

- **[Error]**
    - **OpenCV Error: Assertion failed (_image.type() == CV_8UC3) in apply**
        - Reference Site: [https://stackoverflow.com/questions/37764838/bilateral-filter-error-assertion-failed-src-type-cv-8uc1-src-type](https://stackoverflow.com/questions/37764838/bilateral-filter-error-assertion-failed-src-type-cv-8uc1-src-type)
     
        - Code change following as (이미지 filter 문제):
            
            ```cpp
            cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
            ```
            
    
    ---
    
    - **OpenCV Error: Assertion failed (scn == 3 || scn == 4) in cvtColor**
        - Reference Site: [http://daplus.net/python-열린-cv-오류-215-scn-3-cvtcolor-함수의-scn-4/](http://daplus.net/python-%EC%97%B4%EB%A6%B0-cv-%EC%98%A4%EB%A5%98-215-scn-3-cvtcolor-%ED%95%A8%EC%88%98%EC%9D%98-scn-4/)
            
        - Reference Site: [https://snowdeer.github.io/ros2/2018/01/04/ros2-opencv-camera-image-subscriber/](https://snowdeer.github.io/ros2/2018/01/04/ros2-opencv-camera-image-subscriber/)
            
        - Code change following as (이미지 channel 문제):
            
            ```cpp
            // Error Code Line
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            
            // Modfied following as:
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8);
            ```
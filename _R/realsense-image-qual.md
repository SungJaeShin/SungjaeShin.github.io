---
title: "[Realsense Image Quality] We can also change image quality using rqt_reconfigure"
excerpt: "RealSense Image Quality"
---
# [Realsense Image Quality] We can also change image quality using rqt_reconfigure

---

- **[Reference Site]**
    
    - [Wiki](http://wiki.ros.org/rqt_reconfigure)
    
    - [[ROS 튜토리얼] 1.1.6 ROS의 토픽(topic)에 대한 이해](https://conceptbug.tistory.com/entry/ROS-%ED%8A%9C%ED%86%A0%EB%A6%AC%EC%96%BC-116-ROS%EC%9D%98-%ED%86%A0%ED%94%BDtopic%EC%97%90-%EB%8C%80%ED%95%9C-%EC%9D%B4%ED%95%B4)
    

---

- **[Cause]**
    - Image raw size 가 매우 크므로 이는 delay 가 영향을 주기 때문에 보통 Compressed Image 로 변경하여 데이터를 사용한다. 그런데 여러 로봇들이나 다양한 compressed image 를 사용할 경우 이 역시 Hz 저하나 delay 의 증가를 가지고 온다.
    - 따라서, 추가적으로 image 의 quality 를 추가적으로 조절함으로서 compressed image 의 delay 나 Hz 를 조절할 수 있다.

---

- **[rqt_reconfigure]**
    - Terminal 상에서 rosparam 으로 직접 설정할 수 있지만 **rqt_reconfigure** 을 이용하여 ros parameter 를 쉽게 조절하여 image quality 를 바꿀수 있다.
        - rqt_reconfigure 를 사용하는 방법은 다음과 같다.
        
        ```bash
        # Terminal 
        $ rosrun rqt_reconfigure rqt_reconfigure
        ```
        
        ---
        
        - rqt_reconfigure 를 킨 결과는 다음과 같다.
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/realsense_qual/Untitled.png" alt="">
            </figure>
            

---

- **[Basic Concept]**
    - jpeg_quality: jpeg quality percentile
    - png_level: png compression level
    - rostopic delay: display delay of topic from timestamp in header
    - rostopic bw: display bandwidth used by topic
        - **bandwidth 가 필요하고 중요한 이유를 추가적으로 찾아보자!**

---

- **[Result]**
    - **[Table] jpeg_quality**
        
        
        | jpeg_quality | 1 | 10 | 20 | 30 | 40 | 80 (default) |
        | --- | --- | --- | --- | --- | --- | --- |
        | hz (1/s) | 30 | 30 | 30 | 30 | 30 | 30 |
        | delay (sec) | 0.045 | 0.050 | 0.040 | 0.050 | 0.035 | 0.045 |
        | bw (bps) | 1.40K | 1.30K | 1.20K | 1.10K | 1.00K | 4.00K |
    
    ---
    
    - **[Plot] jpeg_quality**
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/realsense_qual/Untitled 1.png" alt="">
        </figure>
        
    
    ---
    
    - **[Table] png_level**
        
        
        | png_level | 1 | 2 | 3 | 4 | 5 |
        | --- | --- | --- | --- | --- | --- |
        | hz (1/s) | 12.5 | 12 | 9.5 | 10.5 | 7.5 |
        | delay (sec) | 0.820 | 0.850 | 1.04 | 0.97 | 1.320 |
        | bw (bps) | 660 | 630 | 610 | 580 | 560 |
        
        | png_level | 6 | 7 | 8 | 9 (default) |
        | --- | --- | --- | --- | --- |
        | hz (1/s) | 4.5 | 3.3 | 1.70 | 1.40 |
        | delay (sec) | 2.20 | 3.00 | 5.70 | 7.05 |
        | bw (bps) | 545 | 530 | 420 | 385 |
    
    ---
    
    - **[Plot] png_level**
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/realsense_qual/Untitled 2.png" alt="">
        </figure>
        

---

- **[Analysis]**
    - Default setting 은 jpeg_quality: 80 & png_level: 9 로 고정이고 jpeg 가 기본 설정이다.
    - png 가 jpeg 보다 좋은 quality 의 이미지이므로 png 로 변경하여 realsense 를 키게 되면 속도가 바로 늦어져 delay 가 바로 생기게 된다. compression level 의 값이 높아질수록 delay 가 더 커지게 된다.
    - 반면에, jpeg_quality 의 경우 이미지의 quality 에 대한 값으로서 기본적으로는 80 이다.
    - 정확하게 분석하려면 추가적인 실험이 필요하겠지만, compressed image 의 경우 quality 변화에 따른 delay 의 변화는 없고 실제로 이미지가 흐려지고 질감이 뭉그러진다. 마치 smoothing filter 를 적용한듯하게 평활하게 만들어버리는 효과처럼 보이게 된다.
---
title: "[Histogram Equalization] Improvement of stitching results !!"
excerpt: "Histogram Equalization one of Image Filter"
---
# [Histogram Equalization] Improvement of stitching results !!

## [Goal] 어두운 환경이나 엄청 밝은 환경의 이미지를 취득시 추출되기 어려운 feature를 찾아보기

---

- **[Reference Site]**
    - [https://gaussian37.github.io/vision-concept-histogram_equalization/](https://gaussian37.github.io/vision-concept-histogram_equalization/)
                
    - [https://github.com/gaussian37/Vision/blob/master/OpenCV/histogram/histogramEqualizationColor.py](https://github.com/gaussian37/Vision/blob/master/OpenCV/histogram/histogramEqualizationColor.py)
                

---

- **[Histogram Equalization]**
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/histogram_equal/Untitled.png" alt="">
    </figure> 
    
    - Histogram equalization은 영상의 pixel value 들의 누적분포함수를 이용하여 영상을 개선하는 방법이고 image enhancement 방법 중 하나이다.
    - 즉, 화소가 좁은 low contrast 입력 영상을 이용하여 **화소값의 범위가 넓은 high contrast 출력 영상**을 얻는다.
    - 더 쉽게 말하자면, 어두운 이미지는 밝은 이미지로. 너무 밝은 이미지는 적당히 밝은 이미지로 변환해준다.

---

- **[Problem Definition]**
    - **너무 어두운 환경에서 취득된 영상이나 너무 밝은 환경에서 취득된 영상들은** 주변 pixel 들의 값들이나 pixel 변화량 (gradient) 의 차이가 매우 적기 때문에 **쉽게 feature를 추출하기 어렵게 된다.**
    - 그래서 이들을 histogram equalization을 통해서 보다 feature들을 잘 뽑을 수 있도록 pre-processing을 진행해주고 stitching을 진행하면 훨씬 더 많은 matching 결과들을 확인할 수 있다.

---

- **[Code Example]**
    - (1) gray scale 영상은 입력 영상에 바로 histogram equalization을 적용할 수 있다.
    - (2) color 영상은 HSV, YCrCb 등의 컬러 모델로 변환한 다음, 밝기값 채널에 histogram equalization을 적용하고 다시 BGR로 변환한다.
    
    ```cpp
    // Use GrayScale image for applying histogram equalization !
    
    // Convert BGR to GRAY
    cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
    
    // Use Histogram Equalization 
    cv::equalizeHist(img, img);
    
    // Restore GRAY to BGR
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    ```
    

---

- **[Result]**
    - Original Image
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/histogram_equal/Untitled 1.png" alt="">
        </figure> 
        
    
    ---
    
    - Apply Histogram Equalization
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/histogram_equal/Untitled 2.png" alt="">
        </figure> 
        
    
    ---
    
    - Compare the number of stitching image !! (**약 6배 상승**)
        
        
        | Using Histogram Equalization | # of image stiching |
        | --- | --- |
        | O | 340 |
        | X | 56 |

---

- **[KAIST-DP Dataset Result]**
    - Left: Not apply histogram equalization / Right: apply histogram equalization

    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/histogram_equal/Library_Hard_Not_Hist_Equal.gif" alt="">
    </figure> 

    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/histogram_equal/Untitled.gif" alt="">
    </figure> 

---

- **[Other Method]**
    - ***CLAHE equalization*** (= Contrast Limited Adaptive Histogram Equalization)
        - Reference Site:
            
            [[25편] CLAHE](https://m.blog.naver.com/samsjang/220543360864)
            
            [Adaptive Histogram Equalization 이란 무엇인가?](https://3months.tistory.com/407)
            
        - 정리해서 간단하게 표현한다면, 위에 언급된 **일반적인 histogram equalization 의 경우 이미지 전체를 대상으로 histogram 을 생성**하여 고르게 분포하도록 smoothing 을 진행하는데, **CHALE equalization 의 경우, 사용자가 설정해준 patch size 를 기준으로 patch 마다 histogram 을 생성하여 smoothing 을 진행**
            - 단일 equalization 보다 **noise 를 더 감소시킬 수 있고 contrast 가 더 크지는 않고 자연스럽게 나오는 것**을 확인할 수 있음
            - feature 를 추출하는 방식인 FAST 가 contrast 에 민감하기 때문에 이를 적용해서 더 잘 추출할 수 있도록 할 수 있음
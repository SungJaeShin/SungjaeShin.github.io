---
title: "[OpenCV Error] Assertion failed ((globalDescIdx>=0) && (globalDescIdx < size())) in getLocalIdx"
excerpt: "OpenCV Error 1"
---
# [OpenCV Error] Assertion failed ((globalDescIdx>=0) && (globalDescIdx < size())) in getLocalIdx

- **[Error Sign in Terminal]**
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_error1/Untitled.png" alt="">
    </figure>
    

---

- **[Reference Site]**
    - OpenCV Description:
        
        - [OpenCV: Images stitching](https://docs.opencv.org/3.2.0/d1/d46/group__stitching.html)
        
        - [OpenCV: cv::detail::SurfFeaturesFinder Class Reference](https://docs.opencv.org/3.2.0/df/d27/classcv_1_1detail_1_1SurfFeaturesFinder.html)
        
        - [OpenCV: cv::DescriptorMatcher Class Reference](https://docs.opencv.org/3.2.0/db/d39/classcv_1_1DescriptorMatcher.html#details)
        
        - [opencv/stitcher.cpp at 3.2.0 · opencv/opencv](https://github.com/opencv/opencv/blob/3.2.0/modules/stitching/src/stitcher.cpp)
        
        - [opencv/matchers.cpp at 3.2.0 · opencv/opencv](https://github.com/opencv/opencv/blob/3.2.0/modules/features2d/src/matchers.cpp)
        
    
    ---
    
    - Feature Matcher:
        
        - [OpenCV - 28. 특징 매칭(Feature Matching)](https://bkshin.tistory.com/entry/OpenCV-28-%ED%8A%B9%EC%A7%95-%EB%A7%A4%EC%B9%ADFeature-Matching)
        
        - [이미지 프로세싱 & 컴퓨터 시각화 25부 - Feature Matching(2화)](https://m.blog.naver.com/zeus05100/221769874668)
        
        - [[파이썬 OpenCV] 두 영상의 특징점 매칭과 매칭 결과 그리기 - cv2.BFMatcher_create, cv2.DescriptorMatcher, cv2.drawMatches](https://deep-learning-study.tistory.com/260)
        
    
    ---
    
    - Error Related:
        
        - [Opencv Flannbasedmatcher](https://stackoverflow.com/questions/25089393/opencv-flannbasedmatcher)
        
        - [throwing exception in matching function - OpenCV Q&A Forum](https://answers.opencv.org/question/26327/throwing-exception-in-matching-function/)
        
        - [lab/keypoint.py at master · zhudingsuifeng/lab](https://github.com/zhudingsuifeng/lab/blob/master/keypoint.py#L54)
        
        - [OpenCV error in flann knnmatch](https://stackoverflow.com/questions/41689781/opencv-error-in-flann-knnmatch)
        

---

- **[Cause]**
    - Brute Force 방식의 matcher이든, FLANN 방식의 matcher 이던지, OpenCV에서 제공하는 match 방식 중에서 knnMatch 방식이 존재한다. 이를 사용하였을 때 발생하는 문제가 되는데, knnMatch는 어떤 matching 방식인지 먼저 살펴본다.
    - knnMatch는 query descriptor와 target descriptor 간의 근접한 point를 추출한다. 만약에 사전에 k라는 개수를 정해주면, query descriptor 한 point당 최근접한 k개의 point들을 target descriptor 에서 찾아 추출하게 된다.
    - 여기서, 만약에 **query descriptor나 target descriptor의 개수가 knnMatch에서 설정한 k개의 최근접 이웃 개수보다 작다면** 최적 매칭을 하지 못하여 위와 같은 에러가 발생하게 된다. 다시 말하면, 하나의 query descriptor에 k개의 최근접 descriptor를 찾아야하는데 개수가 적어 k개보다 작게 나오는 경우가 존재한다.
        - 예를들면 다음과 같다. knnMatch(query_descriptor, target_descriptor, k=2)
            - 최근접 개수 k: 2
            - query keypoint 개수: 108개
            - target keypoint 개수: 1개
            - 이 경우, query에서 하나의 keypoint에 해당하는 descriptor와 최근접 point 2개를 target image에서 추출해야하는데 keypoint가 1개밖에 없어 이를 수행할 수 없게 된다. 따라서 다음과 같은 에러가 발생하게 된 것이다.

---

- **[Solution]**
    - 추출된 feature와 descriptor를 기반하여 matching을 진행하기 전에 사전에 keypoint 개수를 먼저 확인하여 keypoint가 최근접 개수 k보다 작게 나오게 된다면 feature matching 과정을 건너뛰도록 설정한다.

---

- **[Origin of the error]**
    - OpenCV 기반으로 추출한 stitching 과정은 high level API를 제공하지만 상세하게 구성되고 있는 code를 살펴보면 FLANN 기반의 knnMatch를 사용하고 있다. 여기서 default로 적용하고 있는 최근접 개수 k는 2개 (k=2)로 되어 있다.
    - 그 뿐만 아니라, xfeatures2d를 사용하는 경우에 stitcher API는 SURF extractor & descriptor 를 적용하고 있고 이때의 hessian threshold는 300을 default로 적용하고 있다. 이를 사용하는 경우에 추출된 image의 feature와 descriptor 개수를 살펴봐야한다.
    - 이들을 기반으로 이미지가 지속적으로 들어올 때, keypoints(features) 개수가 1개가 나오는 것을 확인할 수 있었고 이 결과가 위의 에러를 발생하게 되었다.

---

- **[Problem Solving Process]**
    - (0) **[Assume]** 현재 사용하고 있는 OpenCV version은 3.2.0이고 xfeatures2d를 사용하고 있다.
    
    ---
    
    - (1) 먼저, 에러가 발생한 곳을 디버깅하였다. 해당 문제가 발생한 function은 stitch함수로서 이는 다음과 같다. *(In stitcher.cpp)*
        - **cv::Stitcher::stitch(InputArrayOfArrays images, const std::vector<std::vectorcv::Rect> &rois, OutputArray pano)**
            
            ```cpp
            Stitcher::Status Stitcher::stitch(InputArrayOfArrays images, const std::vector<std::vector<Rect> > &rois, OutputArray pano)
            {
                Status status = estimateTransform(images, rois);
                if (status != OK)
                    return status;
                return composePanorama(pano);
            }
            ```
            
            - **cv::Stitcher::estimateTransform(InputArrayOfArrays images, const std::vector<std::vector<cv::Rect>> &rois)**
                - 이 함수는 이미지들을 이용하여 각 카메라 간의 rotation을 estimation하는 함수이다.
            - **cv::Stitcher::composePanorama(OutputArray pano)**
                - estimateTrasform을 통해서 estiamte된 rotation을 이용하여 주어진 이미지들을 stitching하여 최종 panorama 이미지를 만드는 함수이다.
    
    ---
    
    - (2) stitch 함수에서는 descriptor 관련되어 있는 코드가 나타나지 않아 globalDescIdx를 사용하고 있는 argument를 찾아보았다. 이는 **cv::DescriptorMatcher::DescriptorCollection**에 있음을 확인하였다. *(In feature2d.hpp)*
        - globalDescIdx라는 argument를 사용하고 있는 function들은 다음과 같다.
            
            ```cpp
            void DescriptorMatcher::DescriptorCollection::getLocalIdx( int globalDescIdx, int& imgIdx, int& localDescIdx ) const
            {
                CV_Assert( (globalDescIdx>=0) && (globalDescIdx < size()) );
                std::vector<int>::const_iterator img_it = std::upper_bound(startIdxs.begin(), startIdxs.end(), globalDescIdx);
                --img_it;
                imgIdx = (int)(img_it - startIdxs.begin());
                localDescIdx = globalDescIdx - (*img_it);
            }
            ```
            
            - 여기서 CV_Assert라고 해당 에러와 동일한 문구가 적혀있는 것을 확인할 수 있었다.
            
            ```cpp
            const Mat DescriptorMatcher::DescriptorCollection::getDescriptor( int globalDescIdx ) const
            {
                CV_Assert( globalDescIdx < size() );
                return mergedDescriptors.row( globalDescIdx );
            }
            ```
            
            - 이 에러는 발생했던 에러와는 일부분만 겹치므로 이는 아닌 것을 확인할 수 있었다.
    
    ---
    
    - (3) 그런데, cv::Stitcher와 cv::DescriptorMatcher간의 연관성이 있어야 해당 에러가 발생할 것이라고 생각하여 cv::Sticher가 사용하고 있는 feature extraction 및 feature matching 함수들을 살펴보기로 하였다. *(In matchers.hpp)*
        - Feature Extraction 관련 class: **cv::detail::FeaturesFinder**
            - 이 class를 구성하는 하위 class들은 다음과 같다.
                - [1] **cv::detail::AKAZEFeatureFinder**
                - [2] **cv::detail::OrbFeatureFinder**
                - [3] **cv::detail::SurfFeatureFinder**
        - Feature Matching 관련 class: **cv::detail::FeaturesMatcher**
            - 이 class를 구성하는 하위 class들은 다음과 같다.
                - [1] **cv::detail::BestOf2NearestMatcher**
                    - <1> **cv::detail::AffineBestOf2NearestMatcher**
                    - <2> **cv::detail::BestOf2NearestRangeMatcher**
    
    ---
    
    - (4) 아무리 살펴보아도 이렇게 하위 API들을 살펴보아도 위 **cv::DescriptorMatcher::DescriptorCollection** class의 연관성을 찾기 쉽지 않았다. 하지만 해당 에러는 descriptor 관련 에러이므로 분명히 feature extraction과 matcher 속에서 문제가 분명 발생할 것이라고 생각했다. 따라서, 다시 cv::Stitcher를 구성하는 코드부터 살펴보았다.
        - cv::Stitcher를 사용하기 위해서는 **cv::Stitcher Stitcher::createDefault(bool try_use_gpu)** 를 가장 먼저 선언해주어야 한다. 이때 구성되는 FeaturesFinder 및 FeaturesMatcher관련 코드는 다음과 같다.
            
            ```cpp
            // FeaturesFinder Related Code !
            #ifdef HAVE_OPENCV_XFEATURES2D
                    stitcher.setFeaturesFinder(makePtr<detail::SurfFeaturesFinder>());
            #else
                    stitcher.setFeaturesFinder(makePtr<detail::OrbFeaturesFinder>());
            #endif
            
            // FeaturesMatcher Related Code !
            stitcher.setFeaturesMatcher(makePtr<detail::BestOf2NearestMatcher>(try_use_gpu));
            ```
            
            - 이를 통해서 살펴본 결과로는 XFEATURES2D를 사용하고 있으므로 해당 cv::Stitcher는 **cv::detail::SurfFeaturesFinder & cv::detail::BestOf2NearestMatcher** 를 사용하고 있음을 확인했다.
    
    ---
    
    - (5) 그래서 descriptor 관련 에러가 발생했으므로 해당 cv::detail::BestOf2NearestMatcher를 살펴보았다.  *(In matcher.cpp)*
        - CUDA를 사용하고 있지 않으므로 matcher는 다음과 같은 CpuMatcher를 사용하고 있다.
            
            ```cpp
            BestOf2NearestMatcher::BestOf2NearestMatcher(bool try_use_gpu, float match_conf, int num_matches_thresh1, int num_matches_thresh2)
            {
                (void)try_use_gpu;
            
            #ifdef HAVE_OPENCV_CUDAFEATURES2D
                if (try_use_gpu && getCudaEnabledDeviceCount() > 0)
                {
                    impl_ = makePtr<GpuMatcher>(match_conf);
                }
                else
            #endif
                {
                    impl_ = makePtr<CpuMatcher>(match_conf);
                }
            
                is_thread_safe_ = impl_->isThreadSafe();
                num_matches_thresh1_ = num_matches_thresh1;
                num_matches_thresh2_ = num_matches_thresh2;
            }
            ```
            
    
    ---
    
    - (6) 살펴본 결과, CpuMatcher는 FeaturesMatcher의 상속을 받고 있었으며, 이들의 코드를 더 상세하게 살펴보았다. *(In matcher.cpp)*
        - 이중에서 descriptor matching을 담당하는 match function까지 같이 살펴보면 다음과 같다.
            - **CpuMatcher Constructor**
                
                ```cpp
                class CpuMatcher : public FeaturesMatcher
                {
                public:
                    CpuMatcher(float match_conf) : FeaturesMatcher(true), match_conf_(match_conf) {}
                    void match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo& matches_info);
                
                private:
                    float match_conf_;
                };
                ```
                
            - **CpuMatcher match**
                
                ```cpp
                void CpuMatcher::match(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo& matches_info)
                {
                    Ptr<cv::DescriptorMatcher> matcher;
                		Ptr<flann::IndexParams> indexParams = makePtr<flann::KDTreeIndexParams>();
                    Ptr<flann::SearchParams> searchParams = makePtr<flann::SearchParams>();
                
                		if (features2.descriptors.depth() == CV_8U)
                    {
                        indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
                        searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
                    }
                
                    matcher = makePtr<FlannBasedMatcher>(indexParams, searchParams);
                
                		std::vector< std::vector<DMatch> > pair_matches;
                    MatchesSet matches;
                
                    // Find 1->2 matches
                    matcher->knnMatch(features1.descriptors, features2.descriptors, pair_matches, 2);
                
                		~~~~
                }
                ```
                
                - 더 상세하게 살펴보니 **knnMatch 방식을 사용하고 있었고 nearest candidate points(k)는 2개로 setting 되어 있음을 확인**할 수 있었다.
    
    ---
    
    - (7) 그러면 **cv::DescriptorMatcher** 과 **cv::detail::FeaturesFinder** 간의 관계를 찾아봐야하는데 이는 **knnMatch의 함수를 같이 사용하고 있음을 확인**할 수 있었다. 따라서 이는 동일한 에러가 발생하므로 위의 에러가 결국에는 cv::Stitcher라는 high level API에서도 나타날 수 있었던 것이었다.
    
    ---
    
    - (8) Features & Descriptor는 SURF라는 방식을 사용하고 있음을 미리 확인하였으므로 실제 SURF를 사용하고 나타난 keypoints(=features) 개수가 실제로 1개 미만인지 체크를 해야한다.
        - cv::detail::SurfFeaturesFinder는 **default로 hessian threshold 값을 300을 주고 계산**하기 때문에 이를 고려해서 이미지의 keypoint를 추출해본다.
    
    ---
    
    - (9) **[RESULT]**
        - Input Images
            - Image1
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_error1/Untitled 1.png" alt="">
                </figure>
                
            - Image2
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_error1/Untitled 2.png" alt="">
                </figure>                
                
            - Image3
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_error1/Untitled 3.png" alt="">
                </figure>
                
        - ORB로 추출했을 때의 결과
            
            ```bash
            To see img1 keypoints: 29, des: [32 x 29]
            To see img2 keypoints: 500, des: [32 x 500]
            To see img3 keypoints: 497, des: [32 x 497]
            
            To see des1 and des2 matching: 29
            To see des2 and des3 matching: 500
            ```
            
            - 이 결과로 보면 문제는 없어보인다.
        - ***** 위에서 분석한 함수들을 사용한 결과 **(SURF + knnMatch)** *****
            
            ```bash
            To see img1 keypoints: 1, des: [64 x 1]
            To see img2 keypoints: 366, des: [64 x 366]
            To see img3 keypoints: 212, des: [64 x 212]
            
            To see des1 and des2 matching: 1
            To see des2 and des3 matching: 366
            ```
            
            - 하지만!! 위 cv::Stitcher에서 사용한 방식을 적용하면 **image1의 keypoint는 1개가 나오게 된다.** 그래서 위의 에러가 발생하게 되었다.
    
    ---
    
    - (10) **[Solution]**
        - 이를 해결하기 위해서, cv::Stitcher에서 적용했던 방식을 사용하여 keypoint를 stitch 하기 전에 먼저 추출하여 **keypoint.size() ≥ 2 일때부터 stitch가 가능하도록 적용**시면 된다!!
            - **[Example Code] -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_stitching -lopencv_features2d -lopencv_flann -lopencv_xfeatures2d 사용!**
                
                ```cpp
                cv::Mat des1, des2, des3;
                std::vector<cv::KeyPoint> kpt1, kpt2, kpt3;
                
                cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(300);
                surf -> detectAndCompute(left, cv::Mat(), kpt1, des1);
                surf -> detectAndCompute(front, cv::Mat(), kpt2, des2);
                surf -> detectAndCompute(right, cv::Mat(), kpt3, des3);
                
                /* ------------------Solution Part !!------------------- */
                if(kpt1.size() < 2 || kpt2.size() < 2 || kpt3.size() < 2)
                	return;
                /* ----------------------------------------------------- */
                
                cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
                std::vector<std::vector<cv::DMatch>> matches1, matches2;
                
                matcher -> knnMatch(des1, des2, matches1, 2);
                matcher -> knnMatch(des2, des3, matches2, 2);
                ```
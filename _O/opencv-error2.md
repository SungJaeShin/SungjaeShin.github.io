---
title: "[OpenCV Error] Assertion failed (features1.descriptors.type() == features2.descriptors.type()) in function match"
excerpt: "OpenCV Error 2"
---
# [OpenCV Error] Assertion failed (features1.descriptors.type() == features2.descriptors.type()) in function match

- **[Error Sign in Terminal]**
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/opencv_error2/Untitled.png" alt="">
    </figure>
    

---

- **[Reference Site]**
    - OpenCV Github Code:
        
        - [opencv/stitching.hpp at 3.2.0 · opencv/opencv](https://github.com/opencv/opencv/blob/3.2.0/modules/stitching/include/opencv2/stitching.hpp)
        
        - [opencv/stitcher.cpp at 3.2.0 · opencv/opencv](https://github.com/opencv/opencv/blob/3.2.0/modules/stitching/src/stitcher.cpp)
        
        - [opencv/matchers.cpp at 3.2.0 · opencv/opencv](https://github.com/opencv/opencv/blob/3.2.0/modules/stitching/src/matchers.cpp)
        
    - Distance Related:
        
        - [Error:(-215) type == src2.type() && src1.cols == src2.cols && (type == 5 || type == 0) in function cv::batchDistance](https://stackoverflow.com/questions/50402842/error-215-type-src2-type-src1-cols-src2-cols-type-5-type)
        

---

- **[Solving Process 1]**
    - 해당 에러에 대해서 가장 의심이 드는 원인은 stitching을 하는 이미지들의 descriptor type 이 다르다는 것에 대한 에러라고 생각이 든다.
    - 그런데 걱정인 것은 해당 이미지 type들도 모두 동일하고 SURF를 통해서 얻어진 descriptor 방식에 대해서 모두 출력한 결과를 봐도 모두 동일한 type들을 가지고 있어서 이를 어떻게 해결할지 고민이 많이 된다.
        - 여기서 SURF를 통해서 얻어진 descriptor들은 총 3가지로 표현된다.
            - **[Method 1]**
                
                ```cpp
                cv::Mat image, des;
                std::vector<cv::KeyPoint> kpt;
                cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(300);
                surf -> detectAndCompute(image, cv::Mat(), kpt, des);
                ```
                
            
            ---
            
            - **[Method 2]**
                
                ```cpp
                cv::UMat gray_img, descriptor;
                cv::Mat image;
                cv::detail::ImageFeatures feature;
                
                cv::cvtColor(image, gray_img, cv::COLOR_BGR2GRAY);
                surf -> detectAndCompute(gray_img, cv::Mat(), feature.keypoints, descriptor);
                
                feature.descriptors = descriptor.reshape(1, (int)feature.keypoints.size());
                ```
                
            
            ---
            
            - **[Method 3]**
                
                ```cpp
                cv::UMat gray_img;
                cv::detail::ImageFeatures feat;
                
                cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
                cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create();
                
                detector->detect(gray_img, feat.keypoints);
                extractor->compute(gray_img, feat.keypoints, feat.descriptors);
                ```
                
    - 위에서 구한 방법론들을 사용하니깐 **“새롭게 느낀건 Method 1 & Method 2 의 feature 개수는 같은데 Method 3 의 feature 개수가 다르다.”** 이것도 추가로 판단해봐야 할 것 같다.
    - **그런데** **descriptor type의 결과는 모두 동일**했다. 해당 결과는 다음과 같다.
        
        ```cpp
        // Image Types: 16 == CV_8UC3
        **image1 type: 16
        image2 type: 16
        image3 type: 16**
        
        // [Method 1] KeyPoints and Descriptor Types
        **To see img1 keypoints: 57, des: [64 x 57]
        To see img2 keypoints: 577, des: [64 x 577]
        To see img3 keypoints: 692, des: [64 x 692]
        
        des1 descriptor type: 5
        des2 descriptor type: 5
        des3 descriptor type: 5
        
        //** [Method 2] KeyPoints and Descriptor Types
        **To see img1 keypoints: 57, des: [64 x 57]
        To see img2 keypoints: 577, des: [64 x 577]
        To see img3 keypoints: 692, des: [64 x 692]
        
        des1 descriptor type: 5
        des2 descriptor type: 5
        des3 descriptor type: 5
        
        //** [Method 3] KeyPoints and Descriptor Types
        **To see img1 keypoints: 576, des: [64 x 576]
        To see img2 keypoints: 885, des: [64 x 885]
        To see img3 keypoints: 1648, des: [64 x 1648]
        
        des1 descriptor type: 5
        des2 descriptor type: 5
        des3 descriptor type: 5**
        ```
        
    - 해당 코드를 low level 단에서 분석을 해보자는 결론을 내리고 분석을 시작한다.

---

- **[Analysis]**
    - **(0) Header about pre-defined variables**
        - **[1] stitching.hpp**
                        
            ```cpp
            double registr_resol_;
            double seam_est_resol_;
            double compose_resol_;
            double conf_thresh_;
            Ptr<detail::FeaturesFinder> features_finder_;
            Ptr<detail::FeaturesMatcher> features_matcher_;
            cv::UMat matching_mask_;
            Ptr<detail::BundleAdjusterBase> bundle_adjuster_;
            /* TODO OpenCV ABI 4.x
            Ptr<detail::Estimator> estimator_;
            */
            bool do_wave_correct_;
            detail::WaveCorrectKind wave_correct_kind_;
            Ptr<WarperCreator> warper_;
            Ptr<detail::ExposureCompensator> exposure_comp_;
            Ptr<detail::SeamFinder> seam_finder_;
            Ptr<detail::Blender> blender_;
            
            std::vector<cv::UMat> imgs_;
            std::vector<std::vector<cv::Rect> > rois_;
            std::vector<cv::Size> full_img_sizes_;
            std::vector<detail::ImageFeatures> features_;
            std::vector<detail::MatchesInfo> pairwise_matches_;
            std::vector<cv::UMat> seam_est_imgs_;
            std::vector<int> indices_;
            std::vector<detail::CameraParams> cameras_;
            double work_scale_;
            double seam_scale_;
            double seam_work_aspect_;
            double warped_image_scale_;
            ```
            
    - **(1) Error의 시작 function (Main Cause)**
        
        ```cpp
        Stitcher::Status Stitcher::stitch(InputArrayOfArrays images, const std::vector<std::vector<Rect> > &rois, OutputArray pano)
        {
        	CV_INSTRUMENT_REGION()
        
          Status status = estimateTransform(images, rois);
          if (status != OK)
              return status;
          return composePanorama(pano);
        }
        ```
        
        - **[1] estimateTransform Function (Main Cause)**
            
            ```cpp
            Stitcher::Status Stitcher::estimateTransform(InputArrayOfArrays images, const std::vector<std::vector<Rect> > &rois)
            {
                CV_INSTRUMENT_REGION()
            
                images.getUMatVector(imgs_);
                rois_ = rois;
            
                Status status;
            
                if ((status = matchImages()) != OK)
                    return status;
            
                if ((status = estimateCameraParams()) != OK)
                    return status;
            
                return OK;
            }
            ```
            
            - **1) getUMatVector Function**
                - 이 함수는 단순히 cv::Mat 의 정보를 cv::UMat 으로 옮기는 함수일 뿐이다.
            - **2) matchImages Function**
                
                ```cpp
                Stitcher::Status Stitcher::matchImages()
                {
                    if ((int)imgs_.size() < 2)
                    {
                        LOGLN("Need more images");
                        return ERR_NEED_MORE_IMGS;
                    }
                
                    work_scale_ = 1;
                    seam_work_aspect_ = 1;
                    seam_scale_ = 1;
                    bool is_work_scale_set = false;
                    bool is_seam_scale_set = false;
                    UMat full_img, img;
                    features_.resize(imgs_.size());
                    seam_est_imgs_.resize(imgs_.size());
                    full_img_sizes_.resize(imgs_.size());
                
                    LOGLN("Finding features...");
                #if ENABLE_LOG
                    int64 t = getTickCount();
                #endif
                
                    std::vector<UMat> feature_find_imgs(imgs_.size());
                    std::vector<std::vector<Rect> > feature_find_rois(rois_.size());
                
                    for (size_t i = 0; i < imgs_.size(); ++i)
                    {
                        full_img = imgs_[i];
                        full_img_sizes_[i] = full_img.size();
                
                        if (registr_resol_ < 0)
                        {
                            img = full_img;
                            work_scale_ = 1;
                            is_work_scale_set = true;
                        }
                        else
                        {
                            if (!is_work_scale_set)
                            {
                                work_scale_ = std::min(1.0, std::sqrt(registr_resol_ * 1e6 / full_img.size().area()));
                                is_work_scale_set = true;
                            }
                            resize(full_img, img, Size(), work_scale_, work_scale_);
                        }
                        if (!is_seam_scale_set)
                        {
                            seam_scale_ = std::min(1.0, std::sqrt(seam_est_resol_ * 1e6 / full_img.size().area()));
                            seam_work_aspect_ = seam_scale_ / work_scale_;
                            is_seam_scale_set = true;
                        }
                
                        if (rois_.empty())
                            feature_find_imgs[i] = img;
                        else
                        {
                            feature_find_rois[i].resize(rois_[i].size());
                            for (size_t j = 0; j < rois_[i].size(); ++j)
                            {
                                Point tl(cvRound(rois_[i][j].x * work_scale_), cvRound(rois_[i][j].y * work_scale_));
                                Point br(cvRound(rois_[i][j].br().x * work_scale_), cvRound(rois_[i][j].br().y * work_scale_));
                                feature_find_rois[i][j] = Rect(tl, br);
                            }
                            feature_find_imgs[i] = img;
                        }
                        features_[i].img_idx = (int)i;
                        LOGLN("Features in image #" << i+1 << ": " << features_[i].keypoints.size());
                
                        resize(full_img, img, Size(), seam_scale_, seam_scale_);
                        seam_est_imgs_[i] = img.clone();
                    }
                
                    // find features possibly in parallel
                    if (rois_.empty())
                        (*features_finder_)(feature_find_imgs, features_);
                    else
                        (*features_finder_)(feature_find_imgs, features_, feature_find_rois);
                
                    // Do it to save memory
                    features_finder_->collectGarbage();
                    full_img.release();
                    img.release();
                    feature_find_imgs.clear();
                    feature_find_rois.clear();
                
                    LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
                
                    LOG("Pairwise matching");
                #if ENABLE_LOG
                    t = getTickCount();
                #endif
                    (*features_matcher_)(features_, pairwise_matches_, matching_mask_);
                    features_matcher_->collectGarbage();
                    LOGLN("Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
                
                    // Leave only images we are sure are from the same panorama
                    indices_ = detail::leaveBiggestComponent(features_, pairwise_matches_, (float)conf_thresh_);
                    std::vector<UMat> seam_est_imgs_subset;
                    std::vector<UMat> imgs_subset;
                    std::vector<Size> full_img_sizes_subset;
                    for (size_t i = 0; i < indices_.size(); ++i)
                    {
                        imgs_subset.push_back(imgs_[indices_[i]]);
                        seam_est_imgs_subset.push_back(seam_est_imgs_[indices_[i]]);
                        full_img_sizes_subset.push_back(full_img_sizes_[indices_[i]]);
                    }
                    seam_est_imgs_ = seam_est_imgs_subset;
                    imgs_ = imgs_subset;
                    full_img_sizes_ = full_img_sizes_subset;
                
                    if ((int)imgs_.size() < 2)
                    {
                        LOGLN("Need more images");
                        return ERR_NEED_MORE_IMGS;
                    }
                
                    return OK;
                }
                ```
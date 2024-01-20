---
title: "[Image Stitching] How to fast panorama stitching using OpenCV API?"
excerpt: "Image Stitching: Fast Stitching"
---
# [Image Stitching] How to fast panorama stitching using OpenCV API?

---

- **[Goal]**
    - OpenCV 기반 panorama stitching 을 진행할 때 시간 소요가 많이 걸리는데, 많이 소요되는 시간을 판단해보고 빠르게 stitching할 수 있는 방법들을 확인해볼 수 있다.

---

- **[Reference Site]**
    
    - [https://github.com/OpenStitching/stitching_tutorial/blob/master/docs/Stitching%20Tutorial.md](https://github.com/OpenStitching/stitching_tutorial/blob/master/docs/Stitching%20Tutorial.md)
    
    - [https://github.com/opencv/opencv/blob/22edfd26280214f55b4ad93e070ebe10a76c0d04/modules/stitching/src/stitcher.cpp#L379C11-L379C17](https://github.com/opencv/opencv/blob/22edfd26280214f55b4ad93e070ebe10a76c0d04/modules/stitching/src/stitcher.cpp#L379C11-L379C17)
    
    - [https://github.com/opencv/opencv/blob/22edfd26280214f55b4ad93e070ebe10a76c0d04/samples/cpp/stitching_detailed.cpp#L390](https://github.com/opencv/opencv/blob/22edfd26280214f55b4ad93e070ebe10a76c0d04/samples/cpp/stitching_detailed.cpp#L390)
    

---

- **[Basic Procees]**
    
    (1) Find Features 
    
    → 이미지들간 연관성이 있는지 파악하기 위해 먼저 이미지들의 feature들을 extraction 및 이들간의 관계를 파악하기 위해 descriptor 계산
    
    ---
    
    (2) Match Features
    
    → 이미지들간의 feature 및 descriptor 들을 이용하여 연관성을 계산
    
    → 여기서 연관성은 confidence 라는 명칭을 사용하고 이들은 feature matching 된 개수를 이용하여 계산
    
    ```cpp
    **confidence = number of inliers / (8 + 0.3 * number of matches)
    
    // Output
    /* array([[0.        , 2.45009074, 0.56      , 0.44247788],
              [2.45009074, 0.        , 2.01729107, 0.42016807],
              [0.56      , 2.01729107, 0.        , 0.38709677],
              [0.44247788, 0.42016807, 0.38709677, 0.        ]]) */**
    ```
    
    - 이미지 1은 이미지 2와 가장 큰 연관성을 가지고 있다.
    - 이미지 2는 이미지 1 & 이미지 3과 높은 연관성을 가지고 있다.
    - 이미지 3은 이미지 2와 높은 연관성을 가지고 있다.
    - 이미지 4는 이미지 1, 2, 3 모두 연관성이 높지 않다.
    
    ---
    
    (3) Subset
    
    → 위에 계산된 confidence를 기반하여 stitching할 이미지들을 연결짓는 작업 진행
    
    → `Default Confidence Threshold: 1`
    
    → Results
    
    ```
    graph matches_graph{
    	"weir_1.jpg" -- "weir_2.jpg"[label="Nm=157, Ni=135, C=2.45009"];
    	"weir_2.jpg" -- "weir_3.jpg"[label="Nm=89, Ni=70, C=2.01729"];
    	"weir_noise.jpg";}
    ```
    
    - 이를 통해 image 4는 weir_noise로 분류되어 panorama stitching 에 사용되지 않음
    
    → 나머지 연결될 이미지들을 기반하여 confidence를 다시 계산
    
    → Results
    
    ```cpp
    ['imgs\\weir_1.jpg', 'imgs\\weir_2.jpg', 'imgs\\weir_3.jpg']
    [[0.         2.45009074 0.56      ]
     [2.45009074 0.         2.01729107]
     [0.56       2.01729107 0.        ]]
    ```
    
    ---
    
    (4) Camera parameter estimation
    
    → 정확한 wraping을 위해서 camera parameter을 계산
    
    → `CameraEstimator, CameraAdjuster, and WaveCorrector`
    
    ---
    
    (5) Warp images
    
    → 최종적으로 옮겨질 plane으로 옮겨질 곳으로 이미지들을 warp진행
    
    ---
    
    (6) Find Seam
    
    → 이미지들을 결합시키기 위해서 결합할 이미지들간 경계선을 찾는 작업을 진행
    
    ---
    
    (7) Exposure Error Compensation
    
    → 연결될 이미지들의 exposure가 다르기 때문에 이들을 맞춰주기 위한 작업을 진행
    
    ---
    
    (8) Blending 
    
    → 파노라마 이미지로 최종 만드는 작업 진행
    

---

- **[High Stitching API in OpenCV C++]**
    - OpenCV 기반 stitching 에서 제공하는 API (기본 Setting)
        - Code: [https://github.com/opencv/opencv/blob/4.x/modules/stitching/src/stitcher.cpp#L53](https://github.com/opencv/opencv/blob/4.x/modules/stitching/src/stitcher.cpp#L53)
            
            ```cpp
            
            void setRegistrationResol(double resol_mpx) { registr_resol_ = resol_mpx; }
            void setSeamEstimationResol(double resol_mpx) { seam_est_resol_ = resol_mpx; }
            void setCompositingResol(double resol_mpx) { compose_resol_ = resol_mpx; }
            void setPanoConfidenceThresh(double conf_thresh) { conf_thresh_ = conf_thresh; }
            void setWaveCorrection(bool flag) { do_wave_correct_ = flag; }
            void setInterpolationFlags(InterpolationFlags interp_flags) { interp_flags_ = interp_flags; }
            void setWaveCorrectKind(detail::WaveCorrectKind kind) { wave_correct_kind_ = kind; }
            void setFeaturesFinder(Ptr<Feature2D> features_finder) { features_finder_ = features_finder; }
            void setFeaturesMatcher(Ptr<detail::FeaturesMatcher> features_matcher) { features_matcher_ = features_matcher; }
            void setMatchingMask(const cv::UMat &mask) {matching_mask_ = mask.clone();}
            void setBundleAdjuster(Ptr<detail::BundleAdjusterBase> bundle_adjuster) { bundle_adjuster_ = bundle_adjuster; }
            void setEstimator(Ptr<detail::Estimator> estimator) { estimator_ = estimator; }
            void setWarper(Ptr<WarperCreator> creator) { warper_ = creator; }
            void setExposureCompensator(Ptr<detail::ExposureCompensator> exposure_comp) { exposure_comp_ = exposure_comp; }
            void setSeamFinder(Ptr<detail::SeamFinder> seam_finder) { seam_finder_ = seam_finder; }
            void setBlender(Ptr<detail::Blender> b) { blender_ = b; }
            ```
            
    
    ---
    
    - **** 변경할 수 있는 API method ****
        - Code: [https://github.com/opencv/opencv/blob/22edfd26280214f55b4ad93e070ebe10a76c0d04/samples/cpp/stitching_detailed.cpp#L390](https://github.com/opencv/opencv/blob/22edfd26280214f55b4ad93e070ebe10a76c0d04/samples/cpp/stitching_detailed.cpp#L390)
            - Feature Find Method
                
                ```cpp
                // ORB 
                cv::Ptr<Feature2D> finder = cv::ORB::create();
                // AKAZE
                cv::Ptr<Feature2D> finder = cv::AKAZE::create();
                // SIFT
                cv::Ptr<Feature2D> finder = cv::SIFT::create();
                // SURF
                cv::Ptr<Feature2D> finder = cv::xfeatures2d::SURF::create();
                ```
                
            - Matcher Method
                
                ```cpp
                // Affine NN
                cv::Ptr<FeaturesMatcher> matcher = cv::makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
                // Best kNN 
                cv::Ptr<FeaturesMatcher> matcher = cv::makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);
                // Range NN
                cv::Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);
                ```
                
            - Camera Estimator Method
                
                ```cpp
                // Affine Homography
                cv::Ptr<cv::detail::Estimator> estimator = cv::makePtr<AffineBasedEstimator>();
                // Homography
                cv::Ptr<cv::detail::Estimator> estimator = cv::makePtr<HomographyBasedEstimator>();
                ```
                
            - Bundle Adjustment Method
                
                ```cpp
                // Reprojection
                cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<cv::detail::BundleAdjusterReproj>();
                // Ray 
                cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<cv::detail::BundleAdjusterRay>();
                // Affine
                cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();
                // Not use BA
                cv::Ptr<cv::detail::BundleAdjusterBase> adjuster = cv::makePtr<NoBundleAdjuster>();
                ```
                
            - Warp Method
                
                ```cpp
                // Plane
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::PlaneWarper>();
                // Affine 
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::AffineWarper>();
                // Cylinderical
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::CylindricalWarper>();
                // Spherical (Default)
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::SphericalWarper>();
                // Fisheye
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::FisheyeWarper>();
                // Stereographic
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::StereographicWarper>();
                // PaniniA2B1
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::PaniniWarper>(2.0f, 1.0f);
                // PaniniPortraitA2B1
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
                // Mercator
                cv::Ptr<WarperCreator> warper_creator = cv::makePtr<cv::MercatorWarper>();
                ```
                
            - Exposure Method
                
                ```cpp
                // Gain
                cv::Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
                if (dynamic_cast<GainCompensator*>(compensator.get()))
                {
                    GainCompensator* gcompensator = dynamic_cast<GainCompensator*>(compensator.get());
                    gcompensator->setNrFeeds(expos_comp_nr_feeds);
                }
                // Channel
                cv::Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
                if (dynamic_cast<ChannelsCompensator*>(compensator.get()))
                {
                    ChannelsCompensator* ccompensator = dynamic_cast<ChannelsCompensator*>(compensator.get());
                    ccompensator->setNrFeeds(expos_comp_nr_feeds);
                }
                // Block
                cv::Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
                if (dynamic_cast<BlocksCompensator*>(compensator.get()))
                {
                    BlocksCompensator* bcompensator = dynamic_cast<BlocksCompensator*>(compensator.get());
                    bcompensator->setNrFeeds(expos_comp_nr_feeds);
                    bcompensator->setNrGainsFilteringIterations(expos_comp_nr_filtering);
                    bcompensator->setBlockSize(expos_comp_block_size, expos_comp_block_size);
                }
                ```
                
            - Seam Method
                
                ```cpp
                // Graph Cut (Default)
                cv::Ptr<SeamFinder> seam_finder = cv::makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
                cv::Ptr<SeamFinder> seam_finder = cv::makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
                // Dynamic Programming
                cv::Ptr<SeamFinder> seam_finder = cv::makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);
                cv::Ptr<SeamFinder> seam_finder = cv::makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
                // Voronoi
                cv::Ptr<SeamFinder> seam_finder = cv::makePtr<detail::VoronoiSeamFinder>();
                // Not Use 
                cv::Ptr<SeamFinder> seam_finder = cv::makePtr<detail::NoSeamFinder>();
                ```
                
            - Blender Method
                
                ```cpp
                // Feather 
                cv::Ptr<Blender> blender = cv::Blender::createDefault(blend_type, try_cuda);
                cv::FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
                fb->setSharpness(1.f/blend_width);
                blender->prepare(corners, sizes);
                // Multi Band
                cv::Ptr<Blender> blender = cv::Blender::createDefault(blend_type, try_cuda);
                cv::MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                blender->prepare(corners, sizes);
                ```
                
    
    ---
    
    - Github안 코드부분
        
        ```cpp
        // Feature Extraction (cv::Stitcher::Status Stitcher::matchImages())
        cv::Ptr<cv::Feature2D> features_finder_;
        std::vector<cv::detail::ImageFeatures> features_;
        std::vector<cv::UMat> feature_find_imgs(imgs_.size());
        std::vector<cv::UMat> feature_find_masks(masks_.size());
        **cv::detail::computeImageFeatures(features_finder_, feature_find_imgs, features_, feature_find_masks);**
        
        // Feature Matcher (cv::Stitcher::Status Stitcher::matchImages())
        cv::Ptr<cv::detail::FeaturesMatcher> features_matcher_;
        std::vector<cv::detail::MatchesInfo> pairwise_matches_;
        cv::UMat matching_mask_;
        **(*features_matcher_)(features_, pairwise_matches_, matching_mask_);**
        
        // Estimate Camera Parameters & Homography (cv::Stitcher::Status Stitcher::estimateCameraParams())
        std::vector<cv::detail::CameraParams> cameras_;
        cv::Ptr<cv::detail::Estimator> estimator_;
        **if (!(*estimator_)(features_, pairwise_matches_, cameras_))**
        
        // Correction using Bundle Adjustment (cv::Stitcher::Status Stitcher::estimateCameraParams())
        ****cv::Ptr<cv::detail::BundleAdjusterBase> bundle_adjuster_;
        **if (!(*bundle_adjuster_)(features_, pairwise_matches_, cameras_))**
        
        // Warp images (cv::Stitcher::Status Stitcher::composePanorama())
        cv::Ptr<cv::WarperCreator> warper_;
        double warped_image_scale_;
        double seam_work_aspect_;
        cv::Ptr<cv::detail::RotationWarper> w = warper_->create(float(warped_image_scale_ * seam_work_aspect_));
        **w->warp(masks[i], K, cameras_[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);**
        
        // Compensate exposure 
        Ptr<detail::ExposureCompensator> exposure_comp_;
        std::vector<UMat> masks_warped(imgs_.size());
        std::vector<UMat> images_warped(imgs_.size());
        std::vector<Point> corners(imgs_.size());
        exposure_comp_->feed(corners, images_warped, masks_warped);
        **exposure_comp_->apply(int(i), corners[i], images_warped[i], masks_warped[i]);**
        
        // Find Seam
        cv::Ptr<cv::detail::SeamFinder> seam_finder_;
        std::vector<UMat> images_warped_f(imgs_.size());
        **seam_finder_->find(images_warped_f, corners, masks_warped);**
        
        // Blender 
        cv::Ptr<cv::detail::Blender> blender_;
        cv::UMat result;
        cv::UMat result_mask_;
        blender_->feed(img_warped_s, mask_warped, corners[img_idx]);
        **blender_->blend(result, result_mask_);**
        ```
        

---

- **[Which one is fast stitching?]**
    - Stitching Pair (EuRoC Dataset)
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled.png" alt="">
        </figure> 
        
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled 1.png" alt="">
        </figure> 
        
    - Automatic stitching result → **4.00 sec**
    - 여러 API를 건들였을때, 어느 결과가 가장 빠르게 stitching 되었는지 확인해본 결과 다음과 같이 적용할 때, 가장 빠른 것으로 확인 → **0.9 sec**
        
        ```cpp
        stitcher->setFeaturesMatcher(cv::makePtr<cv::detail::BestOf2NearestMatcher>(false));
        stitcher->setBundleAdjuster(cv::makePtr<cv::detail::BundleAdjusterRay>());
        stitcher->setExposureCompensator(cv::makePtr<cv::detail::GainCompensator>());
        stitcher->setSeamFinder(cv::makePtr<cv::detail::DpSeamFinder>(cv::detail::DpSeamFinder::COLOR));
        stitcher->setBlender(cv::makePtr<cv::detail::MultiBandBlender>(false));
        ```
        
        - Stitching 속도를 크게 좌우하는 함수들은 `setExposureCompensator, setSeamFinder` 이다.
    - **Automatic하게 Stitching하게 되면 4초의 stitching 시간이 걸리는데, 위의 방법론을 적용하면 0.9초까지 앞당길 수 있다.**
    
    ---
    
    - Results
        - Original Result → **(4.0 sec)**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled.jpeg" alt="">
            </figure> 
            
        - Seam (Dynamic Programming) → **(3.2 sec)**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled 1.jpeg" alt="">
            </figure> 
            
        - Seam (Dynamic Programming) + BA (Affine Partial) → **(1.8 sec)**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled 2.jpeg" alt="">
            </figure> 
            
        - Seam (Dynamic Programming) + BA (Reprojection) → **(2.8 sec)**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled 3.jpeg" alt="">
            </figure> 
            
        - Seam (Dynamic Programming) + Exposure (Gain Compensator) → **(0.9 sec)**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/stitch_fast/Untitled 4.jpeg" alt="">
            </figure>             
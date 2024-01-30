---
title: "[CVPR 2016] Structure-from-Motion Revisited"
classes: wide
---
2023-07-19

## Author: J. L. Schönberger and J. -M. Frahm

### LAB: Computer Vision and Geometry Group at ETH Zürich

---

- **[간단한 설명]**
    - 새로운 SfM 방식을 적용하여 reconstruction 진행
    - 이 방식으로 incremental SfM 방식의 문제점인 robust & accuracy & completeness and scability 부분을 해결해낼 수 있었음 (unordered 된 이미지들을 이용)

---

- **[선택한 이유]**
    - 이 논문이 COLMAP이라고 불리우는 open source 의 논문
    - 많은 논문들이 기본적으로 이 논문을 많이 이용하여 reconstruction 및 3D visual pose 추정 또는 많은 citation

---

- **[General Structure-from-Motion Method]**
    
    <figure class="align-center">
      <img src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/colmap/pipeline.png" alt="">
    </figure>
    
    - Incremental SfM은 iterative한 reconstruction method
        
        (1) Geometric verification을 이용한 feature matching을 진행
        
        (2) Two-view reconstruction을 위한 이미지가 선택
        
        (3) 그 후 triangulation & BA 후 incrementally reconstruction 진행
        

---

- **[제안하는 방법론]**
    - 원인: Drift나 잘못 registration 및 large scale에 대한 reconstruction 은 쉽지 않음
    - 해결 방안:
        
        (1) Geometric Verification Strategy → the robustness of initialization and triangulation
        
        (2) Next Best View Selection → the robustness of incremental reconstruction process
        
        (3) Triangulation Strategy →  the robustness of triangulation process reducing cost function
        
        (4) Iterative Bundle Adjustment → the robustness of re-triangulation and outlier filtering process reducing drifts
        
        (5) Redundant View Mining   → the robustness of reconstruction for efficient bundle adjustment parameterize
        
    - 비교 알고리즘: **Bundler / VisualSFM**
    
    ---
    
    - Part 1. Scene Graph Augmentation
        - [Goal] Augment the scene graph with appropriate geometric relation
            
            (1) Estimate Fundamental matrix (F) → 이를 이용한 inlier correspondence pair 개수 ($N_F$)를 찾고 threshold 보다 높으면 pass (Using epipolar constraint)
            
            (2) Estimate Homography matrix (H) → 역시 이를 이용한 inlier correspondence pair 개수 ($N_H$)를 찾고 $(N_H/N_F > \epsilon_{HF})$ 라면 pass (그게 아니라면 moving scene이라고 간주)
            
            (2-1) Intrinsic parameter를 알고 있다면 essential matrix (E)를 추정 후 해당 threshold를 넘어가게 되면 correctly calibration이 되었다고 간주 $(N_E/N_F > \epsilon_{EF})$
            
            (2-2) 해당 threshold 보다 작게된다면 essential matrix를 decompose하여 triangulation angle $\alpha_{m}$ 을 통해 median 값을 적용
            
            (3) Similarity transformation matrix를 이용하여 image border 안에 있는 inliers ($N_s$)를 구해 scene graph에 잘못 들어온 이미지들이 없는지 방안 마련
            
            <figure class="align-center">
              <img src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/colmap/similarity_map.png" alt="">
            </figure>
            
            - Ref Paper: **Fixing WTFs: Detecting Image Matches caused by Watermarks, Timestamps, and Frames in Internet Photos**
                - 관련 내용: We assume that for WTFs, similar regions appear mainly at the image borders, while for valid matches, similar regions appear all over the image (Fig. 4). Therefore, we compute spatial histograms of the similarity maps to detect WTFs. We investigate four histogram shapes (Fig. 5).
    
    ---
    
    - Part 2. Next Best View Selection
        - [Problems]
            
            (1) 나머지 reconstruction 결과에 모두 영향을 끼침
            
            (2) pose estimation과 triangulation의 결과에 안좋은 영향을 끼침
            
            (3) BA 결과가 안좋아서 estimation이 되지 않을 수도 있음 
            
        
        ---
        
        - [Goal] Choosing the next best view in robust SfM aims to minimize the reconstruction error given uncertainty driven images
            
            (1) 다음과 같은 조건들을 만족해야 가장 best image selection에 부합이됨 (Score $S$ 계산)
            
            조건 1. Visible points (3D points ro triangulated points) 가 일정 개수 ($N_t$) 이상 보이는지 계속 추정
            
            조건 2. 해당 Point들의 이미지 상의 분포가 uniform distribution 인지 (고르게 분포가 되어 있는지) 체크
            
            ** **Point가 많고 uniform 할수록 score $S$ 는 가장 높을 것임 !!** **
            
            (2) 다음과 같은 방법들을 사용
            
            <figure class="align-center">
              <img src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/colmap/score_points.png" alt="">
            </figure>
            
            1. 2개의 이미지를 grid로 나눔 ($K_l$)
            2. 각 cell들은 2가지 상태를 가짐 ***(empty & full)***
                1. State change case (empty cell → full cell): Score $S_i$ 는 weight ($w_l$) 에 의해서 올라갈 것임 
            3. Cell이 전체 score에 영향을 한번만 주는데 한 부분에 있는 경우보다 uniform하게 분포가 되어 있는 것을 더 선호함 (하나의 cell에 point들이 많이 모여 있으면 결국 cell 당 점수를 부여받기 때문에 점수는 고르게 있는 cell 분포보다는 적게 받을 것임)
            4. 만약 cell의 개수가 visible point들보다 훨씬 많은 경우 ($N_t << K^{2}_{l}$) 에는 distribution 계산이 쉽지 않을 수 있음 (따로 분리) 
            5. 동시에 multi-resolution 방식을 통해서 각 grid를 또 L-level까지 반복적으로 나누어줌 ($K_l = 2^l$)
            6. 각 resolution 마다 score를 계산하고 이들을 모두 합쳐 누적됨
            
            ** **이를 통해서 point 개수들과 distribution 모두 고려할 수 있게 되었음 !!** **
            
    
    ---
    
    - Part 3. Robust and Efficient Triangulation
        - Image의 correspondence pair들을 높히기 위해서는 large baseline이 필요하므로 outlier들이 끼어 있는 robust feature tracking 과정을 추가 (RANSAC 과정임)
        
        ---
        
        - [Goal] An efficient, sampling-based triangulation method that can robustly estimate all points within an outlier-contaminated feature track
            
            (1) Robust triangulation을 진행하기 위해 다음과 같은 조건들을 진행해야함
            
            조건 1. 3D Refinement를 진행하기 전 track feature들을 묶어주는 set를 생성 
            
            조건 2. 잘못된 정합을 방지하기 위해서 triangulation을 recursive하게 진행하여 faulty feature들을 찾음
            
            (2) 다음과 같은 방법들을 사용 
            
            1. Known: 각 2D feature들과 camera projective matrix를 알고 있을 때 $\bar{x}_{a}, \bar{x}_{b}, P_a, P_b$ 를 통한 3D triangulation 진행
            2. DLT를 이용한 triangulation method를 사용
            3. Estimated 된 3D point를 통해서 sufficient triangulation angle $\alpha$ 를 측정
                
                <figure class="align-center">
                  <img src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/colmap/projection.png" alt="">
                </figure>
                
            4. 
            5. 각 camera에서의 positive한 depth를 얻음 ($d_a, d_b$) 
            6. Positive depth & reprojection error를 일정 threshold t보다 작으면 random sampling을 통해 feature set  선택
            7. K번의 RANSAC 과정을 거쳐 best 3D point를 find
    
    ---
    
    - Part 4. Bundle Adjustment
        - Local BA를 먼저 진행하는데 이는 가장 가까운 이미지들로 local BA를 진행하고 특정 percentage마다 global BA를 진행
        - 다음과 같은 단계를 적용
            
            (1) Parameterization: Outlier들을 제거하기 위해 Cauchy function을 loss function 으로 적용하고 Sparse Direct Solver 를 이용하여 많은 parameter들을 optimization 진행
            
            (2) Filtering: BA를 통해서 얻어진 결과 중 일부 observation은 해당 모델과 fit하지 않게 될 수 있음. 그래서 reprojection error가 large 하면 filtering 진행 + triangulation angle 이 최소가 되는 ray pair들을 check. 
            
            (3) Re-Triangulation: Global BA를 위한 prior를 제공하기 위해 global BA 전 한번 더 triangulation 과정을 진행 (이것을 re-triangulation이라 부름). 그리고 global BA를 통해 parameter들이 대게 향상되므로 그 이후 triangulation을 이용하여 다음 pre-triangulation의 prior를 제공 (그런데 더 효율적인 prior를 제공할 수 있음). 이 목적은 부정확한 pose 추정으로 인해 triangulation 이 실패한 지점을 계속 tracking 하여 reconstruction의 완성도를 높히는 목적. 다음 step의 redundancy한 tracking을 보여줄 수 있음
            
            (4) Iterative Refinement: Iterative 하게 filtering & Re-triangulation & BA과정을 진행하여 최종 reconstruction 결과를 좋게 만들 수 있음
            
    
    ---
    
    - Part 5. Redundant View Mining
        - [Goal] Overlap되는 부분들이 많은 불필요한 이미지들을 하나의 group으로 묶어 더 효율적인 BA를 하기 위해
            
            (1) 이를통해 여러 카메라들을 하나로 묶어 expensive한 group을 줄일 수 있도록 효율적인 변경
            
            (2) 많은 카메라를 하나의 submap으로 clustering하는 대신 장면들을 여러 개의 작고 많이 겹치는 카메라 group으로 분할 (Solving cost of camera system 이 감소)
            
            (3) 더 작고 많이 겹치는 camera group의 결과를 이용하여 (separator ??) variable optimization을 넘어갈 수 있음
            
        
        ---
        
        - 다음과 같은 방법들을 사용
            
            (1) Highly overlapped 된 이미지들은 하나의 group으로 묶어 마치 하나의 camera 처럼 변경 ($N_G$: group 개수)
            
            (2) Affected image들은 새롭게 추가된 이미지이거나 관측된 point가 reprojection error point보다 더 멀리 있는 경우를 체크
            
            (3) 각 이미지들은 binary visibility vector 로 구성되어 있는데 point $X_n$ 이 이미지 $i$ 에서 보인다면 1, 그렇지 않다면 0으로 설정하고 해당 각 이미지에 보이는 point들은 $N_X$ 로 정의 (즉, N개의 scene 마다 feature point를 binary vector화)
            
            (4) 두 이미지들을 binary vector화를 진행하고 bitwise 계산을 통해서 두 scene의 상호연관성을 계산 ($V_{ab}$ 값이 크다면 그만큼 연관성이 높다는 의미, 분모는 a와 b의 모든 visibility, 분자는 a와 b에서 동시에 관측되는 points)
            
            (5) 첫번째 이미지인 $l_a$ 는 $V_{ab}$ 가 높은 $l_b$ 를 찾게되면 제거되고 group $G_r$ 을 initialization 진행 (새로운 group을 생성)
            
            (6) 이렇게 grouping이 되면 group 기반 group-local BA를 진행하여 group 단위로 parameter들을 optimization 진행
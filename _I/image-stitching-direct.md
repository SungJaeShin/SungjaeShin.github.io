---
title: "[Image Stitching] Direct Image Stitching using Homography"
excerpt: "Image Stitching: Direct stitch"
---
# [Image Stitching] Direct Image Stitching using Homography

---

- **[Goal]**
    - Panorama image를 생성할 때, 가장 빠르게 stitching하는 방법에 대해서 고찰한 결과를 공유하고자 한다.

---

- **[Reference Site]**
    
    - [GitHub - SungJaeShin/Image_Stitching: OpenCV based Image Stitching applied various method (using RealSense Camera) !!](https://github.com/SungJaeShin/Image_Stitching.git)
    

---

- **[Basic Process]**
    - 기본적으로 auto stitching 하는 방식 중에서 가장 많이 사용되는 방식은 perspective transformation인 homography를 추정하여 stitching 하는 방식이다.
    - Panorama 이미지를 생성하는 과정이 아주 많지만 핵심적으로 homography 기반 transformation 후 바로 합치면 바로 파노라마 이미지를 생성할 수 있기 때문에 이를 기반하여 바로 생성한다.
        - **그러나!! 하나 주의할 점은 OpenCV기반 transformation을 하면 src에 해당하는 image로 잘 transformation되지만 일정 부분에 대한 이미지가 제거가 되기 때문에 이를 살리기 위해서 직접 transformation 후 stitching을 진행한다.**

---

- **[Problem Setting]**
    - Test Pair
        <table>
        <tr>
            <td> Image 1 </td>
            <td> Image 2 </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 1.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>  
        
        
    - Direct stitching results
        <table>
        <tr>
            <td> Using OpenCV API transformation (image 2 → image 1) </td>
            <td> Not using OpenCV API transformation (image 2 → image 1) </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 2.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 3.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>  
        
        - **다음의 결과처럼 OpenCV API를 사용하게 되면 나머지 가장자리 이미지 pixel들은 제거가 되는 현상이 있기 때문에 이를 살리기 위해서 직접 transformation 진행 !!**

---

- **[Process]**
    - [Step 1] Extract good feature matching result
        - Homography를 추정하기 전에 image 1과 image 2간의 가장 좋은 feature matching 결과를 얻기 위해서 다음과 같은 방법들을 적용
            - SIFT features + DAISY descriptors + BF matcher + KNN matching
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 4.png" alt="">
                </figure>                 
    
    ---
    
    - [Step 2] Estimate Homography
        - 가장 좋은 feature matching 결과를 얻었다면 이들을 기반으로 homography 추정을 진행
            - `cv::findHomography` API 적용
    
    ---
    
    - [Step 3] 추정된 Homography를 이용하여 image 2를 image 1 plane으로 transformation 진행
        - Basic Code
            
            ```cpp
            std::vector<cv::Point2f> img1_2d, img2_2d;
            cv::Mat H = cv::findHomography(img1_2d, img2_2d, CV_RANSAC, 1);
            
            cv::Mat warpImg = calcROIImg(img2, H, x_diff, y_diff, x_min, y_min, x_max, y_max);
            
            // WarpImg pixels
            for(int y = 0; y < img2.rows; y++)
            {
                for(int x = 0; x < img2.cols; x++)
                {
                    // Get RGB values
                    cv::Vec3b rgb = img2.at<cv::Vec3b>(y, x);
            
                    cv::Mat tmp_point(3,1,cv::DataType<double>::type);
                    tmp_point.at<double>(0,0) = x;
                    tmp_point.at<double>(1,0) = y;
                    tmp_point.at<double>(2,0) = 1;
            
                    cv::Mat H_tmp_point = H.inv() * tmp_point;
                    H_tmp_point = H_tmp_point / H_tmp_point.at<double>(2,0);
                    H_tmp_point.at<double>(0,0) = H_tmp_point.at<double>(0,0) - x_min;
                    H_tmp_point.at<double>(1,0) = H_tmp_point.at<double>(1,0) - y_min;
                
                    warpImg.at<cv::Vec3b>(H_tmp_point.at<double>(1,0), H_tmp_point.at<double>(0,0)) = rgb;
                }
            }
            ```
            
        - Code Results
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 5.png" alt="">
            </figure> 
                
    ---
    
    - [Step 4] Linear Interpolation for warped image2
        - Perspective transformation 진행하는데 빈 pixel들이 존재하므로 이는 linear interpolation 기반하여 해당 pixel들의 빈 공간을 채우는 작업을 진행
            - Basic Linear Interpolation Code
                
                ```cpp
                cv::Vec3b PixelLinearInter(cv::Vec3b pt1, cv::Vec3b pt2, double t)
                {
                    int interpolation_r = (int)pt1[0] + t * ((int)pt2[0] - (int)pt1[0]);
                    int interpolation_g = (int)pt1[1] + t * ((int)pt2[1] - (int)pt1[1]);
                    int interpolation_b = (int)pt1[2] + t * ((int)pt2[2] - (int)pt1[2]);
                    cv::Vec3b inter_rgb(interpolation_r, interpolation_g, interpolation_b);
                
                    return inter_rgb;
                }
                ```
                
        - Basic Code
            
            ```cpp
            // Linear Interpolation pixels
            cv::Mat interImg = warpImg.clone();
            for(int y = 1; y < warpImg.rows - 1; y++)
            {
                for(int x = 1; x < warpImg.cols - 1; x++)
                {
                    int cur_r = (int)warpImg.at<cv::Vec3b>(y, x)[0];
                    int cur_g = (int)warpImg.at<cv::Vec3b>(y, x)[1];
                    int cur_b = (int)warpImg.at<cv::Vec3b>(y, x)[2];
            
                    int count = 0;
                    for(int p = -1; p <= 1; p++)
                    {
                        for(int q = -1; q <= 1; q++)
                        {
                            if(p == 0 && q == 0)
                                continue;
                            int tmp_r = (int)warpImg.at<cv::Vec3b>(y+p, x+q)[0];
                            int tmp_g = (int)warpImg.at<cv::Vec3b>(y+p, x+q)[1];
                            int tmp_b = (int)warpImg.at<cv::Vec3b>(y+p, x+q)[2];
            
                            if(tmp_r == 0 && tmp_g == 0 && tmp_b == 0)
                                count++;
                        }
                    }
            
                    if(cur_r == 0 && cur_g == 0 && cur_b == 0)
                    {
                        if(count >= 0 && count < 7)
                        {
                            bool minus = false;
                            bool plus = false; 
            
                            int final_inter_r, final_inter_g, final_inter_b;
            
                            cv::Vec3b pt1 = warpImg.at<cv::Vec3b>(y-1, x-1);
                            cv::Vec3b pt2 = warpImg.at<cv::Vec3b>(y+1, x+1);
                            cv::Vec3b inter_pt1 = PixelLinearInter(pt1, pt2, 0.5);
            
                            cv::Vec3b pt3 = warpImg.at<cv::Vec3b>(y-1, x+1);
                            cv::Vec3b pt4 = warpImg.at<cv::Vec3b>(y+1, x-1);
                            cv::Vec3b inter_pt2 = PixelLinearInter(pt3, pt4, 0.5);
            
                            cv::Vec3b compare(0, 0, 0);
                            if(pt1 == compare || pt2 == compare)
                            {
                                final_inter_r = (int)inter_pt2[0];
                                final_inter_g = (int)inter_pt2[1];
                                final_inter_b = (int)inter_pt2[2];
                            }
                            else if(pt3 == compare || pt4 == compare)
                            {
                                final_inter_r = (int)inter_pt1[0];
                                final_inter_g = (int)inter_pt1[1];
                                final_inter_b = (int)inter_pt1[2];
                            }
                            else
                            {
                                final_inter_r = ((int)inter_pt1[0] + (int)inter_pt2[0]) / 2;
                                final_inter_g = ((int)inter_pt1[1] + (int)inter_pt2[1]) / 2;
                                final_inter_b = ((int)inter_pt1[2] + (int)inter_pt2[2]) / 2;
                            }
            
                            cv::Vec3b final_inter_pt(final_inter_r, final_inter_g, final_inter_b);
                            interImg.at<cv::Vec3b>(y, x) = final_inter_pt;
                        }
                    }
                }
            }
            ```
            
        - Code Results
            - Black pixel 기반으로 **“좌상-우하” + “좌하-우상”** pixel들의 interpolation을 종합하여 black pixel을 채워주는 작업 진행
                <table>
                <tr>
                    <td> 좌상-우하 (기울기가 마이너스) 경우에 대한 Linear interpolation 결과 </td>
                    <td> 좌하-우상 (기울기가 플러스) 경우에 대한 Linear interpolation 결과 </td>
                </tr> 
                <tr>
                <td>
                    <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 6.png" alt="">
                    </figure> 
                </td>
                <td>
                    <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 7.png" alt="">
                    </figure> 
                </td>
                </tr>
                </table>  
                
                - 이들 interpolation 을 합친 최종 warp image 2 결과
                    <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 8.png" alt="">
                    </figure>                     
    
    ---
    
    - [Step 5] 이동한만큼 최종 파노라마 이미지 사이즈를 늘리고 translation 진행 후 image 1과 warped image 2를 합치는 작업 진행
        - Basic Code
            
            ```cpp
            cv::Mat AddHomographyImg(cv::Mat img1, cv::Mat img2, cv::Mat warp_img2, cv::Mat H)
            {
                double x_diff, y_diff;
                double x_min, y_min, x_max, y_max;
                cv::Mat ROIImg = calcROIImg(img2, H, x_diff, y_diff, x_min, y_min, x_max, y_max);
            
                // Translate img1 for warp img2 
                cv::Mat img1_trans = img1.clone();
                if(x_min <= 0 && y_min > 0)
                {
                    cv::Mat x_trans_img1(img1.rows, (img1.cols - x_min), img1.type(), cv::Scalar(0,0,0));
                    for(int y = 0; y < img1.rows; y++)
                    {
                        for(int x = 0; x < img1.cols; x++)
                        {
                            cv::Vec3b rgb = img1.at<cv::Vec3b>(y, x);
                            x_trans_img1.at<cv::Vec3b>(y, x - x_min) = rgb;
                        }
                    }
                    img1_trans = x_trans_img1.clone();
                }
                else if(x_min > 0 && y_min < 0)
                {
                    cv::Mat y_trans_img1((img1.rows - y_min), img1.cols, img1.type(), cv::Scalar(0,0,0));
                    for(int y = 0; y < img1.rows; y++)
                    {
                        for(int x = 0; x < img1.cols; x++)
                        {
                            cv::Vec3b rgb = img1.at<cv::Vec3b>(y, x);
                            y_trans_img1.at<cv::Vec3b>(y - y_min, x) = rgb;
                        }
                    }
                    img1_trans = y_trans_img1.clone();
                }
                else if(x_min < 0 && y_min < 0)
                {
                    cv::Mat xy_trans_img1((img1.rows - y_min), (img1.cols - x_min), img1.type(), cv::Scalar(0,0,0));
                    for(int y = 0; y < img1.rows; y++)
                    {
                        for(int x = 0; x < img1.cols; x++)
                        {
                            cv::Vec3b rgb = img1.at<cv::Vec3b>(y, x);
                            xy_trans_img1.at<cv::Vec3b>(y - y_min, x - x_min) = rgb;
                        }
                    }
                    img1_trans = xy_trans_img1.clone();
                }
                
                double img1_col = img1_trans.cols;
                double img1_row = img1_trans.rows;
                double img2_col = warp_img2.cols;
                double img2_row = warp_img2.rows;
            
                cv::Mat panoImg;
                if(img1_col <= img2_col && img1_row <= img2_row)
                {
                    cv::Mat transImg(img2_row, img2_col, img1.type(), cv::Scalar(0,0,0));
                    panoImg = transImg.clone();
                }
                else if(img1_col <= img2_col && img1_row > img2_row)
                {
                    cv::Mat transImg(img1_row, img2_col, img1.type(), cv::Scalar(0,0,0));
                    panoImg = transImg.clone();
                }
                else if(img1_col > img2_col && img1_row <= img2_row)
                {
                    cv::Mat transImg(img2_row, img1_col, img1.type(), cv::Scalar(0,0,0));
                    panoImg = transImg.clone();
                }
                else
                {
                    cv::Mat transImg(img1_row, img1_col, img1.type(), cv::Scalar(0,0,0));
                    panoImg = transImg.clone();
                }
            
                std::cout << "trans img size: " << img1_trans.size() << std::endl;
                std::cout << "pano img size: " << panoImg.size() << std::endl;
            
                // Add two imgs
                for(int y = 0; y < panoImg.rows; y++)
                {
                    for(int x = 0; x < panoImg.cols; x++)
                    {
                        int final_r, final_g, final_b;
                        cv::Vec3b compare(0, 0, 0);
            
                        if(img1_col <= x && x < img2_col)
                        {
                            cv::Vec3b cur_rgb2 = warp_img2.at<cv::Vec3b>(y, x);
                            if(cur_rgb2 == compare)
                                continue;
                            else
                            {
                                final_r = (int)cur_rgb2[0];
                                final_g = (int)cur_rgb2[1];
                                final_b = (int)cur_rgb2[2];
                            }
                            
                            cv::Vec3b final_pt(final_r, final_g, final_b);
                            panoImg.at<cv::Vec3b>(y, x) = final_pt;
            
                            continue;
                        }
                        else if(img2_col <= x && x < img1_col)
                        {
                            cv::Vec3b cur_rgb1 = img1_trans.at<cv::Vec3b>(y, x);
                            if(cur_rgb1 == compare)
                                continue;
                            else
                            {
                                final_r = (int)cur_rgb1[0];
                                final_g = (int)cur_rgb1[1];
                                final_b = (int)cur_rgb1[2];
                            }
                            
                            cv::Vec3b final_pt(final_r, final_g, final_b);
                            panoImg.at<cv::Vec3b>(y, x) = final_pt;
            
                            continue;
                        }    
            
                        if(img1_row <= y && y < img2_row)
                        {
                            cv::Vec3b cur_rgb2 = warp_img2.at<cv::Vec3b>(y, x);
                            if(cur_rgb2 == compare)
                                continue;
                            else
                            {
                                final_r = (int)cur_rgb2[0];
                                final_g = (int)cur_rgb2[1];
                                final_b = (int)cur_rgb2[2];
                            }
                            
                            cv::Vec3b final_pt(final_r, final_g, final_b);
                            panoImg.at<cv::Vec3b>(y, x) = final_pt;
            
                            continue;
                        }
                        else if(img2_row <= y && y < img1_row)
                        {
                            cv::Vec3b cur_rgb1 = img1_trans.at<cv::Vec3b>(y, x);
                            if(cur_rgb1 == compare)
                                continue;
                            else
                            {
                                final_r = (int)cur_rgb1[0];
                                final_g = (int)cur_rgb1[1];
                                final_b = (int)cur_rgb1[2];
                            }
                            
                            cv::Vec3b final_pt(final_r, final_g, final_b);
                            panoImg.at<cv::Vec3b>(y, x) = final_pt;
            
                            continue;
                        }
            
                        // Get RGB values 
                        cv::Vec3b cur_rgb1 = img1_trans.at<cv::Vec3b>(y, x);
                        cv::Vec3b cur_rgb2 = warp_img2.at<cv::Vec3b>(y, x);
            
                        if(cur_rgb1 == compare && cur_rgb2 == compare)
                            continue;
                        else if(cur_rgb1 == compare && cur_rgb2 != compare)
                        {
                            final_r = (int)cur_rgb2[0];
                            final_g = (int)cur_rgb2[1];
                            final_b = (int)cur_rgb2[2];
                        }
                        else if(cur_rgb1 != compare && cur_rgb2 == compare)
                        {
                            final_r = (int)cur_rgb1[0];
                            final_g = (int)cur_rgb1[1];
                            final_b = (int)cur_rgb1[2];
                        }
                        else
                        {
                            final_r = ((int)cur_rgb1[0] + (int)cur_rgb2[0]) / 2;
                            final_g = ((int)cur_rgb1[1] + (int)cur_rgb2[1]) / 2;
                            final_b = ((int)cur_rgb1[2] + (int)cur_rgb2[2]) / 2;
                        }
            
                        cv::Vec3b final_pt(final_r, final_g, final_b);
                        panoImg.at<cv::Vec3b>(y, x) = final_pt;
                    }
                }
            
                return panoImg;
            }
            ```
            
        - Code Results
            <table>
                <tr>
                    <td> OpenCV 기반 warp API 사용 후 direct stitching 한 결과 </td>
                    <td> Homography 추정 후 OpenCV warp API 사용하지 않고 direct stitching 한 결과 </td>
                </tr> 
                <tr>
                <td>
                    <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 9.png" alt="">
                    </figure> 
                </td>
                <td>
                    <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/stitch_direct/Untitled 10.png" alt="">
                    </figure> 
                </td>
                </tr>
                </table> 

---

- **[Stitching Time]**
    
    
    | Method | Auto Stitching | Fast Stitching API | Only Homography Stitching | Only Homography Stitching (not use warp API) |
    | --- | --- | --- | --- | --- |
    | Time (sec) | 4.00 | 0.9 | 0.02 | 0.37 |

---

- **[Analysis]**
    - Homography 추정이 엄청 정확한 것이 아니면 direct stitching 결과가 좋지 않아서 이는 위험성이 존재하기 때문에 추가적으로 문제를 해결해야할 필요가 있음
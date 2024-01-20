---
title: "[Feature Matching: RoMa] Robust Feature Matching in case of different viewpoints between query image and candidate image"
excerpt: "Feature Matching: RoMa"
---
# [Feature Matching: RoMa] Robust Feature Matching in case of different viewpoints between query image and candidate image

---

- **[Goal]**
    - 같은 공간이지만 view point가 많이 다른 2개의 이미지에 대한 robust feature matching을 할 수 있다.

---

- **[Github page]**
    
    - [GitHub - Parskatt/RoMa: [Arxiv 2023] RoMa: Revisiting Robust Losses for Dense Feature Matching](https://github.com/Parskatt/RoMa)
    

---

- **[Dependencies]**
    - `python 3.8`
    - `pytorch 2.0.1`
    - `cuda 11.7`
    - `torchvision 0.15.2`
    - `einops 0.6.1`
    - `kornia 0.7.0`
    - `xformers 0.0.21`
    - `opencv-python`
    - `matplotlib`

---

- **[Model Configuration]**
    - [https://github.com/Parskatt/RoMa/blob/main/roma/models/model_zoo/roma_models.py](https://github.com/Parskatt/RoMa/blob/main/roma/models/model_zoo/roma_models.py)
        - Code
            
            ```python
            import warnings
            import torch.nn as nn
            from roma.models.matcher import *
            from roma.models.transformer import Block, TransformerDecoder, MemEffAttention
            from roma.models.encoders import *
            
            def roma_model(resolution, upsample_preds, device = None, weights=None, dinov2_weights=None, **kwargs):
                # roma weights and dinov2 weights are loaded seperately, as dinov2 weights are not parameters
                torch.backends.cuda.matmul.allow_tf32 = True # allow tf32 on matmul
                torch.backends.cudnn.allow_tf32 = True # allow tf32 on cudnn
                warnings.filterwarnings('ignore', category=UserWarning, message='TypedStorage is deprecated')
                gp_dim = 512
                feat_dim = 512
                decoder_dim = gp_dim + feat_dim
                cls_to_coord_res = 64
                coordinate_decoder = TransformerDecoder(
                    nn.Sequential(*[Block(decoder_dim, 8, attn_class=MemEffAttention) for _ in range(5)]), 
                    decoder_dim, 
                    cls_to_coord_res**2 + 1,
                    is_classifier=True,
                    amp = True,
                    pos_enc = False,)
                dw = True
                hidden_blocks = 8
                kernel_size = 5
                displacement_emb = "linear"
                disable_local_corr_grad = True
                
                conv_refiner = nn.ModuleDict(
                    {
                        "16": ConvRefiner(
                            2 * 512+128+(2*7+1)**2,
                            2 * 512+128+(2*7+1)**2,
                            2 + 1,
                            kernel_size=kernel_size,
                            dw=dw,
                            hidden_blocks=hidden_blocks,
                            displacement_emb=displacement_emb,
                            displacement_emb_dim=128,
                            local_corr_radius = 7,
                            corr_in_other = True,
                            amp = True,
                            disable_local_corr_grad = disable_local_corr_grad,
                            bn_momentum = 0.01,
                        ),
                        "8": ConvRefiner(
                            2 * 512+64+(2*3+1)**2,
                            2 * 512+64+(2*3+1)**2,
                            2 + 1,
                            kernel_size=kernel_size,
                            dw=dw,
                            hidden_blocks=hidden_blocks,
                            displacement_emb=displacement_emb,
                            displacement_emb_dim=64,
                            local_corr_radius = 3,
                            corr_in_other = True,
                            amp = True,
                            disable_local_corr_grad = disable_local_corr_grad,
                            bn_momentum = 0.01,
                        ),
                        "4": ConvRefiner(
                            2 * 256+32+(2*2+1)**2,
                            2 * 256+32+(2*2+1)**2,
                            2 + 1,
                            kernel_size=kernel_size,
                            dw=dw,
                            hidden_blocks=hidden_blocks,
                            displacement_emb=displacement_emb,
                            displacement_emb_dim=32,
                            local_corr_radius = 2,
                            corr_in_other = True,
                            amp = True,
                            disable_local_corr_grad = disable_local_corr_grad,
                            bn_momentum = 0.01,
                        ),
                        "2": ConvRefiner(
                            2 * 64+16,
                            128+16,
                            2 + 1,
                            kernel_size=kernel_size,
                            dw=dw,
                            hidden_blocks=hidden_blocks,
                            displacement_emb=displacement_emb,
                            displacement_emb_dim=16,
                            amp = True,
                            disable_local_corr_grad = disable_local_corr_grad,
                            bn_momentum = 0.01,
                        ),
                        "1": ConvRefiner(
                            2 * 9 + 6,
                            24,
                            2 + 1,
                            kernel_size=kernel_size,
                            dw=dw,
                            hidden_blocks = hidden_blocks,
                            displacement_emb = displacement_emb,
                            displacement_emb_dim = 6,
                            amp = True,
                            disable_local_corr_grad = disable_local_corr_grad,
                            bn_momentum = 0.01,
                        ),
                    }
                )
                kernel_temperature = 0.2
                learn_temperature = False
                no_cov = True
                kernel = CosKernel
                only_attention = False
                basis = "fourier"
                gp16 = GP(
                    kernel,
                    T=kernel_temperature,
                    learn_temperature=learn_temperature,
                    only_attention=only_attention,
                    gp_dim=gp_dim,
                    basis=basis,
                    no_cov=no_cov,
                )
                gps = nn.ModuleDict({"16": gp16})
                proj16 = nn.Sequential(nn.Conv2d(1024, 512, 1, 1), nn.BatchNorm2d(512))
                proj8 = nn.Sequential(nn.Conv2d(512, 512, 1, 1), nn.BatchNorm2d(512))
                proj4 = nn.Sequential(nn.Conv2d(256, 256, 1, 1), nn.BatchNorm2d(256))
                proj2 = nn.Sequential(nn.Conv2d(128, 64, 1, 1), nn.BatchNorm2d(64))
                proj1 = nn.Sequential(nn.Conv2d(64, 9, 1, 1), nn.BatchNorm2d(9))
                proj = nn.ModuleDict({
                    "16": proj16,
                    "8": proj8,
                    "4": proj4,
                    "2": proj2,
                    "1": proj1,
                    })
                displacement_dropout_p = 0.0
                gm_warp_dropout_p = 0.0
                decoder = Decoder(coordinate_decoder, 
                                  gps, 
                                  proj, 
                                  conv_refiner, 
                                  detach=True, 
                                  scales=["16", "8", "4", "2", "1"], 
                                  displacement_dropout_p = displacement_dropout_p,
                                  gm_warp_dropout_p = gm_warp_dropout_p)
                
                encoder = CNNandDinov2(
                    cnn_kwargs = dict(
                        pretrained=False,
                        amp = True),
                    amp = True,
                    use_vgg = True,
                    dinov2_weights = dinov2_weights
                )
                h,w = resolution
                symmetric = True
                attenuate_cert = True
                matcher = RegressionMatcher(encoder, decoder, h=h, w=w, upsample_preds=upsample_preds, 
                                            symmetric = symmetric, attenuate_cert=attenuate_cert, **kwargs).to(device)
                matcher.load_state_dict(weights)
                return matcher
            ```
            

---

- **[Get Pre-trained Model]**
    - 자동적으로 URL을 통해서 다운 받게 coding 되어 있지만 수동적으로 해당 모델을 다운 받아서 사용할 수 있다.
        - **roma/models/model_zoo/__init__.py**
            - roma_indoor model: **https://github.com/Parskatt/storage/releases/download/roma/roma_indoor.pth**
            - roma_outdoor model: **https://github.com/Parskatt/storage/releases/download/roma/roma_outdoor.pth**
            - DINOv2 model: **https://dl.fbaipublicfiles.com/dinov2/dinov2_vitl14/dinov2_vitl14_pretrain.pth**

---

- **[Basic Code]**
    - **demo folder에 작성되어 있는 예시 기반 code**
        
        ```python
        from roma import roma_indoor, roma_outdoor
        from PIL import Image
        import numpy as np
        import PIL
        import torch
        import pdb
        import triton
        import cv2
        
        # ====================================================================================================== # 
        def draw_keypoints_on_image(image, keypoints, color='blue', radius=2, use_normalized_coordinates=False):
          """Draws keypoints on an image.
        
          Args:
            image: a PIL.Image object.
            keypoints: a numpy array with shape [num_keypoints, 2].
            color: color to draw the keypoints with. Default is red.
            radius: keypoint radius. Default value is 2.
            use_normalized_coordinates: if True (default), treat keypoint values as
              relative to the image.  Otherwise treat them as absolute.
          """
          draw = PIL.ImageDraw.Draw(image)
          im_width, im_height = image.size
          keypoints_x = [k[1] for k in keypoints]
          keypoints_y = [k[0] for k in keypoints]
          if use_normalized_coordinates:
            keypoints_x = tuple([im_width * x for x in keypoints_x])
            keypoints_y = tuple([im_height * y for y in keypoints_y])
          for keypoint_x, keypoint_y in zip(keypoints_x, keypoints_y):
            draw.ellipse([(keypoint_x - radius, keypoint_y - radius),
                          (keypoint_x + radius, keypoint_y + radius)],
                         outline=color, fill=color)
        # ====================================================================================================== #
        # ====================================================================================================== # 
        def draw_line_on_image(image, kptsA, kptsB, color='red', size=0):
          draw = PIL.ImageDraw.Draw(image)
          length = kptsA.shape[0]
          for i in range(length):
            correspondence = [(kptsA[i][0], kptsA[i][1]), (kptsB[i][0], kptsB[i][1])]
            draw.line(correspondence, fill=color, width=size)
        # ====================================================================================================== # 
        
        # ============================================= MAIN =================================================== # 
        # ================= Set image path and cuda device ================= #
        query_path = "~/src/RoMa/query.png"
        cand_path = "~/src/RoMa/cand.png"
        
        device = "cuda"
        
        # ====================== Create RoMa Model ====================== #
        # Create Model 
        roma_model = roma_indoor(device=device)
        
        # ====================== Get Output Resolution ====================== #
        # Output: 560 x 560  
        H, W = roma_model.get_output_resolution() 
        
        # ============ Change image size to fit output resolution ============ #
        im1 = Image.open(query_path).resize((W, H))
        im2 = Image.open(cand_path).resize((W, H))
        
        # ============ Create image to show the correspondence pairs  ============ #
        # Create a new output image that concatenates the two images together
        output_img = Image.new("RGB", (im1.width + im2.width, im1.height))
        output_img.paste(im1, (0, 0))
        output_img.paste(im2, (im1.width, 0))
        
        # ====================== Match Two Images ====================== #
        # Get warp and convariance (certainty)
        # warp size: torch.Size([560, 1120, 4]) & type: <class 'torch.Tensor'>
        # certainty size: torch.Size([10000]) & type: <class 'torch.Tensor'>
        warp, certainty = roma_model.match(query_path, cand_path, device=device)
        
        # ====================== Match Two Images ====================== #
        # Sample matches for estimation
        # matches size: torch.Size([10000, 4]) & type: <class 'torch.Tensor'>
        matches, certainty = roma_model.sample(warp, certainty)
        
        # ====================== Get Feature Points ====================== #
        # Convert to pixel coordinates (RoMa produces matches in [-1,1]x[-1,1])
        # kptsA size: torch.Size([10000, 2]) & type: <class 'torch.Tensor'>
        # kptsB size: torch.Size([10000, 2]) & type: <class 'torch.Tensor'>
        kptsA, kptsB = roma_model.to_pixel_coordinates(matches, H, W, H, W)
        
        # ====================== Get Feature Points ====================== #
        # Find a fundamental matrix (or anything else of interest)
        F, mask = cv2.findFundamentalMat(
            kptsA.cpu().numpy(), kptsB.cpu().numpy(), ransacReprojThreshold=0.2, method=cv2.USAC_MAGSAC, confidence=0.999999, maxIters=10000
        )
        
        # ================= Convert PIL image channel to RGB  ================= #
        image1_pil = Image.fromarray(np.uint8(im1)).convert('RGB')
        image2_pil = Image.fromarray(np.uint8(im2)).convert('RGB')
        
        # ================= Select inlier points  ================= #
        # We select only inlier points
        kptsA = kptsA[mask.ravel()==1]
        kptsB = kptsB[mask.ravel()==1]
        
        # ================= Draw Feature Points  ================= #
        draw_keypoints_on_image(image1_pil, np.array(kptsA.cpu()))
        draw_keypoints_on_image(image2_pil, np.array(kptsB.cpu()))
        
        # use numpy to convert the pil_image into a numpy array
        numpy_image1 = np.array(image1_pil)  
        numpy_image2 = np.array(image2_pil)  
        
        # convert to a openCV2 image and convert from RGB to BGR format
        cv_image1 = cv2.cvtColor(numpy_image1, cv2.COLOR_RGB2BGR)
        cv_image2 = cv2.cvtColor(numpy_image2, cv2.COLOR_RGB2BGR)
        
        # ================= Move feature points for plotting correspondence pair  ================= #
        # Get each image row & column
        rows1, cols1 = cv_image1.shape[:2]
        rows2, cols2 = cv_image2.shape[:2]
        kptsC = np.array(kptsB.cpu()) + [cols1, 0]
        
        # ================= Draw Matching Pairs  ================= #
        # Draw matching pair
        draw_line_on_image(output_img, np.array(kptsA.cpu()), kptsC)
        output_img.show()
        
        ```
        
    - Reference Site
        
        - [https://github.com/datitran/object_detector_app/blob/master/object_detection/utils/visualization_utils.py](https://github.com/datitran/object_detector_app/blob/master/object_detection/utils/visualization_utils.py)
        
        - [Python과 OpenCV – 45 : 등극선 기하(Epipolar Geometry)](http://www.gisdeveloper.co.kr/?p=6922)
        
        - [Python OpenCV 와  PIL 의 상호 변환](https://www.zinnunkebi.com/python-opencv-pil-convert/)
        
        - [Python PIL | ImageDraw.Draw.line() - GeeksforGeeks](https://www.geeksforgeeks.org/python-pil-imagedraw-draw-line/)
        
        - [Concatenate images with Python, Pillow | note.nkmk.me](https://note.nkmk.me/en/python-pillow-concat-images/)
        

---

- **[Experiments]**
    - View point가 다른 challenging 한 이미지 pair 3쌍을 준비
        - Test Pair 1
            <table>
            <tr>
                <td> Query Image </td>
                <td> Candidate Image </td>
            </tr> 
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 1.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
           
            
        - Test Pair 2 (Challenging !)
            <table>
            <tr>
                <td> Query Image </td>
                <td> Candidate Image </td>
            </tr> 
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 2.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 3.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
            

        - Test Pair 3
            <table>
            <tr>
                <td> Query Image </td>
                <td> Candidate Image </td>
            </tr> 
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 4.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 5.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
                        

---

- **[Result]**
    - 주어진 desktop 환경은 **NVIDIA RTX 3060 이 탑재되어 있고 RoMa를 돌리면 약 5.5GB 정도의 GPU memory 소모 (좀 무거운듯?)**
    - RoMa에서 주어진 **pre-trained model을 적용하여 해당 결과 plot** (본 데이터셋으로 추가 train 시키지 않음!)
    - Not Change Output Resolution: 기본 이미지 사이즈로 feature matching 진행하는 경우
        - Keypoint Detection
            <table>
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 6.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 7.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
                        
        - Inlier Keypoint using cv2.findFundamentalMat
            <table>
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 8.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 9.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
            
        - Feature Matching Result
            - Draw All inliers
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 10.png" alt="Candidate Image">
                </figure> 
                
            - Draw 50 inlier pairs
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 11.png" alt="Candidate Image">
                </figure> 
                
            - Draw 100 inlier pairs
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 12.png" alt="Candidate Image">
                </figure> 
                
    - Change Output Resolution: RoMa에서 제공한 output resolution으로 이미지 size를 변경 후 matching 진행하는 경우
        - Keypoint Detection
            <table>
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 13.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 14.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
            
        - Inlier Keypoint using cv2.findFundamentalMat
            <table>
            <tr>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 15.png" alt="">
                </figure> 
            </td>
            <td>
                <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 16.png" alt="">
                </figure> 
            </td>
            </tr>
            </table>
                        
        - Feature Matching Result
            - Draw All inliers
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 17.png" alt="Candidate Image">
                </figure>     
                
            - Draw 50 inlier pairs
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 18.png" alt="Candidate Image">
                </figure>                 
                
            - Draw 100 inlier pairs
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 19.png" alt="Candidate Image">
                </figure> 
                
---

- **[Ablation Study]**
    - Not change image size !
    - **[Case 1] ransacReprojThreshold=0.2**
        <table>
        <tr>
            <td> # of inliers: 2978 </td>
            <td> # of inliers: 512 </td>
            <td> # of inliers: 697 </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 20.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 21.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 22.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>

        
    - **[Case 2] ransacReprojThreshold=0.1**
        <table>
        <tr>
            <td> # of inliers: 1596 </td>
            <td> # of inliers: 312 </td>
            <td> # of inliers: 323 </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 23.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 24.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 25.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>
      
        
    - **[Case 3] ransacReprojThreshold=0.05**
        <table>
        <tr>
            <td> # of inliers: 855 </td>
            <td> # of inliers: 158 </td>
            <td> # of inliers: 175 </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 26.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 27.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 28.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>
        
        
    - **[Case 4] ransacReprojThreshold=0.01**
        <table>
        <tr>
            <td> # of inliers: 146 </td>
            <td> # of inliers: 31 </td>
            <td> # of inliers: 25 </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 29.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 30.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 31.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>


    - **[Case 5] ransacReprojThreshold=0.005**
        <table>
        <tr>
            <td> # of inliers: 89 </td>
            <td> # of inliers: 15 </td>
            <td> # of inliers: 19 </td>
        </tr> 
        <tr>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 32.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 33.png" alt="">
            </figure> 
        </td>
        <td>
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 34.png" alt="">
            </figure> 
        </td>
        </tr>
        </table>


    - **[Case 6] ransacReprojThreshold=0.001**
        - **# of inliers: 18**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/roma/Untitled 35.png" alt="">
            </figure> 
            
        - **2번째 test pair는 matching pair가 없음 !!!**
        - **3번째 test pair도 matching pair가 없음 !!!**
    
    ---
    
    - **TEST PAIR 2번째가 매우 challenging한데… 역시 해당 모델도 feature matching이 쉽지 않네요…ㅎㅎㅎ**
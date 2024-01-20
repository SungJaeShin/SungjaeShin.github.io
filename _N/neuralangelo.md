---
title: "[Neuralangelo] Run Neuarlangelo using own dataset and extract mesh in anaconda environment ! (CVPR 2023)"
excerpt: "Run Neuralangelo using conda env"
---
# [Neuralangelo] Run Neuarlangelo using own dataset and extract mesh in anaconda environment ! (CVPR 2023)

---

- **[Goal]**
    - Neuralangelo를 직접 취득한 데이터 기반으로 돌려보고 해당 환경의 정보를 mesh화 하여 얻을 수 있다.

---

- **[Brief Comment]**
    - 작성 당시 neural-based 3D mesh generation 하는 논문 중 가장 성능이 높은 (가장 dense한) mesh generation 할 수 있는 paper이고 open-source 이기 때문에 선정

---

- **[Neuralangelo github]**
    
    - [GitHub - NVlabs/neuralangelo: Official implementation of "Neuralangelo: High-Fidelity Neural Surface Reconstruction" (CVPR 2023)](https://github.com/nvlabs/neuralangelo)
    

---

- **[How to install?]**
    - Step 0. Install Basic Anaconda Environment
        - 기본적인 환경을 설치
            - **Neuralangelo에서 제공하는 conda `neuralangelo.yaml` file이 있어 바로 이를 create 할 수 있도록 만들어두었지만, 그렇게 설치를 하면 cuda version과 cudatoolkit version이 latest version에 fit되어 설치가 되기 때문에 서로 안맞을 수 있음!**
                - **예를 들면 그냥 위의 방식으로 철치하면 cudatoolkit은 11.8 로 cudatoolkit-dev는 11.7로 설치되는 문제가 발생 !**
            - 그래서 본 저자는 해당 code가 돌아갈수 있는 stable한 torch & torchvision & cuda & cudnn을 먼저 다운 받고 설치해준다!
                - 해당 site에서 stable한 파일을 다운로드!!
                    
                    - [https://download.pytorch.org/whl/torch_stable.html](https://download.pytorch.org/whl/torch_stable.html)
                    
                - (basic) 기본적인 환경 version !!
                    - **`pytorch 2.0.1`**
                    - **`torchvision 0.15.2`**
                    - **`cuda 11.7`**
                    - **`cudnn 8.5.0`**
            - 최종적인 환경 setting 절차는 다음과 같음
                
                ```bash
                # Conda 환경 설치 (with python version 3.8)
                $ conda create -n neuralangelo python=3.8
                $ conda activate neuralangelo
                
                # [그냥 참고] pytorch 2.0.0 / torchvision 0.15.1 이렇게가 맞는 버전 !!
                **# pytorch 2.0.1 / torchvision 0.15.2 / cuda 11.7 버전으로 설치 !!**
                $ pip3 install torch-2.0.1+cu117.with.pypi.cudnn-cp38-cp38-linux_x86_64.whl
                $ pip3 install torchvision-0.15.2+cu117-cp38-cp38-linux_x86_64.whl
                
                # 나머지 dependencies 들은 pip3로 모두 설치 !!
                $ pip3 install gpustat
                $ pip3 install gdown
                $ pip3 install cmake
                $ pip3 install scipy
                $ pip3 install ipython
                $ pip3 install jupyterlab
                $ pip3 install cython
                $ pip3 install ninja
                $ pip3 install diskcache
                
                # 그리고 나머지 dependencies 들은 requirements.txt에 있음 !!
                **# [중요!!!] git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch 를 지우고 나머지 설치 !!!**
                $ pip3 install -r requirements.txt
                
                # 마지막 cudatoolkit-dev 설치 ! 
                **# (cudatoolkit-dev가 11.8을 작성 당시에는 제공하지 않으므로 11.7로 fit하게 환경을 설정하였음 !)**
                $ conda install -c conda-forge cudatoolkit-dev=11.7 
                ```
                
    
    ---
    
    - Step 1. Clone Repo with submodules
        - 해당 workspace에 github code clone
            
            ```bash
            $ git clone https://github.com/NVlabs/neuralangelo.git
            ```
            
        - **tiny-cuda-nn의 package는 따로 github에 가서 clone을 진행하여 수동 설치를 진행해야함! (에러를 해결하기 위해!!)**
            - github site
                
                - [GitHub - NVlabs/tiny-cuda-nn: Lightning fast C++/CUDA neural network framework](https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch)
                
                ```bash
                $ cd neuralangelo/third_party
                $ git clone --recursive https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
                ```
                
    
    ---
    
    - Step 2. Install tiny-cuda-nn package
        - neuralangelo github page처럼 `pip3 install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch` 로 해버리면 error가 발생하기 때문에 수동 설치를 진행 !
            
            **[1]** conda 기반으로 설치를 하였기 때문에 해당 경로를 export하여 지정해줄 필요가 있음
            
            ```bash
            $ export PATH="/home/sj/anaconda3/envs/neuralangelo/pkgs:$PATH"
            $ export LD_LIBRARY_PATH="/home/sj/anaconda3/envs/neuralangelo/lib:$LD_LIBRARY_PATH"
            ```
            
            **[2]** tiny-cuda-nn 을 설치해줄 python code로 이동
            
            ```bash
            $ cd tiny-cuda-nn/bindings/torch
            ```
            
            **[3] [setup.py](https://github.com/NVlabs/tiny-cuda-nn/blob/master/bindings/torch/setup.py)** 로 들어가서 마지막 코드 수정 (에러를 방지하기 위해)
            
            ```python
            # Before
            cmdclass={"build_ext": BuildExtension}
            
            # After
            cmdclass={'build_ext': BuildExtension.with_options(use_ninja=False)}
            ```
            
            **[4]** 그리고 수정된 [**setup.py**](https://github.com/NVlabs/tiny-cuda-nn/blob/master/bindings/torch/setup.py)를 install **(시간 조금 소요됨 !)**
            
            ```bash
            $ pip3 setup.py install
            ```
            
        
        ---
        
        - **[Error 1]**
            - `RuntimeError: Error compiling objects for extension`
        
        ---
        
        - **[Solve 1]**
            - setup.py로 들어가서 마지막 코드 수정
                
                ```python
                # Before
                cmdclass={"build_ext": BuildExtension}
                
                # After
                cmdclass={'build_ext': BuildExtension.with_options(use_ninja=False)}
                ```
                
        
        ---
        
        - **[Reference Site 1]**
            
            - [I cannot install pytorch extension on windows · Issue #195 · NVlabs/tiny-cuda-nn](https://github.com/NVlabs/tiny-cuda-nn/issues/195#issuecomment-1316275803)
            
        
        ---
        
        - **[Error 2]**
            - `fatal error: filesystem: No such file or directory`
        
        ---
        
        - **[Solve 2]**
            - tiny-cuda-nn의 package는 c++ 17 version을 요구 (gcc & g++ 기준 8.x.x 이상) 하고 있는데 본 저자의 desktop에 설치되어 있는 저자의 gcc version은 7.5.0이므로 본 gcc 버전을 upgrade 및 main version으로 지정
                
                [1] gcc version 확인
                
                ```bash
                $ gcc --version
                ```
                
                [2] gcc 버전이 8.x.x 보다 작으면 해당 버전으로 추가 설치
                
                ```bash
                $ sudo apt install gcc-8 g++-8
                ```
                
                [3] 최신 버전의 gcc를 default로 설정
                
                ```bash
                $ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
                $ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8
                ```
                
                [4] 변경된 default gcc 버전 확인
                
                ```bash
                $ gcc --version
                ```
                
        
        ---
        
        - **[Reference Site 2]**
            
            - [[해결법] fatal error: filesystem: No such file or directory](https://jtrimind.github.io/troubleshooting/filesystem/)
            
            - [[Solved] Failed to build tinycudann; Could not build wheels for tinycudann; Could not find filesystem; xxx.so.xx no such file or directory · Issue #337 · NVlabs/tiny-cuda-nn](https://github.com/NVlabs/tiny-cuda-nn/issues/337#issuecomment-1632178963)
            
    
    ---
    
    - Step 3. Make Neuralangelo input dataset
        - 해당 github code에서도 video나 이미지들을 colmap 기반으로 형식으로 변경해주는 code를 제공
            - **Neuralangelo에서 요구하는 input dataset 구조 및 이름 (파일 이름도 맞춰주기!)**
                
                ```
                DATA_PATH
                ├─ database.db      (COLMAP database)
                ├─ images           (undistorted input images)
                ├─ images_raw       (raw input images)
                ├─ sparse           (COLMAP data from SfM)
                │  ├─ cameras.bin   (camera parameters)
                │  ├─ images.bin    (images and camera poses)
                │  ├─ points3D.bin  (sparse point clouds)
                │  ├─ 0             (a directory containing individual SfM models. There could also be 1, 2... etc.)
                │  ...
                ├─ stereo (COLMAP data for MVS, not used here)
                │
                ├─ transforms.json  (Relative camera poses)
                ├─ {SEQUENCE}.yaml  (Hyperparameters config file)
                ```
                
            - 한번에 요구하는 dataset 구조로 만들어주는 bash file도 있고 순차적으로 만들어주는 bash file도 제공하고 있음 !!
                - **(본인 생각) 개인적으로 instant-ngp 보다 더 편리하게 변환할 수 있어 좋은 것 같음 !!**
                    - **1분짜리 영상에 downsampling_rate를 2로 두면 약 800장 이상의 이미지들이 생성되고 이들을 colmap으로 돌리면 엄청 오랜 시간이 걸렸음…. ㅎㅎ (물론 해당 영상의 환경이 challenging 하기 때문인 것도 있음!)**
                - Default Setting !
                    
                    ```bash
                    SEQUENCE=lego # your custom name for the video sequence.
                    PATH_TO_VIDEO=lego.mp4 # absolute/relative path to your video.
                    DOWNSAMPLE_RATE=2 # temporal downsampling rate of video sequence (for extracting video frames).
                    SCENE_TYPE=object # {outdoor,indoor,object} 중 하나! 
                    ```
                    
                - **[Case 1]** 한번에 변환해주는 shell file !!
                    
                    ```bash
                    # End-to-end dataset generation !!
                    sh ./projects/neuralangelo/scripts/preprocess.sh ${SEQUENCE} ${PATH_TO_VIDEO} ${DOWNSAMPLE_RATE} ${SCENE_TYPE}
                    ```
                    
                    - **[Sample] 본 저자의 custom dataset으로 colmap 만든 결과**
                        
                        ```yaml
                        # 1분짜리 / downsample rate 2로 두고 colmap end-to-end로 돌리면 약 887장 이미지 생성 + 약 6시간 정도 걸림
                        # **B301.yaml file도 만들어줌**! -> 이거 가지고 train.py에 이용하면 됨!!
                        # [Example of b301.yaml]
                        _parent_: projects/neuralangelo/configs/base.yaml
                        data:
                            readjust:
                                center:
                                - 0.0
                                - 0.0
                                - 0.0
                                scale: 1.0
                            root: datasets/b301_ds2
                            train:
                                image_size:
                                - 1065
                                - 1894
                            type: projects.neuralangelo.data
                            val:
                                image_size:
                                - 300
                                - 533
                        model:
                            appear_embed:
                                enabled: false
                            background:
                                enabled: false
                            object:
                                sdf:
                                    encoding:
                                        coarse2fine:
                                            init_active_level: 8
                                    mlp:
                                        inside_out: true
                            render:
                                num_samples:
                                    background: 0
                        ```
                        
                - **[Case 2]** 순차적으로 변환해주는 shell file !! (보고 없는 파일들만 생성하면 좋을듯!!)
                    
                    ```bash
                    # 이미지 생성만 !
                    sh projects/neuralangelo/scripts/run_ffmpeg.sh ${SEQUENCE} ${PATH_TO_VIDEO} ${DOWNSAMPLE_RATE}
                    
                    # 이미지들을 이용하여 COLMAP 진행 !
                    DATA_PATH=datasets/${SEQUENCE}_ds${DOWNSAMPLE_RATE}
                    sh projects/neuralangelo/scripts/run_colmap.sh ${DATA_PATH}
                    
                    # COLMAP 결과를 이용하여 JSON file 생성 !
                    python3 ./projects/neuralangelo/scripts/convert_data_to_json.py --data_dir ${DATA_PATH} --scene_type ${SCENE_TYPE}
                    
                    # 위의 결과들을 이용하여 config file 생성 !
                    python3 ./projects/neuralangelo/scripts/generate_config.py --sequence_name ${SEQUENCE} --data_dir ${DATA_PATH} --scene_type ${SCENE_TYPE}
                    ```
                    
        
        ---
        
        - **[Reference Site]**
            
            - [https://github.com/NVlabs/neuralangelo/blob/main/DATA_PROCESSING.md](https://github.com/NVlabs/neuralangelo/blob/main/DATA_PROCESSING.md)
            
    
    ---
    
    - Step 4. Train Neuralangelo !!
        - 앞서 취득된 dataset으로 neuralangelo를 training 진행 ! **(처음 training한 dataset 기준: 약 6시간 30분정도 걸렸음)**
            - Default Setting
                
                ```bash
                EXPERIMENT=toy_example
                GROUP=example_group
                NAME=example_name
                CONFIG=projects/neuralangelo/configs/custom/${EXPERIMENT}.yaml
                GPUS=1  # use >1 for multi-GPU training!
                ```
                
            - Training
                
                ```bash
                torchrun --nproc_per_node=${GPUS} train.py \
                			   --logdir=logs/${GROUP}/${NAME} \
                		     --config=${CONFIG} \
                		     --show_pbar
                ```
                
                - **[Tip]** 해당 github page에서 말하길 paper는 NVIDIA V100 24GB GPU로 training을 시켰는데 해당 gpu보다 성능이 낮으면 out of memory 가 발생할 것이기 때문에 training하기 전에 hyperparameter를 조절해주어야 함!!
                    - **본 저자의 GPU는 NVIDIA RTX 3060 (12GB) 이기 때문에 dict_size=21, dim=4 로 setting !**
                    - 변경할 hyperparameter 위치 → **neuralangelo/projects/neuralangelo/configs/base.yaml**
                    - 변경할 hyperparameter 변수 → `**model.object.sdf.encoding.hashgrid**`
                        
                        
                        | GPU VRAM | Hyperparamter |
                        | --- | --- |
                        | 8GB | dict_size=20, dim=4 |
                        | 12GB | dict_size=21, dim=4 |
                        | 16GB | dict_size=21, dim=8 |
                    - Reference Site
                        
                        - [GitHub - NVlabs/neuralangelo: Official implementation of "Neuralangelo: High-Fidelity Neural Surface Reconstruction" (CVPR 2023)](https://github.com/nvlabs/neuralangelo#:~:text=GPU%20VRAM,%2C%20dim%3D8)
                        
                
                ---
                
                - **[Tip 2]** custom dataset으로 training을 하여도 parent 에 해당하는 yaml file도 포함할 수 있도록 설정하기 때문에 parent에 해당하는 base.yaml 에서 --max_iter을 설정할 수 있음 !!
                    - **기본적인 setting 은 `max_iter: 500000` 로 되어 있어 본 저자는 `200000`로 setting !**
        
        ---
        
        - Training 결과 (Batch size: 1)
            - Training dataset: 887장 & Validation dataset: 4장
            - **Max iteration: 200000**
            - **총 training에 걸리는 시간: 약 22585초 = 약 6시간 30분**
            - **Epoch 당 걸리는 시간 평균: 약 50초**
            - **GPU Memory 소모: 평균 11.5GB**
            - Epoch 11번 당 1번꼴로 evaluation 진행
            - Wandb로 나중에 추가해서 또 보자!! (처음에는 못해봤음!)
            - 저장된 결과물들
                - 저장 위치: **logs/${GROUP}/${NAME}**
                - **config.yaml & checkpoint.pt (개당 1.2G정도) & latest_checkpoint.txt & wandb_id.txt**
    
    ---
    
    - Step 5: Extract Mesh from train model !!
        - 앞서 취득된 dataset으로 train 시킨 모델을 이용하여 mesh를 생성 !
            - Default Setting
                
                ```bash
                CHECKPOINT=logs/${GROUP}/${NAME}/epoch_00451_iteration_000200000_checkpoint.pt # exmaple
                OUTPUT_MESH=xxx.ply
                CONFIG=logs/${GROUP}/${NAME}/config.yaml
                RESOLUTION=2048
                BLOCK_RES=128
                GPUS=1  # use >1 for multi-GPU mesh extraction
                ```
                
            - Extract Meshing
                
                ```bash
                torchrun --nproc_per_node=${GPUS} projects/neuralangelo/scripts/extract_mesh.py \
                    --config=${CONFIG} \
                    --checkpoint=${CHECKPOINT} \
                    --output_file=${OUTPUT_MESH} \
                    --resolution=${RESOLUTION} \
                    --block_res=${BLOCK_RES} \
                		--textured # extract meshes with textures !!
                ```
                
                - **[Tip]** --textured 를 넣으면 mesh를 extract 진행할 때 texture까지 같이 넣어 ply file로 생성 **(textured가 추가가 되면 color로 mesh generation이 가능해짐!!!!!)**
        
        ---
        
        - Extract 결과
            - 1.2G 짜리 train 된 model을 mesh extract 진행
            - **GPU Memory 소모: 평균 8GB**
            - Allow TensorFloat32 operations on supported devices
            - Extracting surface at resolution 2575 1691 3072
            - **vertices: 41071121 & faces: 80835928 & colors: 41071121**
            - 저장된 결과물들
                - 저장 위치: **./neuralangelo/xxx.ply**
                - **xxx.ply (위의 데이터셋을 이용한 결과 1.7G 정도 되는 mesh file이 생성됨)**

---

- **[Results]**
    - DATASET 3으로 취득한 결과로 얻어진 mesh 결과는 다음과 같음
        - PLY pointcloud Visualization Tool
            - open3d: `pip3 install open3d`
        
        ---
        
        - **Visualization base code**
            
            ```python
            from open3d import *    
            
            def main():
                ply_path = "./xxx.ply" 
                cloud = io.read_point_cloud(ply_path) # Read point cloud
                visualization.draw_geometries([cloud])    # Visualize point cloud      
            
            if __name__ == "__main__":
                main()
            ```
            
        
        ---
        
        - Image Result w/ texture
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled.png" alt="">
            </figure> 
            
        - Result w/ texture (B301 DATASET3)
            <figure class="align-center">
                <video src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled.mp4" alt="">
            </figure> 
                    
        ---
        
        - Result w/o texture (B301 DATASET3)
            <figure class="align-center">
                <video src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 1.mp4" alt="">
            </figure>             
        
        ---
        
        - Reference Site
            
            - [Python - Display 3D Point Cloud](https://stackoverflow.com/questions/50965673/python-display-3d-point-cloud)
            
    
    ---
    
    - DATASET 5으로 train 시킨 경우
        - 위의 config 인 `dict_size=21, dim=4` 로 train 시키면 Out of Memory 가 발생
        - 그래서 이 경우 (총 train dataset이 3855 장) 8GB GPU VRAM case로 두고 train 진행
            - **[Train]**
                - Max iteration: 380000
                - Epoch 당 train에 걸리는 시간 약 90초
                - 평균 GPU 소모: 약 10.5GB
                - 총 학습에 걸리는 시간: **약 9.5시간**
            - **[Mesh]**
                - 하아.. 데탑 용량이 없어서 다 iteration 돌지 못하고 중간에 멈춰버림…
                - 그래서 가장 최신의 checkpoint model을 이용하여 mesh extraction 진행
                    - extraction 진행한 모델: epoch_00363_iteration_000380000_checkpoint.pt
                        - Extracting surface at resolution 2094 789 3072
                        - **vertices: 42115776 & faces: 82877690 &colors: 42115776**
        
        ---
        
        - Image Result w/ texture
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 1.png" alt="">
            </figure> 
            
        - Result w/ texture (B401 DATASET 5)
            <figure class="align-center">
                <video src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 2.mp4" alt="">
            </figure>         
            
            - 확실히 전보다는 dense 하게 reconstruction 진행할 수 있었으나 조금 더 진행할 필요가 있어보임
    
    ---
    
    - DATASET 5으로 train 시킨 경우 2번째 진행
        - **[Train]**
            - Max iteration: 500000
            - Epoch 당 train에 걸리는 시간 약 85초
            - 평균 GPU 소모: 약 11.5GB
            - 총 학습에 걸리는 시간: **약 13시간**
        - **[Mesh]**
            - model parameter count: 53,029,160
            - Extracting surface at resolution 2094 789 3072
            - **vertices: 40916810 & faces: 80505567 & colors: 40916810**
        - Image Result w/ texture
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 2.png" alt="">
            </figure> 
            
        - Result w/ texture (B401 DATASET 5)
            <figure class="align-center">
                <video src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 3.mp4" alt="">
            </figure>             

---

- **[Compare Instant-NGP]**
    - 겉으로 보기에는 매우 reconstruction 결과가 좋아보이지 않은 것을 확인하였다…
    - 그런데 Instant-NGP와 비교를 해보면 상당히 좋은 결과를 확인할 수 있었다!!
    - Image Result
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 3.png" alt="">
        </figure> 
        
    - Video Result
        <figure class="align-center">
            <video src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 4.mp4" alt="">
        </figure>  
        
---

- **Instant-NGP 의 경우 약간 artifact 해보이나 neuralangelo 의 경우 보다 realistic 한 결과를 확인할 수 있었다 !!!**

---

- **[Compare Instant-NGP using good dataset !]**
    - 좋은 데이터셋을 가지고 위 2개의 모델을 비교 및 mesh reconstruction 진행
        - **[Neuralangelo Case]**
            - **Train**
                - 공간: KAIST KI Building B503
                - 데이터셋 갯수: 687개
                - Max iteration: 500000
                - Epoch (343장) 당 train에 걸리는 시간: 약 24초
                - 평균 GPU 소모: 약 8GB
                - 총 학습에 걸리는 시간: **약 10시간**
            - **Mesh**
                - model parameter count: 53,029,160
                - Extracting surface at resolution 3073 1548 2709
                - 평균 GPU 소모: 약 7GB
                - Mesh를 추출하는데 걸리는 시간: 약 10분
                - **vertices: 36552089 & faces: 71958641 & colors: 36552089**
            - Image Result w/ texture
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 4.png" alt="">
                </figure> 
                
            - Video Result
                <figure class="align-center">
                    <video src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/neuralangelo/Untitled 5.mp4" alt="">
                </figure>  
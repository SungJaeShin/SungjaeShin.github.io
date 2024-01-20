---
title: "[NeRF-SLAM] Run NeRF-SLAM using own dataset in anaconda environment !"
excerpt: "Run NeRF-SLAM using conda env"
---
# [NeRF-SLAM] Run NeRF-SLAM using own dataset in anaconda environment !

---

- **[Goal]**
    - NeRF-SLAM을 직접 취득한 데이터 기반으로 돌려볼 수 있다.

---

- **[NeRF-SLAM Github]**
    
    - [GitHub - ToniRV/NeRF-SLAM: NeRF-SLAM: Real-Time Dense Monocular SLAM with Neural Radiance Fields. https://arxiv.org/abs/2210.13641   +   Sigma-Fusion: Probabilistic Volumetric Fusion for Dense Monocular SLAM  https://arxiv.org/abs/2210.01276](https://github.com/ToniRV/NeRF-SLAM)
    

---

- **[How to Install?]**
    - Step 0. Install Basic Anaconda Environment
        - 기본적인 환경을 설치
            
            ```bash
            # Conda 환경 설치 (with python version 3.8)
            $ conda create -n nerfslam python=3.8
            
            # Pytorch 1.12.1 / Cuda 11.3 
            # $ conda install -c pytorch pytorch=1.12.1=gpu_cuda113py38h19ae3d8_1
            
            # Pytorch 1.4.0 / Cuda 10.1 ; 이걸로 설치한 이유는... torch_scatter 때문에...
            # **해당 설치 방법은 [Run NeRF-SLAM]의 [Error 7] Part로 가서보자!**
            # $ pip3 install torch-1.4.0-cp38-cp38-linux_x86_64.whl
            
            # 하아.... 최소 cuda 버전 요건은 10.2 임... 그래서 다음과 같은 버전으로 설치!!
            # $ pip3 install torch-1.9.0+cu102-cp38-cp38-linux_x86_64.whl
            # $ pip3 install torchvision-0.10.0+cu102-cp38-cp38-linux_x86_64.whl
            
            **# 아....!!!!! 위의 것도 안됨!!! 왜냐면 cudatoolkit-dev이 제공하는 버전이 10.0, 10.1 그리고 바로 11.0 부터 진행... (10.2 버전 지원안됨...)**
            $ pip3 install torch-1.9.0+cu111-cp38-cp38-linux_x86_64.whl
            $ pip3 install torchvision-0.10.0+cu111-cp38-cp38-linux_x86_64.whl
            
            # **나머지 dependencies (미리 말하자면 conda로 설치하지 말고 pip3로 install 해야 안전함!!!)**
            $ conda install -c anaconda numpy=1.24.3=py38hf838250_0
            $ conda install -c conda-forge tqdm=4.65.0=py38hb070fc8_0
            $ conda install -c conda-forge matplotlib=3.7.1=py38h06a4308_0
            $ conda install -c open3d-admin open3d
            $ conda install -c anaconda jupyter=1.0.0=py38h06a4308_8
            $ conda install -c conda-forge imageio=2.31.1=py38h06a4308_0
            $ conda install -c conda-forge glog
            $ conda install -c conda-forge icecream
            $ conda install -c conda-forge pandas=1.5.3=py38h417a72b_0
            $ conda install -c conda-forge pybind11=2.10.4=py38hdb19cb5_0 
            $ conda install -c conda-forge gdown
            # Cuda version에 맞춰서 해당 version 설치 
            # $ conda install -c conda-forge cudatoolkit-dev=**11.3
            # 해당 버전은 10.2를 지원하지 않으므로 본 저자는 10.1.243을 설치하였음 !!!
            # Ref Site: [https://anaconda.org/conda-forge/cudatoolkit-dev/files?version=](https://anaconda.org/conda-forge/cudatoolkit-dev/files?version=)**
            $ conda install -c conda-forge cudatoolkit-dev=**11.1** 
            # 그래도 가능한지는 계속 살펴봐야함
            
            # 주의 !!!!
            $ conda install -c anaconda scipy 
            # (특정 version으로 설치하면 pytorch 가 CPU로 변경될 수 있기 때문에 주의) 
            
            # 이건 아직 설치 안함 !!!!
            # $ conda install -c conda-forge torch-scatter
            # (이거 설치는 조금 나중에 기다려보자 -> 이것도 cpu로 변경되어 설치하려고함...)
            # 이것도 밑에 설치 방법이 있는 것인데 이걸로 사용하지 말자!!! 흠.... 
            **# 해당 설치 방법은 [Run NeRF-SLAM]의 [Error 7] Part로 가서보자!**
            # $ pip3 install torch_scatter-2.0.4-cp38-cp38-linux_x86_64.whl
            
            # 해당 버전은 cuda 11.1 버전으로 설치 (10.2 버전이랑 같음!)
            $ pip3 install torch_scatter-2.0.7-cp38-cp38-linux_x86_64.whl
            ```
            
            - **[중요] For torch-scatter**
                - torch-scatter가 자꾸 pytorch를 GPU → CPU로 downgrade 시키는 이유는 다음과 같음
                    
                    (1) **pytorch version > 1.13.0 이상 (cpu & cuda 11.7 & cuda 11.8 일때만)인 경우 & pytorch 2.0 인 경우** 
                    
                    → “**`conda install torch-scatter`**” 가 가능 !!!
                    
                    (2) 그 이외의 버전이라면 (본 저자의 case인 pytorch version = 1.12.1 이고 cuda 11.3 인 경우); ***이렇게 설치를 하게 되면 밑에 [Error 7]과 같은 error가 발생하기 때문에 그냥 원인만 알아두는 것으로만 보자!*** 
                    
                    **→ “`conda install -c pyg pytorch-scatter`” 로 설치 !!!**
                    
    
    ---
    
    - Step 1. Clone Repo with submodules
        - 하위 module들까지 모두 포함된 github code clone
            
            ```bash
            $ git clone https://github.com/ToniRV/NeRF-SLAM.git --recurse-submodules
            $ git submodule update --init --recursive
            ```
            
    
    ---
    
    - Step 2. Install dependencies using anaconda
        - **“**`requirements.txt`**”** 에 있는 dependencies들을 anaconda를 이용하여 설치
            - 설치 목록
                
                (1) numpy

                (2) scipy

                (3) tqdm

                (4) matplotlib
                
                (5) open3d

                (6) opencv-python
                
                (7) torch-scatter
                
                (8) jupyter

                (9) imageio
                
                (10) glog

                (11) icecream
                
                (12) pandas

                (13) pyrealsense2
                
                (14) pybind11
                
                (15) gdown
                
                - 본 작성자는 추가로 설치했던 anaconda는 (7) & (10) & (11) & (13) & (14) 인데 **pyrealsense2는 conda에서 제공한 package가 없어 pip3 로 설치** 진행
                    
                    ```bash
                    $ pip3 install pyrealsense2
                    ```
                    
    
    ---
    
    - Step 3. Instant-NGP compile
        - NeRF-SLAM 중 rendering에 사용되었던 Instant-NGP를 compile 진행 !
            
            ```bash
            $ cmake ./thirdparty/instant-ngp -B build_ngp
            $ cmake --build build_ngp --config RelWithDebInfo -j
            ```
            
            (cmake version > 3.22) **(2번째 줄 시간이 조금 걸림 !)**
            
            ---
            
            - **[Error 1]**
                - **CMake Error at /usr/local/share/cmake-3.25/Modules/CMakeDetermineCUDACompiler.cmake:180 (message):
                Failed to find nvcc. Compiler requires the CUDA toolkit.  Please set the CUDAToolkit_ROOT variable.**
            
            ---
            
            - **[Cause 1]**
                - CudaToolkit을 이용하는 nvcc compiler가 있어야 사용할 수 있는데, (1) nvcc가 설치가 되어 있지 않거나, (2) anaconda에서 설치되는 cudatoolkit은 nvcc까지 같이 설치가 되지 않았거나, (3) 설치된 경로를 못찾았기 때문에 발생
            
            ---
            
            - **[Solve 1]**
                - Cmake를 진행할 때, “`CUDAToolkit_ROOT`”는 cmake를 위한 변수이지 그 자체가 environment가 아니므로 어디에 있는지 직접 해당 경로를 찾아서 cmake 뒤에 붙여주면 됨
                - **Anaconda를 이용하여 설치를 진행할 때, “`conda install -c anaconda cudatoolkit`” 이것 뿐만 아니라, nvcc를 사용할 수 있는 package인 “`conda install -c conda-forge cudatoolkit-dev`” 까지 꼭!! 설치해주어야 함 (설치가 조금 오래 걸림)**
                - **결국 순차적으로 하면 다음과 같이 진행하면 됨**
                    
                    ```bash
                    # Install cudatoolkit with nvcc compiler in anaconda env
                    $ conda install -c anaconda cudatoolkit=${cuda_version}
                    $ conda install -c conda-forge cudatoolkit-dev
                    
                    # Then you must see new installed package in 
                    # "anaconda3/env/{YOUR_ENV}/pkgs" !!
                    
                    # Go to clone code and put the changed cmake variables !
                    $ cmake ./thirdparty/instant-ngp -D CUDA_TOOLKIT_ROOT_DIR=/home/${COMPUTER_NAME}/anaconda3/envs/${CONDA_ENV_NAME}/pkgs -B build_ngp
                    ```
                    
            
            ---
            
            - **[Reference Site 1]**
                
                - [How to let cmake find CUDA](https://stackoverflow.com/questions/19980412/how-to-let-cmake-find-cuda)
                
                - [Where is CUDAToolKit path when installed via conda?](https://discuss.pytorch.org/t/where-is-cudatoolkit-path-when-installed-via-conda/47791/1)
                
                - [Nvcc missing when installing cudatoolkit?](https://stackoverflow.com/questions/56470424/nvcc-missing-when-installing-cudatoolkit)
                
            
            ---
            
            - **[Error 2]**
                - **“cc1plus: fatal error: cuda_runtime.h: No such file or directory”**
            
            ---
            
            - **[Solve 2]**
                - 해당 원인은 잘 확인되기 어려웠으나… 왜냐면 해당 reference site들을 보면 보통 CUDA_PATH setting관련 문제라고 하는데 본 저자는 anaconda 기반으로 environment를 생성하였기 때문에 /usr/local 에 없고 conda안에 있어서 이를 연결시키기에는 쉽지 않았음
                - **그래서 결국 무언가가 잘못 설치되었다는 것을 확인하고 다시 conda environment를 재설치 + 모든 dependencies 들을 conda install 기반으로 진행하지 않고 pip3 install 기반으로 진행 !!!**
            
            ---
            
            - **[Error 3]**
                - **CMake Error at dependencies/tiny-cuda-nn/CMakeLists.txt:148 (message): CUDA version too low.  tiny-cuda-nn require CUDA 10.2 or higher.**
            
            ---
            
            - **[Solve 3]**
                - 본 저자가 설치한 CUDA version은 10.1 이었기 때문에 해당 버전이 아닌 최소 10.2 버전이상의 cuda version 설치 !!
                - 그런데 추가로 conda에서는 cudatoolkit-dev version이 10.2를 제공하지 않기 때문에 11.1 버전을 설치 !!!
                    - **torch_scatter 의 문제도 같이 해결하기 위해서 torch version은 1.10.x 미만으로 + Cuda version 11.x로 설치 !!**
    
    ---
    
    - Step 4. GTSAM compile
        - NeRF-SLAM에서 optimization tool인 GTSAM compile 진행 !
            
            ```bash
            $ cmake ./thirdparty/gtsam -DGTSAM_BUILD_PYTHON=1 -B build_gtsam 
            $ cmake --build build_gtsam --config RelWithDebInfo -j
            $ cd build_gtsam
            $ make python-install
            ```
            
            ---
            
            - **[Error]**
                - “`cmake --build build_gtsam --config RelWithDebInfo -j`” 에 해당하는 에러 발생
                    - **[ 56%] Built target pybind_wrap_gtsam
                    [ 56%] Linking CXX shared library [libgtsam.so](http://libgtsam.so/)
                    [ 56%] Built target gtsam
                    Makefile:165: recipe for target 'all' failed
                    make: *** [all] Error 2**
            
            ---
            
            - **[Solve]**
                - NeRF-SLAM issue에 해당 문제를 다루고 있었음
                    
                    - [type object 'gtsam.gtsam.Pose3' has no attribute 'identity' · Issue #21 · ToniRV/NeRF-SLAM](https://github.com/ToniRV/NeRF-SLAM/issues/21#issuecomment-1373536626)
                    
                    - 해당 github page (https://github.com/jrpowers/NeRF-SLAM) 에서의 gtsam을 가지고 오고 다시 “`cmake --build build_gtsam --config RelWithDebInfo -j`”를 하게 되면 문제없이 build 진행
            
            ---
            
            - **[Reference Site]**
                
                - [type object 'gtsam.gtsam.Pose3' has no attribute 'identity' · Issue #21 · ToniRV/NeRF-SLAM](https://github.com/ToniRV/NeRF-SLAM/issues/21)
                
    
    ---
    
    - Step 5. Install DROID-SLAM
        - NeRF-SLAM에서 사용된 neural 기반 VO model인 DROID-SLAM을 install
            
            ```bash
            $ python3 setup.py install
            ```
            
            ---
            
            - **[Error 1]**
                - **File "/~/anaconda3/envs/~/lib/python3.8/site-packages/pkg_resources/_vendor/packaging/version.py", line 195, in init**
                    
                    **match = self._regex.search(version)
                    TypeError: expected string or bytes-like object**
                    
            
            ---
            
            - **[Solve 1]**
                - 입력하는 과정에서 string type의 변수를 넣을 때 생기는 문제로서 해당 함수에 가서 **“`str(~)`”** 이렇게 type을 작성해주어야 함
                    
                    (1) anaconda에서 에러가 발생한 python code를 찾아감
                    
                    - 해당 경로: **“/~/anaconda3/envs/~/lib/python3.8/site-packages/pkg_resources/_vendor/packaging/version.py”**
                    
                    (2) version.py안에 해당 함수를 다음과 같이 변경
                    
                    ```python
                    # Before
                    match = self._regex.search(version)
                    
                    # After (Changing !)
                    match = self._regex.search(str(version))
                    ```
                    
                    (3) 다시 “`**python3 setup.py install**`” 진행 ! 
                    
            
            ---
            
            - **[Reference Site 1]**
                
                - [setup.py install failed · Issue #20 · princeton-vl/DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM/issues/20#issuecomment-968624970)
                
            
            ---
            
            - **[Error 2]**
                - **File "/~/anaconda3/envs/~/lib/python3.8/site-packages/pkg_resources/_vendor/packaging/version.py", line 197, in init
                raise InvalidVersion(f"Invalid version: '{version}'")
                pkg_resources.extern.packaging.version.InvalidVersion: Invalid version: 'None'**
            
            ---
            
            - **[Casue 2]**
                - python package 중 setuptools version이 너무 높거나 아니면 너무 낮거나 해서 생기는 문제
            
            ---
            
            - **[Solve 2]**
                - 본 작성자의 setuptools version은 당시 최신 version (67.8.0)이기 때문에 해당 version을 downgrade 진행
                    - setuptools version을 finding하는 방법
                        
                        (1) “`conda list`” 로 확인
                        
                        (2) “`pip3 list | grep setuptools`” 로 확인 
                        
                - setuptools 중 65.6.3 version으로 설치 진행 (python 3.8 이므로 맞게 설치)
                    
                    ```bash
                    $ conda install -c conda-forge setuptools=65.6.3=py38h06a4308_0
                    ```
                    
                - 다시 “`**python3 setup.py install**`” 진행 !
            
            ---
            
            - **[Reference Site 2]**
                
                - [`InvalidVersion` exception when invalid version used on Setuptools 66 · Issue #3772 · pypa/setuptools](https://github.com/pypa/setuptools/issues/3772#issuecomment-1384342813)
                
                - [Cannot build package due to pkg_resources.extern.packaging.version.InvalidVersion - ROS Answers: Open Source Q&A Forum](https://answers.ros.org/question/411949/cannot-build-package-due-to-pkg_resourcesexternpackagingversioninvalidversion/)
                
            
            ---
            
            - **[Error 3]**
                - **File "/~/anaconda3/envs/~/lib/python3.8/site-packages/torch/utils/cpp_extension.py", line 817, in _check_cuda_version raise RuntimeError(CUDA_MISMATCH_MESSAGE.format(cuda_str_version, torch.version.cuda))
                RuntimeError: The detected CUDA version (11.7) mismatches the version that was used to compile PyTorch (None). Please make sure to use the same CUDA versions.**
            
            ---
            
            - **[Cause 3]**
                - 일단 nvcc에 작성되어 있는 cuda version과 “torch.version.cuda” 상의 version이 달라 생성된 오류
                - 해당 버전을 downgrade 진행
            
            ---
            
            - **[Solve 3]**
                - 현재 본 저자의 설치된 환경
                    
                    (1) “`nvcc -V`” 결과
                    
                    ```bash
                    $ nvcc -V
                    # Output -> *Build cuda_11.7.r11.7/compiler.32415258_0*
                    ```
                    
                    (2) “`python3`” 결과
                    
                    ```bash
                    $ python3
                    >>> import torch
                    >>> print(torch.__version__)
                    1.12.1
                    >>> print(torch.version.cuda)
                    None
                    print(torch.backends.cudnn.version())
                    None
                    ```
                    
                    - 보니 cuda가 설치되어 있지 않은 것을 확인하였다.
                        - **조금 더 자세히 살펴보니 pytorch 설치할 때 cpu_py38 버전으로 설치되었있었음!!**
                        - **그래서 pytorch를 설치할 때 gpu_cuda~ 로 되어 있는 버전으로 설치를 진행!!**
                        - **[중요!] pytorch version이 1.13 이상인 경우에는 gpu_cuda~ 이렇게 되어 있는 것을 설치하는 것이 아니라 따로 pytorch-cuda package를 설치해야한다 !!!!!**
                    
                    (3) “cudatoolkit” version 과 “cudatoolkit-dev” version을 일치시켜야 해야함
                    
                    - cudatoolkit version = 11.3
                    - cudatoolkit-dev version = 11.7
                        - 다음과 같이 변경 (downgrade 시킴)
                            - “`**conda install -c conda-forge cudatoolkit-dev=11.3**`”
                
                ---
                
                - 그래서 애초에 결론을 지은 것은 새로운 conda environment를 생성하고 pytorch 설치를 할 때 gpu~로 되어 있는 것을 설치하는 것으로 적용!!
                    
                    ```bash
                    $ conda install -c pytorch pytorch=1.12.1=gpu_cuda113py38h19ae3d8_1
                    
                    ```
                    
                    - 해당 “`python3`” 결과 !
                        
                        ```bash
                        $ python3
                        >>> import torch
                        >>> torch.__version__
                        '1.12.1'
                        >>> torch.version.cuda
                        '11.3'
                        >>> torch.backends.cudnn.version()
                        8902
                        ```
                        
                
                ---
                
                - 다시 “`**python3 setup.py install**`” 진행 !
                    - install 하는데 시간이 조금 많이 소요됨!
            
            ---
            
            - **[Reference Site 3]**
                
                - [What's the relationship between cuda_toolkit and pytorch](https://forums.developer.nvidia.com/t/whats-the-relationship-between-cuda-toolkit-and-pytorch/251096)
                

---

- **[Preparing Dataset to run NeRF-SLAM]**
    - 바로 RUN하면 되는 줄 알았는데 그게 아니네요….하하…. ^^.. 허허…. ^^
        
        **[Case 1. Replica Dataset]**
        
        - replica dataset으로 돌리면 pose를 달라고 하네요… 헤헤
            - **[Error 1]**
                - `FileNotFoundError: [Errno 2] No such file or directory: '/~/Dataset/Replica_dataset/room_0/traj.txt’`
            
            ---
            
            - **[Solve 1]**
                - **Traj.txt 만들어야 하는데… 추가로… ^^**
        
        **[Case 2. EuRoC Dataset]**
        
        - euroc dataset으로 돌리면 추가 error 발생
            - **[Error 1]**
                - `ModuleNotFoundError: No module named 'ruamel’`
            
            ---
            
            - **[Solve 1]**
                - Install ruamel.yaml
                    
                    ```bash
                    $ conda install -c conda-forge ruamel.yaml=0.17.21=py38h5eee18b_0
                    ```
                    
        
        **[Case 3. ~]**
        
        - **Custom dataset으로도 돌리고 싶은데 나중에… 추가로… ^^**

---

- **[Run NeRF-SLAM]**
    - install까지 모두 끝났다면 NeRF-SLAM을 RUN
        - **(중요) 본 저자는 하나의 GPU밖에 사용하지 않기 때문에 “`--parallel_run --multi_gpu`” 에 대한 flag는 지워야 가능!!!!!**
            - **해당 코드가 주는 다양한 flag들을 꼭 살펴볼 필요가 있다!!!!!!!!!**
                - **Flag가 담겨 있는 Code 부분 !**
                    
                    ```python
                    def parse_args():
                        parser = argparse.ArgumentParser(description="Instant-SLAM")
                    
                        # SLAM ARGS
                        parser.add_argument("--parallel_run", action="store_true", help="Whether to run in parallel")
                        parser.add_argument("--multi_gpu", action="store_true", help="Whether to run with multiple (two) GPUs")
                        parser.add_argument("--initial_k", type=int, help="Initial frame to parse in the dataset", default=0)
                        parser.add_argument("--final_k", type=int, help="Final frame to parse in the dataset, -1 is all.", default=-1)
                        parser.add_argument("--img_stride", type=int, help="Number of frames to skip when parsing the dataset", default=1)
                        parser.add_argument("--stereo", action="store_true", help="Use stereo images")
                        parser.add_argument("--weights", default="droid.pth", help="Path to the weights file")
                        parser.add_argument("--buffer", type=int, default=512, help="Number of keyframes to keep")
                    
                        parser.add_argument("--dataset_dir", type=str,
                                            help="Path to the dataset directory",
                                            default="/home/tonirv/Datasets/euroc/V1_01_easy")
                        parser.add_argument('--dataset_name', type=str, default='euroc',
                                            choices=['euroc', 'nerf', 'replica', 'real'],
                                            help='Dataset format to use.')
                    
                        parser.add_argument("--mask_type", type=str, default='ours', choices=['no_depth', 'raw', 'ours', 'ours_w_thresh'])
                    
                        #parser.add_argument("--gui", action="store_true", help="Run the testbed GUI interactively.")
                        parser.add_argument("--slam", action="store_true", help="Run SLAM.")
                        parser.add_argument("--fusion", type=str, default='', choices=['tsdf', 'sigma', 'nerf', ''],
                                            help="Fusion approach ('' for none):\n\
                                                -`tsdf' classical tsdf-fusion using Open3D\n \
                                                -`sigma' tsdf-fusion with uncertainty values (Rosinol22wacv)\n \
                                                -`nerf' radiance field reconstruction using Instant-NGP.")
                    
                        # GUI ARGS
                        parser.add_argument("--gui", action="store_true", help="Run O3D Gui, use when volume='tsdf'or'sigma'.")
                        parser.add_argument("--width",  "--screenshot_w", type=int, default=0, help="Resolution width of GUI and screenshots.")
                        parser.add_argument("--height", "--screenshot_h", type=int, default=0, help="Resolution height of GUI and screenshots.")
                    
                        # NERF ARGS
                        parser.add_argument("--network", default="", help="Path to the network config. Uses the scene's default if unspecified.")
                    
                        parser.add_argument("--eval", action="store_true", help="Evaluate method.")
                    
                        return parser.parse_args()
                    ```
                    
            - 해당 github에서 작성된 run code
                
                ```bash
                $ python3 ./examples/slam_demo.py --dataset_dir=./datasets/Replica/office0 --dataset_name=nerf --buffer=100 --slam --parallel_run --img_stride=2 --fusion='nerf' --multi_gpu --gui
                ```
                
            - 본 저자에 해당되는 run code
                
                ```bash
                $ python3 ./examples/slam_demo.py --dataset_dir=./datasets/Replica/office0 --dataset_name=nerf --buffer=100 --slam --img_stride=2 --fusion='nerf' --gui
                ```
                
        
        ---
        
        - **[Error 1]**
            - `ModuleNotFoundError: No module named 'colored_glog’`
        
        ---
        
        - **[Solve 1]**
            - Install coloredlogs
                
                ```bash
                $ pip3 install colored-glog
                ```
                
        
        ---
        
        - **[Error 2]**
            - `ModuleNotFoundError: No module named 'cv2’`
        
        ---
        
        - **[Solve 2]**
            - Install opencv-python
                
                ```bash
                $ pip3 install opencv-python
                ```
                
                - 그렇게 하면 **“python3 → import cv2”** 하면 해당 에러가 없어진 것을 확인할 수 있음
                - **(주의) conda install -c conda-forge opencv=4.6.0=py38hd653453_2 이런식으로 설치하면 pytorch가 cpu version으로 변경되기 때문에 pip3 으로 설치해야함**
        
        ---
        
        - **[Error 3]**
            - `OSError: libtorch_cuda_cu.so: cannot open shared object file: No such file or directory`
        
        ---
        
        - **[Solve 3]**
            - 하아… 해결하기 어려웠던 에러임…
                - 원인은 torch-scatter에서 일어난 문제인 것 같은데 conda environment의 torch version을 체크하고 거기에 맞게 설치가 되어 있는지 체크먼저 해봐야 할듯
                    - 그런데 본 저자의 경우에는 올바르게 설치된것 같음 (본 저자의 torch version 1.12.1, cuda version 11.3)
                        
                        ```bash
                        $ conda list
                        pytorch-scatter           2.1.0           py38_torch_1.12.0_cu113    pyg
                        ```
                        
                - 아니면 pybind11에서 읽어드리는 경로가 다를 수 있는 문제일 수도 있음
                    - 그런데 이 경우는 본 저자의 문제에는 걸리는 것이 아닌 것 같음…
                        
                        ```bash
                        $ python3
                        >>> import pybind11
                        >>> pybind11.get_include()
                        '/home/sj/anaconda3/envs/nerfslam/lib/python3.8/site-packages/pybind11/include'
                        >>> pybind11.get_include(True)
                        '/home/sj/anaconda3/envs/nerfslam/lib/python3.8/site-packages/pybind11/include'
                        ```
                        
            - 결국, “`conda install -c pyg pytorch-scatter`” 로 설치하는 것이 아니라 pip3 로 설치하는 방법을 선택 !
                - 여기서도 “`pip3 install torch-scatter -f https://data.pyg.org/whl/torch-1.12.1+cu113.html`[”](https://data.pyg.org/whl/torch-1.12.1+cu113.html”) 로 설치하는 것이 아니라 “https://pytorch-geometric.com/whl/torch-1.12.1+cu113.html” 로 설치해야함 !
                    - **추가로 “--no-index”를 꼭! 추가해주어야 설치가 가능함!!!**
                        
                        ```bash
                        # Install torch_scatter !!!!
                        $ pip3 install --no-index torch-scatter -f https://pytorch-geometric.com/whl/torch-1.12.1+cu113.html
                        ```
                        
        
        ---
        
        - **[Reference Site 3]**
            
            - [OSError: libtorch_cuda_cu.so: cannot open shared object file: No such file or directory · Issue #223 · rusty1s/pytorch_scatter](https://github.com/rusty1s/pytorch_scatter/issues/223)
            
            - [torch-scatter Error! · Issue #205 · rusty1s/pytorch_scatter](https://github.com/rusty1s/pytorch_scatter/issues/205)
            
        
        ---
        
        - **[Error 4]**
            - `ModuleNotFoundError: No module named 'sklearn’`
        
        ---
        
        - **[Solve 4]**
            - Install scikit-learn
                
                ```bash
                $ conda install -c anaconda scikit-learn=1.3.0=py38h1128e8f_0
                ```
                
        
        ---
        
        - **[Error 5]**
            - `ModuleNotFoundError: No module named 'addict’`
        
        ---
        
        - **[Solve 5]**
            - Install addict
                
                ```bash
                $ conda install -c conda-forge addict
                ```
                
        
        ---
        
        - **[Error 6]**
            - `ModuleNotFoundError: No module named 'plyfile’`
        
        ---
        
        - **[Solve 6]**
            - Install plyfile
                
                ```bash
                $ conda install -c conda-forge plyfile
                ```
                
        
        ---
        
        - **[Error 7]**
            - 끝도 없는 error의 세계…. 이것도 에러 해결하는데 빡세네요…. ^^
                - 이것도 원인은 torch-scatter에서 일어난 문제인 것 같은데 conda environment의 torch version을 체크하고 거기에 맞게 설치가 되어 있는지 체크먼저 해봐야 할듯
                    - 그런데 본 저자의 경우에는 올바르게 설치된것 같음 (본 저자의 torch version 1.12.1, cuda version 11.3)
                        
                        ```bash
                        $ python3
                        >>> import torch
                        >>> torch.__version__
                        '1.12.1'
                        >>> torch.version.cuda
                        '11.3'
                        
                        $ pip3 list | grep scatter
                        torch-scatter                   2.1.0+pt112cu113
                        ```
                        
                - 위와 같이 pip3 로 install 하면 되지 않을까? 해서 pytorch_geometric 에서 제공하는 다른 package도 모두 설치해보았다…
                    - 그런데 동일한 error가 계속 발생함…
                        
                        ```bash
                        $ pip3 install --no-index torch-cluster torch-sparse torch-spline-conv -f https://pytorch-geometric.com/whl/torch-1.12.1+cu113.html
                        $ pip3 install torch-geometric -f https://pytorch-geometric.com/whl/torch-1.12.1+cu113.html
                        $ pip3 install pyg-lib -f https://data.pyg.org/whl/torch-1.12.1+cu113.html
                        ```
                        
            - 결국!!! 해결했는데 그것이 torch version 을 낮추고 그것에 맞는 torch_scatter를 설치해주어야 한다!
                - **이 답글대로 해야 함…!!**
                    - Reference Site → [https://github.com/pyg-team/pytorch_geometric/issues/999#issuecomment-606565132](https://github.com/pyg-team/pytorch_geometric/issues/999#issuecomment-606565132)
                - Stable한 Pytorch를 다운받는 site
                    
                    - [https://download.pytorch.org/whl/torch_stable.html](https://download.pytorch.org/whl/torch_stable.html)
                    
                    - 본 저자는 “`cu101/torch-1.4.0-cp38-cp38-linux_x86_64.whl` & `cu101/torchvision-0.5.0-cp38-cp38-linux_x86_64.whl`” 를 다운받고 해당 conda environment에서 pip3 install 진행
                - 해당 torch version에 맞는 torch_scatter 다운받는 site
                    
                    - [https://pytorch-geometric.com/whl/torch-1.4.0%2Bcu101.html](https://pytorch-geometric.com/whl/torch-1.4.0%2Bcu101.html)
                    
                    - 본 저자는 “`torch_scatter-2.0.4-cp38-cp38-linux_x86_64.whl` & `torch_cluster-1.5.3-cp38-cp38-linux_x86_64.whl` & `torch_sparse-0.6.1-cp38-cp38-linux_x86_64.whl` & `torch_spline_conv-1.2.0-cp38-cp38-linux_x86_64.whl`” 를 다운받고 conda environment에서 pip3 install 진행
                - 이렇게 하면 python3에서 import torch_scatter 를 진행해도 다른 error 없이 import 진행
        
        ---
        
        - **[Reference Site 7]**
            
            - [https://pytorch-geometric.com/whl/](https://pytorch-geometric.com/whl/)
            
            - [can't run with this error: undefined symbol: _ZN5torch3jit17parseSchemaOrNameERKSs · Issue #3836 · pyg-team/pytorch_geometric](https://github.com/pyg-team/pytorch_geometric/issues/3836)
            
            - [Undefined symbol: _ZN5torch3jit17parseSchemaOrNameERKSs · Issue #999 · pyg-team/pytorch_geometric](https://github.com/pyg-team/pytorch_geometric/issues/999)
            
        
        ---
        
        - **[Error 8]**
            - ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29' not found (required by /home/sj/anaconda3/envs/nerfslam/lib/python3.8/site-packages/pandas/_libs/window/aggregations.cpython-38-x86_64-linux-gnu.so)
        
        ---
        
        - **[Solve 8]**
            - 해당 버전이 local에 설치되어 있지 않기 때문에 생기는 문제
                
                ```bash
                $ strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
                ```
                
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nerf_slam/Untitled.png" alt="">
                </figure>
                
            - 다음과 같은 명령어를 치면 해당 버전이 upgrade 됨
                
                ```bash
                $ sudo add-apt-repository ppa:ubuntu-toolchain-r/test 
                $ sudo apt-get update
                $ sudo apt-get upgrade
                $ sudo apt-get dist-upgrade
                
                **# 결과 확인 !! -> GLIBCXX_3.4.29 가 생성되어 있는 것을 볼 수 있음 !!**
                $ strings /usr/lib/x86_64-linux-gnu/libstdc++.so.6 | grep GLIBCXX
                ```

                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/nerf_slam/Untitled 1.png" alt="">
                </figure>
        
        ---
        
        - **[Reference Site 8]**
            
            - [ImportError: /lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.29' not found](https://velog.io/@ssw9999/ImportError-libx8664-linux-gnulibstdc.so.6-version-GLIBCXX3.4.29-not-found)
            
        
        ---
        
        - **[Error 9]**
            - `ModuleNotFoundError: No module named 'yaml’`
        
        ---
        
        - **[Solve 9]**
            - install pyyaml
                
                ```bash
                $ pip3 install pyyaml
                ```
                
                - **Conda로 설치하면 똑같은 에러가 발생할 수 있으므로 pip으로 설치 !!**
        
        ---
        
        - **[Reference Site 9]**
            
            - [ModuleNotFoundError: No module named 'yaml' 에러 해결](https://supermemi.tistory.com/entry/ModuleNotFoundError-No-module-named-yaml-%EC%97%90%EB%9F%AC-%ED%95%B4%EA%B2%B0)
            
        
        ---
        
        - **[Error 10]**
            - `AttributeError: type object 'gtsam.gtsam.Pose3' has no attribute 'identity’`
        
        ---
        
        - **[Solve 10]**
            - 수정된 gtsam github가 있어 이걸 clone하고 이걸 build 진행 (위에서 clone github를 해도 문제가 생기는 경우)
                
                [GitHub - ToniRV/gtsam-1: GTSAM is a library of C++ classes that implement smoothing and mapping (SAM) in robotics and vision, using factor graphs and Bayes networks as the underlying computing paradigm rather than sparse matrices.](https://github.com/ToniRV/gtsam-1.git)
                
        
        ---
        
        - **[Reference Site]**
            
            - [type object 'gtsam.gtsam.Pose3' has no attribute 'identity' · Issue #21 · ToniRV/NeRF-SLAM](https://github.com/ToniRV/NeRF-SLAM/issues/21#issuecomment-1373102037)
            

---

- **[GPU Memory]**
    - 메모리 사용량을 체크하고 싶다면 해당 site에서 추천하는 사용량을 보는 tool을 이용
        
        ```bash
        # Install GPU memory monitoring
        $ pip3 install --upgrade nvitop
        
        # Running GPU memory monitor
        nvitop --monitor
        ```
        

---

- **[Run Only Instant-NGP]**
    - 먼저 Instant-NGP에 들어가는 dataset을 만들기
        - video를 colmap기반 dataset으로 만드는 코드를 제공하고 있음
            - 해당 코드 [scripts/colmap2nerf.py](https://github.com/ToniRV/instant-ngp/blob/54aba7cfbeaf6a60f29469a9938485bebeba24c3/scripts/colmap2nerf.py) 를 살펴보면 다음과 같이 제공하고 있음을 알 수 있음
                - **해당 코드가 주는 다양한 flag들을 꼭 살펴볼 필요가 있다!!!!!!!!!**
                    - **scripts/colmap2nerf.py에서 제공하는 flag**
                        
                        ```python
                        def parse_args():
                        	parser = argparse.ArgumentParser(description="convert a text colmap export to nerf format transforms.json; optionally convert video to images, and optionally run colmap in the first place")
                        
                        	parser.add_argument("--video_in", default="", help="run ffmpeg first to convert a provided video file into a set of images. uses the video_fps parameter also")
                        	parser.add_argument("--video_fps", default=2)
                        	parser.add_argument("--time_slice", default="", help="time (in seconds) in the format t1,t2 within which the images should be generated from the video. eg: \"--time_slice '10,300'\" will generate images only from 10th second to 300th second of the video")
                        	parser.add_argument("--run_colmap", action="store_true", help="run colmap first on the image folder")
                        	parser.add_argument("--colmap_matcher", default="sequential", choices=["exhaustive","sequential","spatial","transitive","vocab_tree"], help="select which matcher colmap should use. sequential for videos, exhaustive for adhoc images")
                        	parser.add_argument("--colmap_db", default="colmap.db", help="colmap database filename")
                        	parser.add_argument("--colmap_camera_model", default="OPENCV", choices=["SIMPLE_PINHOLE", "PINHOLE", "SIMPLE_RADIAL", "RADIAL","OPENCV"], help="camera model")
                        	parser.add_argument("--colmap_camera_params", default="", help="intrinsic parameters, depending on the chosen model.  Format: fx,fy,cx,cy,dist")
                        	parser.add_argument("--images", default="images", help="input path to the images")
                        	parser.add_argument("--text", default="colmap_text", help="input path to the colmap text files (set automatically if run_colmap is used)")
                        	parser.add_argument("--aabb_scale", default=16, choices=["1","2","4","8","16"], help="large scene scale factor. 1=scene fits in unit cube; power of 2 up to 16")
                        	parser.add_argument("--skip_early", default=0, help="skip this many images from the start")
                        	parser.add_argument("--keep_colmap_coords", action="store_true", help="keep transforms.json in COLMAP's original frame of reference (this will avoid reorienting and repositioning the scene for preview and rendering)")
                        	parser.add_argument("--out", default="transforms.json", help="output path")
                        	parser.add_argument("--vocab_path", default="", help="vocabulary tree path")
                        	args = parser.parse_args()
                        	return args
                        ```
                        
                - 해당 github에서 작성된 convert code
                    
                    ```bash
                    # Using Video File !!
                    $ python [path-to-instant-ngp]/scripts/colmap2nerf.py --video_in <filename of video> --video_fps 2 --run_colmap --aabb_scale 16
                    ```
                    
                - 본 저자에 맞는 convert code
                    
                    ```bash
                    $ python3 ./thirdparty/instant-ngp/scripts/colmap2nerf.py \
                    					--video_in /home/sj/workspace/bag/Dataset/KAIST-RGBD/B401_Classroom_with_ceiling.mp4 \
                    					--video_fps 2 --run_colmap \
                    					--colmap_db /home/sj/workspace/bag/Dataset/KAIST-RGBD/ceiling.db \
                    					--text /home/sj/workspace/bag/Dataset/KAIST-RGBD/ceiling --aabb_scale 16 \
                    					--out /home/sj/workspace/bag/Dataset/KAIST-RGBD/ceiling.json
                    ```
                    
    
    ---
    
    - 만들어진 dataset 결과와 Instant-NGP를 build한 결과 **build_ngp/testbed** or **scripts/run.py** 를 직접 돌려 instant-ngp 시작!
        - **해당 코드가 주는 다양한 flag들을 꼭 살펴볼 필요가 있다!!!!!!!!!**
            - **scripts/run.py에서 제공하는 flag**
                
                ```python
                def parse_args():
                	parser = argparse.ArgumentParser(description="Run neural graphics primitives testbed with additional configuration & output options")
                
                	parser.add_argument("--scene", "--training_data", default="", help="The scene to load. Can be the scene's name or a full path to the training data.")
                	parser.add_argument("--mode", default="", const="nerf", nargs="?", choices=["nerf", "sdf", "image", "volume"], help="Mode can be 'nerf', 'sdf', 'image' or 'volume'. Inferred from the scene if unspecified.")
                	parser.add_argument("--network", default="", help="Path to the network config. Uses the scene's default if unspecified.")
                
                	parser.add_argument("--load_snapshot", default="", help="Load this snapshot before training. recommended extension: .msgpack")
                	parser.add_argument("--save_snapshot", default="", help="Save this snapshot after training. recommended extension: .msgpack")
                
                	parser.add_argument("--nerf_compatibility", action="store_true", help="Matches parameters with original NeRF. Can cause slowness and worse results on some scenes.")
                	parser.add_argument("--test_transforms", default="", help="Path to a nerf style transforms json from which we will compute PSNR.")
                	parser.add_argument("--near_distance", default=-1, type=float, help="Set the distance from the camera at which training rays start for nerf. <0 means use ngp default")
                	parser.add_argument("--exposure", default=0.0, type=float, help="Controls the brightness of the image. Positive numbers increase brightness, negative numbers decrease it.")
                
                	parser.add_argument("--screenshot_transforms", default="", help="Path to a nerf style transforms.json from which to save screenshots.")
                	parser.add_argument("--screenshot_frames", nargs="*", help="Which frame(s) to take screenshots of.")
                	parser.add_argument("--screenshot_dir", default="", help="Which directory to output screenshots to.")
                	parser.add_argument("--screenshot_spp", type=int, default=16, help="Number of samples per pixel in screenshots.")
                
                	parser.add_argument("--video_camera_path", default="", help="The camera path to render, e.g., base_cam.json.")
                	parser.add_argument("--video_camera_smoothing", action="store_true", help="Applies additional smoothing to the camera trajectory with the caveat that the endpoint of the camera path may not be reached.")
                	parser.add_argument("--video_loop_animation", action="store_true", help="Connect the last and first keyframes in a continuous loop.")
                	parser.add_argument("--video_fps", type=int, default=60, help="Number of frames per second.")
                	parser.add_argument("--video_n_seconds", type=int, default=1, help="Number of seconds the rendered video should be long.")
                	parser.add_argument("--video_spp", type=int, default=8, help="Number of samples per pixel. A larger number means less noise, but slower rendering.")
                	parser.add_argument("--video_output", type=str, default="video.mp4", help="Filename of the output video.")
                
                	parser.add_argument("--save_mesh", default="", help="Output a marching-cubes based mesh from the NeRF or SDF model. Supports OBJ and PLY format.")
                	parser.add_argument("--marching_cubes_res", default=256, type=int, help="Sets the resolution for the marching cubes grid.")
                
                	parser.add_argument("--width", "--screenshot_w", type=int, default=0, help="Resolution width of GUI and screenshots.")
                	parser.add_argument("--height", "--screenshot_h", type=int, default=0, help="Resolution height of GUI and screenshots.")
                
                	parser.add_argument("--gui", action="store_true", help="Run the testbed GUI interactively.")
                	parser.add_argument("--train", action="store_true", help="If the GUI is enabled, controls whether training starts immediately.")
                	parser.add_argument("--n_steps", type=int, default=-1, help="Number of steps to train for before quitting.")
                	parser.add_argument("--second_window", action="store_true", help="Open a second window containing a copy of the main output.")
                
                	parser.add_argument("--sharpen", default=0, help="Set amount of sharpening applied to NeRF training images. Range 0.0 to 1.0.")
                
                	return parser.parse_args()
                ```
                
        - 해당 github에서 작성된 run code
            
            ```bash
            $ instant-ngp$ ./build/testbed --mode nerf --scene [path to training data folder containing transforms.json]
            ```
            
        - 본 저자에 맞는 run code
            
            ```bash
            $ ./build_ngp/testbed --mode nerf --scene /home/sj/workspace/bag/Dataset/KAIST-RGBD/KAIST_RGBD_2/transforms.json
            ```
            
    
    ---
    
    - Instant-NGP만 돌리는대에도 error가 존재
        - **[Error 1]**
            - `ffmpeg was not found`
        
        ---
        
        - **[Solve 1]**
            - ffmpeg 설치
                
                ```bash
                $ pip3 install ffmpeg
                ```
                
        
        ---
        
        - **[Reference Site 1]**
            
            - [ffmpeg was not found. How do i fix this?](https://stackoverflow.com/questions/73335675/ffmpeg-was-not-found-how-do-i-fix-this)
            
        
        ---
        
        - **[Error 2]**
            - `No module named 'pyngp’`
        
        ---
        
        - **[Solve 2]**
            - build 경로가 명칭이 변경되었거나 달라졌기 때문에 해당 path를 absolute path로 명명해주어야 함
                - scripts/run.py 로 가서 `import pyngp as ngp` 앞에 다음과 같은 code 추가!
                    
                    ```python
                    pyngp_path = '/path/to/your/build'
                    sys.path.append(pyngp_path)
                    import pyngp as ngp
                    ```
                    
        
        ---
        
        - **[Reference Site 2]**
            
            - [No module named 'pyngp' · Issue #43 · NVlabs/instant-ngp](https://github.com/NVlabs/instant-ngp/issues/43#issuecomment-1016136005)
            
        
        ---
        
        - **[Error 3]**
            - `ImportError: libcublas.so.11: cannot open shared object file: No such file or directory`
        
        ---
        
        - **[Solve 3]**
            - **conda에 install이 되어 있고 /usr/local 에 없는 경우 해당 파일을 못찾는 것이 원인이기 때문에 직접 경로를 추가해주고 다시 build를 해주어야 한다!!**
                - 본 저자가 terminal에 직접 넣어준 code
                    
                    ```bash
                    $ export PATH="/home/sj/anaconda3/envs/nerfslam/pkgs:$PATH"
                    $ export LD_LIBRARY_PATH="/home/sj/anaconda3/envs/nerfslam/lib:$LD_LIBRARY_PATH"
                    ```
                    
        
        ---
        
        - **[Reference Site 3]**
            
            - [GitHub - NVlabs/instant-ngp: Instant neural graphics primitives: lightning fast NeRF and more](https://github.com/NVlabs/instant-ngp#:~:text=We%20also%20recommend,%24LD_LIBRARY_PATH%22)
            
    
    ---
    
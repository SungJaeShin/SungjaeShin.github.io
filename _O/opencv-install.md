---
title: "[OpenCV] How to install appropriate OpenCV version?"
excerpt: "Install Specific OpenCV Version"
---
# [OpenCV] How to install appropriate OpenCV version?

## [Goal] 사용자가 원하는 OpenCV library Version을 local에 설치하고 기존에 있는 OpenCV library를 제거

---

- To check current version of opencv on desktop
    
    ```bash
    $ pkg-config --modversion opencv
    
    # example above sentence
    3.2.0
    ```
    

---

- Remove current version of opencv on desktop
    
    ```bash
    $ sudo apt purge libopencv* python-opencv
    
    # To remove it more neatly, look for the following as:
    $ cd /
    $ sudo find -name "*opencv*"
    
    **# It directly searches for the files corresponding to the results of the above sentence and deletes them**
    # One example of aformentioned sentence
    $ cd /usr/local/lib
    $ sudo rm -r libopencv_*
    
    # Then checking there is no package on own desktop
    $ pkg-config --modversion opencv
    
    # example above sentence 
    No package 'opencv' found
    ```
    

---

- To install opencv version 3.4.0 **[There are two ways: (1) OpenCV Only (2) OpenCV with CUDA]**
    - Reference Site: [https://www.pytorials.com/how-to-install-opencv340-on-ubuntu1604/](https://www.pytorials.com/how-to-install-opencv340-on-ubuntu1604/)
        
    - Following as:
        
        ```bash
        # To install Opencv Only
        $ pip install opencv-python
        
        # Install Dependencies
        $ sudo apt-get install build-essential
        $ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
        
        # To Download OpenCV 3.4.0 & OpenCV Contrib 3.4.0 
        # (Contrib has very useful algorithms which is a must for anyone working on Computer Vision)
        $ wget [https://github.com/opencv/opencv/archive/3.4.0.zip](https://github.com/opencv/opencv/archive/3.4.0.zip) -O opencv-3.4.0.zip
        $ wget https://github.com/opencv/opencv_contrib/archive/3.4.0.zip -O opencv_contrib-3.4.0.zip
        
        # Extrack OpenCV
        $ unzip opencv-3.4.0.zip
        $ unzip opencv_contrib-3.4.0.zip
        
        # Make a directory named build inside OpenCV-3.4.0
        $ cd opencv-3.4.0
        $ mkdir build
        $ cd build
        
        # Configuration cmake
        # First method **install only opencv**
        $ cmake -D CMAKE_BUILD_TYPE=RELEASE \
                -D CMAKE_INSTALL_PREFIX=/usr/local \
                -D OPENCV_GENERATE_PKGCONFIG=YES \
                -D WITH_CUDA=OFF \
                -D WITH_CUDNN=OFF \
                -D OPENCV_DNN_CUDA=OFF \
                -D WITH_FFMPEG=OFF \
                -D WITH_CUFFT=ON \
                -D ENABLE_FAST_MATH=ON \
                -D CUDA_FAST_MATH=ON \
                -D WITH_CUBLAS=ON \
                -D WITH_LIBV4L=ON \
                -D WITH_GSTREAMER=ON \
                -D WITH_GSTREAMER_0_10=OFF \
                -D WITH_QT=OFF \
                -D WITH_OPENGL=ON \
                -D BUILD_opencv_cudacodec=OFF \
                -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
                -D WITH_TBB=ON \
        				-D ENABLE_PRECOMPILED_HEADERS=OFF \
                -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.0/modules \
                ../
        
        # If you want to **connect with CUDA** then following this github:
        # ***** **https://github.com/engcang/vins-application#-cv_bridge-with-opencv-3x-version** *****
        $ cmake -D CMAKE_BUILD_TYPE=RELEASE \
                -D CMAKE_C_COMPILER=gcc-6 \ # IF there has some error then install gcc-6 g++-6 !!
                -D CMAKE_CXX_COMPILER=g++-6 \
                -D CMAKE_INSTALL_PREFIX=/usr/local \
                -D OPENCV_GENERATE_PKGCONFIG=YES \
                -D WITH_CUDA=ON \
                -D WITH_CUDNN=ON \
                -D OPENCV_DNN_CUDA=ON \
                -D WITH_CUFFT=ON \
                -D WITH_FFMPEG=ON \
                -D CUDA_ARCH_BIN=8.6 \ # Please check Compute capability
                -D CUDA_ARCH_PTX="" \
                -D ENABLE_FAST_MATH=ON \
                -D CUDA_FAST_MATH=ON \
                -D WITH_CUBLAS=ON \
                -D WITH_LIBV4L=ON \
                -D WITH_GSTREAMER=ON \
                -D WITH_GSTREAMER_0_10=OFF \
                -D WITH_QT=ON \
                -D WITH_OPENGL=ON \
                -D BUILD_opencv_cudacodec=OFF \
                -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
                -D WITH_TBB=ON \
                ../
        
        # Build
        $ make -j4
        
        # Install in the location /usr/local using command
        $ sudo make install
        ```
        

---

- Now, check changed opencv version
    
    ```bash
    $ pkg-config --modversion opencv
    
    # example above sentence
    3.4.0
    ```
    

---

- ********* **[Error List] *******
    - **[1] [TypeError]**: 'NoneType' object is not iterable opencv & **[ERROR]**: Command errored out with exit status 1:  ~/.venvs/venv/bin/python ~/.venvs/venv/lib/python2.7/site-packages/pip/_vendor/pep517/ _in_process.py get_requires_for_build_wheel /tmp/tmp7q7z4L Check the logs for full command output.
        - **(A)** Change version of **pip & opencv-python**
            - Reference Site: [https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild](https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild)
                
            - Reference Site (Kor): [https://remoted.tistory.com/418](https://remoted.tistory.com/418)
                
            - Following As:
                
                ```bash
                # To upgrade pip
                $ python -m pip install --upgrade pip
                
                # To install downgrade opencv-python version
                $ python -m pip install opencv-python==4.2.0.32
                ```
                
    
    ---
    
    - **[2]** **[CMake Error]**: The following variables are used in this project, but they are set to NOTFOUND. Please set them or make sure they are set and tested correctly in the CMake files: CUDA_nppi_LIBRARY (ADVANCED)
        - **(A)** Change **FindCUDA.cmake & OpenCVDetectCUDA.cmake in  opencv-3.4.0/cmake**
            - Reference Site:  [https://stackoverflow.com/questions/46584000/cmake-error-variables-are-set-to-notfound](https://stackoverflow.com/questions/46584000/cmake-error-variables-are-set-to-notfound)
                
            - Change Code from following as: [https://github.com/opencv/opencv/tree/master/cmake](https://github.com/opencv/opencv/tree/master/cmake)   
    
    ---
    
    - **[3] [fatal error]**: stdlib.h: No such file or directory #include_next <stdlib.h>
        - **(A)** Add one line when cmake in build folder or modified CMakeList.txt in opencv folder
            - Reference Site: [https://stackoverflow.com/questions/40262928/error-compiling-opencv-fatal-error-stdlib-h-no-such-file-or-directory](https://stackoverflow.com/questions/40262928/error-compiling-opencv-fatal-error-stdlib-h-no-such-file-or-directory)
                                
            - Following as:
                
                ```bash
                # Add following line:
                -D ENABLE_PRECOMPILED_HEADERS=OFF
                
                # Or modified CMakeList.txt file
                OCV_OPTION(ENABLE_PRECOMPILED_HEADERS "Use precompiled headers"                                  
                **OFF** IF (NOT IOS AND NOT CMAKE_CROSSCOMPILING) )
                ```
                
    
    ---
    
    - **[4] [Linking error]**: undefined reference TIFFReadDirectory@LIBTIFF_4.0 & TIFFWriteEncodedStrip@LIBTIFF_4.0 & TIFFIsTiled@LIBTIFF_4.0 &  TIFFOpen@LIBTIFF_4.0 &
    TIFFReadEncodedStrip@LIBTIFF_4.0 &
    TIFFSetField@LIBTIFF_4.0 &
    TIFFWriteScanline@LIBTIFF_4.0 &
    TIFFGetField@LIBTIFF_4.0 &
    TIFFScanlineSize@LIBTIFF_4.0 &
    TIFFNumberOfStrips@LIBTIFF_4.0 &
    TIFFSetWarningHandler@LIBTIFF_4.0 &
    TIFFSetErrorHandler@LIBTIFF_4.0 &
    TIFFReadEncodedTile@LIBTIFF_4.0 &
    TIFFReadRGBATile@LIBTIFF_4.0 &
    TIFFClose@LIBTIFF_4.0 &
    TIFFRGBAImageOK@LIBTIFF_4.0 &
    TIFFReadRGBAStrip@LIBTIFF_4.0
        - **(A)** Modified CMakeList.txt in opencv folder or In conda environment then uninstall tiff library **(conflict with anaconda lib and local lib)**
            - Reference Site: [https://github.com/BVLC/caffe/issues/4436](https://github.com/BVLC/caffe/issues/4436)
                
            - Following as:
                
                ```yaml
                # Modified CMakeList.txt in opencv folder
                OCV_OPTION(WITH_TIFF           "Include TIFF support"                        
                **OFF**   IF (NOT IOS) )
                ```
                
            - **[Recommend]** Reference Site: [https://github.com/colmap/colmap/issues/188](https://github.com/colmap/colmap/issues/188)
                
            - Following as:
                
                ```bash
                # First Check libtiff existed in /usr/lib/x86_64-linux-gnu
                $ sudo ldconfig -p | grep libtiff
                
                # In terminal with anaconda environment
                (base) $ conda uninstall libtiff
                ```
                
    
    ---
    
    - **[5]** **[Linking error]**: libSM.so.6: undefined reference to `uuid_generate@UUID_1.0', `uuid_unparse_lower@UUID_1.0'
        - **(A)** **Conflict with Anaconda library and local library**. So remove libuuid.* in anaconda library
            - Reference Site: [https://stackoverflow.com/questions/45584275/getting-error-usr-lib-lib64-libsm-so-undefined-reference-to-uuid-unparse-l](https://stackoverflow.com/questions/45584275/getting-error-usr-lib-lib64-libsm-so-undefined-reference-to-uuid-unparse-l)
                
            - Following as:
                
                ```bash
                # First Check libuuif existed in /usr/lib/x86_64-linux-gnu
                $ sudo ldconfig -p | grep libuuif
                
                # Go to Anaconda library and remove uuid library
                $ cd anaconda3/lib
                $ sudo rm -r libuuid.* 
                ```
                
            - Then re-build opencv
    
    ---
    
    - **[6]** **[Make error]**: Makefile:162: recipe for target 'all' failed
        - (A) Do not limited core when make opencv build.
            - Reference Site: [https://github.com/kakao/khaiii/issues/23#issuecomment-445687382](https://github.com/kakao/khaiii/issues/23#issuecomment-445687382)
                
            - Following as:
                
                ```bash
                # Not "make -j4" but only "make" 
                $ make
                ```
                
    
    ---
    
    - **[7] [Cmake error]**: CMake Error at cmake/OpenCVCompilerOptions.cmake:21 (else): Flow control statements are not properly nested.
        - (A) Comment 21 ~ 22 line in OpenCVCompilerOptions.cmake
            - Reference Site: [https://gist.github.com/LorenzoLamberti94/2f35d844121a63558c425618ea12ceef#file-opencv3-2withcontrib-sh-L76](https://gist.github.com/LorenzoLamberti94/2f35d844121a63558c425618ea12ceef#file-opencv3-2withcontrib-sh-L76)
                
            - Following as:
                
                ```bash
                $ cd opencv-3.2.0/cmake
                $ sed -i '21,22 s/^/#/' ./OpenCVCompilerOptions.cmake
                ```
                
    
    ---
    
    - **[8] [fatal error]**: LAPACKE_H_PATH-NOTFOUND/lapacke.h: No such file or directory #include "LAPACKE_H_PATH-NOTFOUND/lapacke.h"
        - (A) OpenCV cannot find lapacke package → change installed path
            - Reference Site: [https://github.com/YuvalNirkin/face_swap/issues/51](https://github.com/YuvalNirkin/face_swap/issues/51)
                
            - Following as:
                
                ```bash
                # Install lapacke library
                $ sudo apt-get install liblapacke-dev checkinstall
                
                # Go to build folder
                $ cd opencv-3.2.0/build
                
                # Change line 2 
                # [Before] #include "LAPACKE_H_PATH-NOTFOUND/lapacke.h"
                # [After] #include "/usr/include/lapacke.h"
                
                $ make
                ```
                
    
    ---
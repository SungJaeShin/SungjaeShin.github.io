---
title: "[Cuda] How to find version of Cuda, Cuda Toolkit, and Cudnn?"
excerpt: "Find Cuda, Cuda Toolkit, and Cudnn version"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Cuda] How to find version of Cuda, Cuda Toolkit, and Cudnn?

- (1) 자신의 desktop이 어떤 GPU를 사용하는지 확인
    - Terminal로 찾는 방법
        
        ```bash
        $ sudo lshw -C display
        ```
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/cuda/terminal.png" alt="">
        </figure> 

    - Desktop Setting에서 확인하는 방법
        - Settings → Details → About → Find Graphics
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/cuda/details.png" alt="">
            </figure> 
                       

---

- (2) 자신의 GPU 환경에 맞는 Nvidia Driver 버전 확인
    - Reference Site        
        - [드라이버 다운로드](https://www.nvidia.co.kr/Download/index.aspx?lang=kr)
        
    - Reference Site
        - **해당 하드웨어에 대한 드라이버가 어디 버전까지 지원이 가능한지 볼 수 있음**
            
            - [Advanced Driver Search](https://www.nvidia.com/Download/Find.aspx)
            
        - Example
            - 위의 예시로 통한 버전 확인 결과
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/cuda/nvidia-driver-version.png" alt="">
                </figure> 
              

---

- (3) 자신의 NVIDIA Driver 버전에 맞는 Cuda Toolkit 버전 확인
    - Reference Site
        - [CUDA GPUs](https://developer.nvidia.com/cuda-gpus)
        
    - Reference Site        
        - [CUDA Compatibility](https://docs.nvidia.com/deploy/cuda-compatibility/)
        
    - Reference Site        
        - [Release Notes :: CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html)
        
        - Example
            - 위의 예시로 통한 Cuda Toolkit 버전 확인 결과
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/cuda/minimum.png" alt="">
                </figure> 

                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/cuda/table.png" alt="">
                </figure>

---

- (4) 자신의 Cuda 버전에 맞는 cuDNN 버전 확인
    - Reference Site: [https://developer.nvidia.com/rdp/cudnn-archive](https://developer.nvidia.com/rdp/cudnn-archive)
        - Example
            - 위의 예시로 통한 cuDNN 버전 확인 결과
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/cuda/cuDNN_Version.png" alt="">
                </figure>
                

---

- (5) 결론: 이를 통해 확인된 나의 desktop에 맞는 버전은 다음과 같다.
    - **NVIDIA Driver → 470.82**
    - **CUDA Toolkit Driver → CUDA 11.4**
    - **cuDNN → 8.2.4**

---

- (6) Docker 환경을 통해서 해당 환경을 설치하려면 다음과 같은 사이트를 참고해준다.
    - Reference Site: [https://github.com/deepshwang/howtodocker#sec1](https://github.com/deepshwang/howtodocker#sec1)
        - **설명이 아주 잘 되어있음!**

---

- (7) Appendix (자신의 Compatibility 에 맞는 CUDA 및 NVIDIA driver version 확인하는 사이트)
    - Reference Site: [https://en.wikipedia.org/wiki/CUDA](https://en.wikipedia.org/wiki/CUDA)
        
    - Reference Site: [https://docs.nvidia.com/pdf/CUDA_Compatibility.pdf](https://docs.nvidia.com/pdf/CUDA_Compatibility.pdf)

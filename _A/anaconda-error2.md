---
title: "[Anaconda Error] ImportError: /lib/x86_64-linux-gnu/~.so.6: version `GLIBC_2.29' not found (required by /home/sj/anaconda3/envs/~/lib/python3.8/site-packages/~.so)"
excerpt: "Anaconda Error 2"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Anaconda Error] ImportError: /lib/x86_64-linux-gnu/~.so.6: version `GLIBC_2.29' not found (required by /home/sj/anaconda3/envs/~/lib/python3.8/site-packages/~.so)

---

- **[Goal]**
    - Anaconda를 이용하다가 GLIBC version과 충돌이 되거나 없는 경우, 추가 및 변경하여 이를 해결할 수 있음

---

- **[Cause]**
    - Conda를 사용하다가 보면 GNU에서 만든 local lib version 이 없거나 다른 경우 해당 문제가 발생하는데 생각보다 이러한 문제가 생긴 경우가 많음

---

- **[How to solve?]**
    - [Step 1] 해당 버전이 있는지 check
        
        ```bash
        $ strings /lib/x86_64-linux-gnu/libXXX.so.6 | grep GLIBC_
        GLIBC_2.2.5
        GLIBC_2.4
        GLIBC_2.15
        GLIBC_2.18
        GLIBC_2.23
        GLIBC_2.24
        GLIBC_2.25
        GLIBC_2.26
        GLIBC_2.27
        GLIBC_PRIVATE
        
        # Current Version
        $ ldd --version
        ldd (Ubuntu GLIBC 2.27-3ubuntu1.6) 2.27
        Copyright (C) 2018 Free Software Foundation, Inc.
        This is free software; see the source for copying conditions.  There is NO
        warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
        Written by Roland McGrath and Ulrich Drepper.
        ```
        
        - 해당 저자의 경우 GLIBC_2.29 가 없고 2.27이기 때문에 해당 버전을 추가 설치 진행
    
    ---
    
    - [Step 2] 해당 버전 local에 설치
        
        ```bash
        $ wget -c https://ftp.gnu.org/gnu/glibc/glibc-2.29.tar.gz
        $ tar -zxvf glibc-2.29.tar.gz
        $ cd glibc-2.29
        $ mkdir build
        $ cd build
        $ ../configure --prefix=/usr/local/glibc-2.29/build --disable-sanity-checks
        $ make
        $ sudo make install
        ```
        
        - **[Error]** `** These critical programs are missing or too old: gawk bison`
            - 해당 library가 local에 설치가 되어 있지 않거나 version이 낮기 때문에 문제가 추가 발생
                
                (1) gawk & bison 버전 찾기
                
                ```bash
                $ ls -lah /usr/bin/gawk
                ls: cannot access '/usr/bin/gawk': No such file or directory
                $ ls -lah /usr/bin/bison 
                ls: cannot access '/usr/bin/bison': No such file or directory
                $ bison --version
                $ gawk --version
                ```
                
                (2) 해당 lib 설치
                
                ```bash
                $ sudo apt install gawk
                $ sudo apt install bison
                ```
                
    
    ---
    
    - [Step 3] 제대로 lib가 설치되었는지 확인
        
        ```bash
        $ cd /usr/local/glibc-2.29/build/lib
        $ ls | grep libm.so
        
        # Results
        libm.so
        libm.so.6
        ```
        
    
    ---
    
    - [Step 4] 해당 lib가 local GNU에 추가 및 soft linking (connection) 진행
        
        ```bash
        # Copy libm.so.6 in /usr/local/glibc-2.29/build/lib
        $ sudo cp /usr/local/glibc-2.29/build/lib/libm-2.29.so /lib/x86_64-linux-gnu/
        
        # Make a forced soft connection
        $ cd /lib/x86_64-linux-gnu/
        $ sudo ln -sf libm-2.29.so libm.so.6
        ```
        
    
    ---
    
    - [Step 5] 버전이 추가되었는지 최종 확인
        
        ```bash
        $ strings /lib/x86_64-linux-gnu/libm.so.6 | grep GLIBC_
        
        # Results
        GLIBC_2.2.5
        GLIBC_2.4
        GLIBC_2.15
        GLIBC_2.18
        GLIBC_2.23
        GLIBC_2.24
        GLIBC_2.25
        GLIBC_2.26
        GLIBC_2.27
        GLIBC_2.28
        GLIBC_2.29
        GLIBC_PRIVATE
        ```
        

---

- **[Correct Results]**
    - `../configure --prefix=/usr/local/glibc-2.29/build --disable-sanity-checks` 하면 올바르게 나온 결과의 마지막 부분이 다음과 같아야함
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/anaconda_error1/Untitled.png" alt="">
        </figure>         
    
    ---
    
    - `make` 하면 올바르게 나온 결과의 마지막 부분이 다음과 같아야함
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/anaconda_error1/Untitled 1.png" alt="">
        </figure> 
    
    ---
    
    - `make install` 하면 올바르게 나온 결과의 마지막 부분이 다음과 같아야함
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/anaconda_error1/Untitled 2.png" alt="">
        </figure> 

---

- **[Reference Site]**
    
    - [Solved "GLIBC_2.29 not found" error on Ubuntu Linux \mid CyberITHub](https://www.cyberithub.com/solved-glibc-2-29-not-found-error-on-ubuntu-linux/)
    
    - [【请谨慎操作】Ubuntu18.04升级GLIBC_2.29，解决ImportError: /lib/x86_64-linux-gnu/libm.so.6: version `GLIBC_2.29‘-CSDN博客](https://blog.csdn.net/m0_37201243/article/details/123641552)
---
title: "[Anaconda Error] ImportError: ~/anaconda3/envs/ee837/bin/../lib/libgio-2.0.so.0: undefined symbol: g_unix_get_passwd_entry"
excerpt: "Anaconda Error 1"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Anaconda Error] ImportError: ~/anaconda3/envs/ee837/bin/../lib/libgio-2.0.so.0: undefined symbol: g_unix_get_passwd_entry

---

- **[Cause]**
    - Anaconda를 이용하면서 OpenCV를 사용하면 다음과 같은 error가 나타나는데 생각보다 자주 일어난다.
    - 해당 terminal 결과는 다음과 같이 나타난다.
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/anaconda_error1/Untitled.png" alt="">
        </figure> 

---

- **[Reference Site]**
    
    - [ImportError: lib/libgio-2.0.so.0: undefined symbol: g_unix_get_passwd_entry](https://www.jianshu.com/p/11027477097a)
    

---

- **[Solve]**
    - 해당 문제는 local에 설치되어 있는 OpenCV에 대한 library와 Anaconda에 설치되어 있는 library 경로가 **서로 겹치기 때문에 생기는 문제**가 된다.
    - 즉, 같은 library가 설치되어 있어 경로 설정을 해주거나 anaconda에 있는 libgio관련 라이브러리를 지워야한다.
        
        ```bash
        $ cd ~/anaconda3/env/${env_name}/lib
        $ ls | grep libgio
        $ rm -r libgio-2.0.so libgio-2.0.so.0 libgio-2.0.so.0.6901.0
        ```
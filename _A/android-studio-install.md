---
title: "[Android Studio] How to manually install App in Android Device?"
excerpt: "Manually install App in Android Device."
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Android Studio] How to manually install App in Android Device?

---

- **[Goal]**
    - Android Studio에서 build는 successfully 되었는데 핸드폰에 자동적으로 install이 되지 않을 때 cmd를 이용하여 설치할 수 있다.

---

- **[Reference Site]**
    - Install ADB Related
        
        - [QA 끝! ADB 설치부터 사용까지](https://labs.brandi.co.kr/2018/08/10/kimcy.html)
        
    - Install Manually App in device Related
        
        - [Android successful built gradle but not install apps on emulator](https://stackoverflow.com/a/52067804)
        

---

- **[Success Case]**
    - 다음과 같은 결과가 **모두 나와야 핸드폰에 정상적으로 install 된 경우**이다.
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled.png" alt="">
        </figure> 

---

- **[Problem]**
    - Success Case 처럼 결과가 모두 나오지 않은 경우 (자동적으로 핸드폰에 install이 되지 않는 경우)
        
        ```bash
        03/15 15:07:32: Launching 'app' on Device.
        ```
        
    - 중간에 install 할 때 다음과 같은 error가 나오는 경우
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 1.png" alt="">
        </figure> 

---

- **[Solution]**
    - **[Step 1] “adb.exe”가 있는 ADB 경로를 먼저 찾음**
        - ADB 경로
            - **C:\Users\${USER_NAME}\AppData\Local\Android\Sdk\platform-tools\adb.exe**
    
    ---
    
    - **[Step 2] adb가 window cmd에서 인식할 수 있도록 path 지정**
        - Path 경로
            - **검색 → 제어판 → 시스템 및 보안 → 시스템 → 고급 시스템 설정 → 환경 변수 → 시스템 변수 → Path → 편집 → 새로 만들기 → ADB 경로 (C:\Users\${USER_NAME}\AppData\Local\Android\Sdk\platform-tools) 추가**
            - 환경 변수 화면
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 2.png" alt="">
                </figure> 
                
            - ADB 경로 추가 화면
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 3.png" alt="">
                </figure> 
                    
    ---
    
    - **[Step 3] cmd에서 adb 인식 결과**
        - “window + R” → cmd → 확인
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 4.png" alt="">
            </figure>             
    
    ---
    
    - **[Step 4] Android Studio에서 build 진행**
        - **Android Studio → Build → Build Bundle(s) / APK(s) → Build APK(s)**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 5.png" alt="">
            </figure>             
    
    ---
    
    - **[Step 5] Android Studio에서 build 결과에 해당하는 apk (app-debug.apk) 찾기**
        - 해당 코드의 build 결과 경로
            - **${Code}\app\build\outputs\apk\debug\app-debug.apk**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 6.png" alt="">
            </figure> 
    
    ---
    
    - **[Step 6] 해당 apk를 adb.exe가 있는 위치로 복사**
        - **C:\Users\${USER_NAME}\AppData\Local\Android\Sdk\platform-tools**
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 7.png" alt="">
        </figure>                 
    
    ---
    
    - **[Step 7] cmd로 adb.exe가 있는 경로로 가서 수동으로 install**
        - 해당 경로로 이동
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 8.png" alt="">
            </figure> 
        
        ---
        
        - 현재 연결되어 있는 핸드폰이 있는지 검색 (**$ adb devices**)
            - 다음과 같은 결과가 나와야 정상적으로 연결되어 있음
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 9.png" alt="">
                </figure> 
        
        ---
        
        - 핸드폰이 정상적으로 연결이 되어 있다면 다음과 같은 명령어로 핸드폰에 수동 install
            
            ```bash
            **$ adb install app-debug.apk**
            ```
            
            - 다음과 같은 결과가 나오면 성공적으로 핸드폰에 설치 완료
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/android/Untitled 10.png" alt="">
                </figure> 
                               
---

- **[Error Related]**
    - **[Error 1] Android Installation error: INSTALL_FAILED_UPDATE_INCOMPATIBLE**
        - Reference Site
            
            - [Android Installation error: INSTALL_FAILED_UPDATE_INCOMPATIBLE](https://zerodice0.tistory.com/118)
            
        - **Solution) 핸드폰의 어플을 지우고 다시 설치하면 됨**
    
    ---
    
    - **[Error 2] adb: failed to install app-release.apk: Failure INSTALL_FAILED_TEST_ONLY: installPackageLI**
        - Reference Site
            
            - [안드로이드 - INSTALL_FAILED_TEST_ONLY 에러 해결방법](https://codechacha.com/ko/how-to-disable-testonly-in-android/)
            
        - **Solution) RUN을 하게 되면 test용 apk가 생성되기 때문에 build만 진행시켜주고 그 결과물(app-debug.apk)을 install하면 됨**
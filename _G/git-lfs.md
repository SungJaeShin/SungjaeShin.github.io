---
title: "[Git LFS] File size exceeds GitHub origin upload size"
excerpt: "Git Large File Storage"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Git LFS] File size exceeds GitHub origin upload size

## [Goal] GitHub 에는 일반적으로 30MB 이상의 file이 upload 되지 않아서 이를 해결하는 방법에 대해 살펴보기

---

- **[Summary]**
    - GitHub 에는 일반적으로 100MB 이하의 파일들만 upload 가 가능하다.
    - 만약, 100MB 이상의 파일을 upload 하고 싶다면 Git에서 제공하는 **대용량 저장소를 사용**한다.
        - 설치하는 package 이름은 Git Large File Storage 줄여서 **“Git LFS”** 이다.

---

- **[Reference Site]**
    
    - [[Git] Ubuntu 16.04에서 Git LFS(Large File Storage) 사용법](https://miiingo.tistory.com/333)
    

---

- **[Installation and Upload]**
    - Go to follow web site: [https://packagecloud.io/github/git-lfs/install](https://packagecloud.io/github/git-lfs/install)
        
    - Then following as:
        
        ```bash
        # Copy above web site
        $ curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
        
        # Install Git LFS
        $ sudo apt-get install git-lfs
        
        # Go to upload directory that has file over 30MB
        $ cd $(upload directory)
        $ git lfs install
        
        # For example file name is "example.gif" in src folder 
        $ git lfs track "./src/*.gif"
        
        # add .gitattributes
        $ git add .gitattributes
        
        # Then same other procedure follow as:
        $ git commit -m "~"
        $ git push origin main
        ```
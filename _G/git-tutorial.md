---
title: "[Git] Basic Concept and Information"
excerpt: "Git Tutorial"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Git] Basic Concept and Information

## [Goal] Git을 사용하는데 있어 필수적으로 사용되거나 알아야 할 다양한 기능들에 대해서 알아본다.

---

- **[Reference Site]**
    - 맨 처음에 기초적으로 사용되는 기능들에 대해서 참고하기 좋은 사이트인 것 같다.
        - [https://backlog.com/git-tutorial/kr/stepup/stepup2_2.html](https://backlog.com/git-tutorial/kr/stepup/stepup2_2.html)
    

---

- **[사용되는 기능 및 명령어]**
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/git/Untitled.png" alt="">
    </figure> 
    
    - 출처: [https://support.nesi.org.nz/hc/en-gb/articles/360001508515-Git-Reference-Sheet](https://support.nesi.org.nz/hc/en-gb/articles/360001508515-Git-Reference-Sheet)
    
    ```bash
    # Make first local repository 
    $ git init
    
    # To see all branch corresponding this package
    $ git branch -a 
    
    # To make another branch corresponding this package
    $ git branch <own_name>
    
    # Move to another branch in corresponding this package
    $ git checkout <branch_name> 
    
    # Add all changes(modified files) to current branch 
    $ git add .
    
    # Add some files to current branch
    $ git add README.md
    
    # Register modified files to remote storage
    $ git commit -m "Message Content" 
    
    # Save and upload modified files to remote storage
    $ git push origin <branch_name>
    
    # See git status
    $ git status
    
    # See git history
    $ git log
    
    # To go pull another branch in origin root
    $ git reset --hard (6~7 words)
    
    # Get remote repository to update
    $ git pull 
    ```
    

---

- **[주의 사항]**
    - **git clone을 하게 되면 특정 version branch가 아닌 main branch가 clone이 된다.** 그래서 우리는 특정 branch로 이동하기 위해서 다음과 같이 변경해주어야 한다.
        - Example of "rosbot-stm32-firmware" in "lib" folder
        
        ```bash
        # First download default main 
        $ git clone https://github.com/byq77/rosserial-mbed.git
        
        # Then convert to which person want to see
        $ git reset --hard 509e7c1
        ```
        
    
    ---
    
    - 이제는 git clone을 해서 username & password를 사용하는 것이 아니라, **SSH key를 만들어서 접속**한다.
        - 공개 key와 private key를 만들어서 이것들을 USB에 담아서 다른 저장장치에 사용한다.
            - Reference Site: [https://jootc.com/p/201905122827](https://jootc.com/p/201905122827)
            
            - Reference Site: [https://insight.infograb.net/docs/user/ssh_keys/](https://insight.infograb.net/docs/user/ssh_keys/)
                        
            - Reference Site: [https://sidorl.tistory.com/52](https://sidorl.tistory.com/52)
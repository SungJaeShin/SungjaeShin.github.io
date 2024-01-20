---
title: "[SSH] How to access another computer"
excerpt: "Remote Access"
---
# [SSH] How to access another computer

- **[Settings]**
    - 원격 접속하려는 computer는 **같은 master IP(Router)를 공유**해야 한다.
    - 같은 router에 접속하여 자신의 IP 주소를 확인한다.
        
        ```bash
        $ ifconfig
        ```
        

---

- **[Install]**
    
    ```bash
    $ sudo apt update
    $ sudp apt install openssh-server
    $ sudo service ssh start
    ```
    

---

- **[Access another computer]**
    
    ```bash
    # Example of ssh: ssh cps@192.168.0.2
    $ ssh username@ip_address
    
    ------------------------------------------------------------------------------
    
    # For easy to use ssh
    $ gedit ~/.bashrc
    $ alias (name)='ssh -X username@ip_address'
    $ name
    
    # Example of using bashrc
    $ gedit ~/.bashrc
    $ alias robot1='ssh -X cps@192.168.0.2'
    $ robot1
    ```
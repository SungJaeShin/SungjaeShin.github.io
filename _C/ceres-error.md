---
title: "[Ceres Solver Test Error] cuda_memcheck_dense_~ error"
excerpt: "Ceres Solver Error"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Ceres Solver Test Error] cuda_memcheck_dense_~ error

---

- **[Goal]**
    - Ceres Solver를 설치하는 과정 중 test에 생기는 error를 해결할 수 있다.

---

- **[Problem]**
    - Ceres Solver를 데스크탑에서 설치하면 문제가 없었는데 Jetson Board (본 저자는 Xavier AGX사용)를 사용할 때 make test 시 다음과 같은 error가 발생하였다.
        - Error List
            
            ```bash
            # USE Error log
            $ ctest -C Release --output-on-failure --rerun-failed -VV
            
            # Output
            The following tests FAILED:
            					1 - cuda_memcheck_dense_qr_test (Failed)
            					2 - cuda_memcheck_dense_cholesky_test (Failed)
            Errors while running CTest
            ```
            

---

- **[Reference Site]**
    
    - [cuda_memcheck_* and covariance_test tests failed on VS 2022, ceres 2.1.0rc1 · Issue #769 · ceres-solver/ceres-solver](https://github.com/ceres-solver/ceres-solver/issues/769)
    
    - [Nvidia NX ceres2.1.0 build OK, test with error. - bytemeta](https://bytemeta.vip/repo/ceres-solver/ceres-solver/issues/813)
    
    - [https://ceres-solver-review.googlesource.com/c/ceres-solver/+/21760](https://ceres-solver-review.googlesource.com/c/ceres-solver/+/21760)
    
    - [https://ceres-solver-review.googlesource.com/c/ceres-solver/+/21761/1/internal/ceres/CMakeLists.txt](https://ceres-solver-review.googlesource.com/c/ceres-solver/+/21761/1/internal/ceres/CMakeLists.txt)
    

---

- **[Cause]**
    - [Cause 1] 이전 CUDA memcheck test는 **“environment PATH”**에 영향을 받았는데 변경된 버전에서는 **“CMake가 발견한 경로”**를 사용하도록 변경되었다.
        - 따라서 ceres 설치 시, CMakeList.txt에 올바른 경로를 넣어주어야 한다.
    
    ---
    
    - [Cause 2] CUDA가 bash 안에서 export가 되었는지 확인이 필요하다. Install 시 ${PATH} & ${LD_LIBRARY_PATH} 안에 cuda가 들어있어야 해당 경로를 인식한다.
    
    ---
    
    - [Cause 3] Jetson 디버그 모드에서 cuda를 실행할 수 있는 권한이 설정되었는지 체크해야한다. 그렇지 않으면 디버그 모드 시 사용되지 않는다.

---

- **[Solution 1]**
    - [Cause 1]에 대한 해결방안
        - [Step 1] 먼저, ceres 설치를 위한 CMakeList.txt 파일을 찾는다.
            
            ```bash
            # Go To CERES CMakeList.txt
            $ cd ceres-solver-2.1.0/**internal/ceres/CMakeList.txt**
            ```
            
        
        ---
        
        - [Step 2] 137 & 141번째 줄을 다음과 같이 변경한다.
            
            ```bash
            [# Change 137 & 141 Line following as]
            ## BEFORE
            COMMAND cuda-memcheck --leak-check full
            
            ## AFTER
            COMMAND **${CUDA_TOOLKIT_ROOT_DIR}/bin/**cuda-memcheck --leak-check full
            ```
            

---

- **[Solution 2]**
    - “Solution 1”의 방법이 안된다면 다음 방법을 사용한다.
    - [Cause 2]에 대한 해결방안
        - [Step 1] Terminal 창의 bash 설정을 살펴본다.
            - bash 안에 다음과 같은 명령어가 있는지 찾아본다.
                
                ```bash
                # gedit ~/.bashrc
                export PATH=/usr/local/cuda-10.2/bin:$PATH
                export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
                ```
                
                - 다음의 명령어가 없으면 해당 line을 추가한다.

---

- **[Solution 3]**
    - “Solution 2”의 방법이 안된다면 다음 방법을 사용한다.
    - [Cause 3]에 대한 해결방안
        - [Step 1] /dev 로 가서 nvhost-dbg-gpu가 있는지 체크한다. 그 후 permission 제한을 풀어준다.
            
            ```bash
            # Find nvhost-dbg-gpu
            $ cd /dev
            $ ls | grep nvhost-dbg-gpu
            
            # Change Permission
            $ sudo chmod 666 /dev/nvhost-dbg-gpu
            ```
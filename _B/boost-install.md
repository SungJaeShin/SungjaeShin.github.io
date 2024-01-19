---
title: "[Boost Install] How to install Boost package or change Boost version using zip file in local?"
excerpt: "Install specific Boost version"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Boost] How to install Boost package or change Boost version using zip file in local?

---

- **[Goal]**
    - Dependency가 다른 boost version을 알맞게 설치할 수 있다.

---

- **[Reference Site]**
    
    - [Uninstall boost and install another version](https://stackoverflow.com/questions/8430332/uninstall-boost-and-install-another-version)
    

---

- **[Process]**
    - [Step 1] Download Boost package
        
        - [Version 1.65.1](https://www.boost.org/users/history/version_1_65_1.html)
        
    - [Step 2] Unzip package and install Boost
        
        ```bash
        $ tar -zxvf boost_1_65_1.tar.gz
        $ cd boost_1_65_1/
        $ ./bootstrap.sh --prefix=/usr/local
        $ sudo ./b2 --prefix=/usr/local --with=all --install
        ```
        
        - **특정 라이브러리만 설치**하고 싶으면 `sudo ./b2 --prefix=/usr/local --with-python --with-regex --install` 등등 이렇게 뒤에 붙여주면 됨
        - **전체 라이브러리를 설치**하고 싶으면 `sudo ./b2 --prefix=/usr/local --with=all --install` 만 해주면됨
    - [Step 3] Check Boost version correctly
        
        ```bash
        $ cat /usr/local/include/boost/version.hpp | grep "BOOST_LIB_VERSION"
        //  BOOST_LIB_VERSION must be defined to be the same as BOOST_VERSION
        #define BOOST_LIB_VERSION "1_65_1"
        ```
        

---

- **[Change Boost version]**
    - [Step 1] Boost package를 download하기 전에 기존의 boost와 관련된 라이브러리 모두 제거 후 위의 과정을 다시 진행
        
        ```bash
        $ sudo apt-get -y --purge remove libboost-all-dev libboost-doc libboost-dev
        $ sudo rm -r /usr/lib/x86_64-linux-gnu/libboost_*
        ```
        

---

- **[Boost Install Error about libboost_python]**
    - Follow this page
        
        - [[Boost Error] gcc.compile.c++ bin.v2/libs/python/build/gcc-7/release/link-static/threading-multi/converter/builtin_converters.o libs/python/src/converter/builtin_converters.cpp: In function ‘void* boost::python::converter::{anonymous}::convert_to_cstring(PyObject*)’: libs/python/src/converter/builtin_converters.cpp:51:35: error: invalid conversion from ‘const void*’ to ‘void*’ [-fpermissive] return PyUnicode_Check(obj) ? _PyUnicode_AsString(obj) : 0; Built with Notion](https://sungjaeshin.github.io/B/boost-error1/)
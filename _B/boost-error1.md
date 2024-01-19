---
title: "[Boost Error] gcc.compile.c++ bin.v2/libs/python/build/gcc-7/release/link-static/threading-multi/converter/builtin_converters.o
libs/python/src/converter/builtin_converters.cpp: In function ‘void* boost::python::converter::{anonymous}::convert_to_cstring(PyObject*)’:
libs/python/src/converter/builtin_converters.cpp:51:35: error: invalid conversion from ‘const void*’ to ‘void*’ [-fpermissive]
return PyUnicode_Check(obj) ? _PyUnicode_AsString(obj) : 0;"
excerpt: "Boost Error 1"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Boost Error] gcc.compile.c++ bin.v2/libs/python/build/gcc-7/release/link-static/threading-multi/converter/builtin_converters.o libs/python/src/converter/builtin_converters.cpp: In function ‘void* boost::python::converter::{anonymous}::convert_to_cstring(PyObject*)’: libs/python/src/converter/builtin_converters.cpp:51:35: error: invalid conversion from ‘const void*’ to ‘void*’ [-fpermissive] return PyUnicode_Check(obj) ? _PyUnicode_AsString(obj) : 0;

---

- **[Goal]**
    - Boost를 python 버전과 함께 install 할 때 발생하는 문제를 해결할 수 있다.

---

- **[Reference Site]**
    
    - [Can't compile boost.python 1.65.1 with MSVC2015](https://stackoverflow.com/questions/54991157/cant-compile-boost-python-1-65-1-with-msvc2015)
    
    - [Fix build with Python 3.7 · boostorg/python@660487c](https://github.com/boostorg/python/commit/660487c43fde76f3e64f1cb2e644500da92fe582#diff-467cabb22a6c637452d730accca26d2e)
    

---

- **[Cause]**
    - Boost python 라이브러리를 설치할 때, python version에 따른 경로를 찾지 못하여 문제가 발생하였기 때문에 boost package를 일부 수정

---

- **[Process]**
    - [Step 1] boost package 내부 코드인 `~/boost_1_XX_X/libs/python/src/converter/builtin_converters.cpp`를 수정 **(Line 38 ~ 47)**
        - Before
            
            ```cpp
            // An lvalue conversion function which extracts a char const* from a
            // Python String.
            #if PY_VERSION_HEX < 0x03000000
              void* convert_to_cstring(PyObject* obj)
              {
                  return PyString_Check(obj) ? PyString_AsString(obj) : 0;
              }
            #else
              void* convert_to_cstring(PyObject* obj)
              {
                  return PyUnicode_Check(obj) ? _PyUnicode_AsString(obj) : 0;
              }
            ```
            
        - After
            
            ```cpp
            // An lvalue conversion function which extracts a char const* from a
            // Python String.
            #if PY_VERSION_HEX < 0x03000000
              void* convert_to_cstring(PyObject* obj)
              {
                  return PyString_Check(obj) ? PyString_AsString(obj) : 0;
              }
            #elif PY_VERSION_HEX < 0x03070000
              void* convert_to_cstring(PyObject* obj)
              {
                  return PyUnicode_Check(obj) ? _PyUnicode_AsString(obj) : 0;
              }
            #else
              void* convert_to_cstring(PyObject* obj)
              {
                  return PyUnicode_Check(obj) ? const_cast<void*>(reinterpret_cast<const void*>(_PyUnicode_AsString(obj))) : 0;
              }
            #endif
            ```
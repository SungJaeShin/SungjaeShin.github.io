---
title: "[Boost Error] libcv_bridge.so：undefined reference to ‘boost::re_detail_106900::perl_matcher<char const*, std::allocator<boost::sub_match<char const*> >, boost::regex_traits<char, boost::cpp_regex_traits<char> > >::construct_init(boost::basic_regex<char, boost::regex_traits<char, boost::cpp_regex_traits<char> > > const&, boost::regex_constants::_match_flags)"
excerpt: "Boost Error 2"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
---
# [Boost Error] libcv_bridge.so：undefined reference to ‘boost::re_detail_106900::perl_matcher<char const*, std::allocator<boost::sub_match<char const*> >, boost::regex_traits<char, boost::cpp_regex_traits<char> > >::construct_init(boost::basic_regex<char, boost::regex_traits<char, boost::cpp_regex_traits<char> > > const&, boost::regex_constants::_match_flags)

---

- **[Goal]**
    - cv_bridge와 연관되어 있는 Boost관련 에러를 해결할 수 있다.

---

- **[Reference Site]**
    
    - [undefined reference to `boost::re_detail::cpp_regex_traits_implementation · Issue #5449 · monero-project/monero](https://github.com/monero-project/monero/issues/5449)
    

---

- **[Process]**
    - [Step 1] workspace로 가서 dependency인 boost 경로가 어디에 있는지 체크
        - `grep -i boost find build -name CMakeCache.txt`
            
            ```bash
            build/cv_bridge/CMakeCache.txt:Boost_DIR:PATH=Boost_DIR-NOTFOUND
            build/cv_bridge/CMakeCache.txt:Boost_INCLUDE_DIR:PATH=/usr/local/include
            **build/cv_bridge/CMakeCache.txt:Boost_LIBRARY_DIR_DEBUG:PATH=/usr/lib/x86_64-linux-gnu**
            **build/cv_bridge/CMakeCache.txt:Boost_LIBRARY_DIR_RELEASE:PATH=/usr/lib/x86_64-linux-gnu**
            
            build/camera_model/CMakeCache.txt:Boost_DIR:PATH=Boost_DIR-NOTFOUND
            build/camera_model/CMakeCache.txt:Boost_INCLUDE_DIR:PATH=/usr/local/include
            **build/camera_model/CMakeCache.txt:Boost_LIBRARY_DIR_DEBUG:PATH=/usr/local/lib
            build/camera_model/CMakeCache.txt:Boost_LIBRARY_DIR_RELEASE:PATH=/usr/local/lib**
            ```
            
        - [Step 2] 저자의 경우 boost version이 2개가 동시에 설치가 되어 있기 때문에 사용하고자 하는 버전의 Boost absolute path를 export
            
            ```bash
            export BOOST_INCLUDE_DIR="/usr/local/include"
            export BOOST_LIBRARYDIR="/usr/local/lib"
            export BOOST_ROOT="/usr/include/boost"
            ```
            
        - [Step 3] `catkin clean` 후 다시 `catkin build` 진행
            - **추가로 이렇게 했는데도 변경된 버전으로 build되지 않은 경우 boost 버전 중 하나를 제거 !!!**
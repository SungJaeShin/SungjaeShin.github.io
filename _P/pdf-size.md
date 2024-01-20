---
title: "[PDF Size] How to make small size of pdf in ubuntu"
excerpt: "Change PDF Size"
---
# [PDF Size] How to make small size of pdf

- **[Terminal]**
    
    ```bash
    $ gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.7 -dPDFSETTINGS=/prepress -dPrinted=false -dNOPAUSE -dQUIET -dBATCH -sOutputFile=root_small.pdf root.pdf
    ```
    
    - PDF size가 크다면 이미지 파일이 최대 PDF와 같은 해상도를 가지고 줄여주는 행위
---
title: "[Terminal Color Text] How to put coloured text  in Linux terminal?"
excerpt: "Terminal Color"
---
# [Terminal Color Text] How to put coloured text  in Linux terminal?

---

- **[Reference Site]**
    
    - [ANSI escape code - Wikipedia](https://en.wikipedia.org/wiki/ANSI_escape_code#Colors)
    
    - [How do I output coloured text to a Linux terminal?](https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal)
    

---

- **[Details about Linux Terminal]**
    
    
    |  | foreground | background |
    | --- | --- | --- |
    | black | 30 | 40 |
    | red | 31 | 41 |
    | green | 32 | 42 |
    | yellow | 33 | 43 |
    | blue | 34 | 44 |
    | magenta | 35 | 45 |
    | cyan | 36 | 46 |
    | white | 37 | 47 |

---

- **[Example of coloured text in c++]**
    
    ```cpp
    std::cout << "\033[1;31mbold red text\033[0m\n";
    ```
---
title: "[Marching Squares: An Implement Marching Squares using OpenCV in C++]"
classes: wide
---

## Github Code 
[![Github_Page]({{ site.url }}{{ site.baseurl }}/assets/images/github_logo.png)](https://github.com/SungJaeShin/Marching_squares.git)*Click Image!*

## [Goal] Implement Marching Squares using OpenCV in C++
- Randomly generated grid image \
  <figure class="align-center">
    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/init_img.png" alt="">
  </figure> 

## 1. Prerequisites
### 1.1 Dependencies
OpenCV 3.2.0, C++ 11 version

### 1.2. OpenCV Installation
Follow [OpenCV](https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html)
- Install appropriate OpenCV version: [Here](https://sungjaeshin.github.io/O/opencv-install/).

## 2. Changing Parameters
### Parameters in "parameters.h"
```
// Init Image Size
#define WIDTH  640
#define HEIGHT 480

// For making interval of grid in image
#define DIFF_X 10
#define DIFF_Y 10
```

## 3. Build and Run 
Clone the repository and build and run simultaneously:
```
   $ cd ${workspace}
   $ git clone https://github.com/SungJaeShin/Marching_squares.git
   $ cd Marching_squares
   $ sh start.sh
```

## 4. Setting Rules between Vertices and Edges !!
```
  p0           a           p1
     ○ ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ ○
     |                   |
     |                   |
  d  |                   |   b
     |                   |
     |                   |
     |                   |
     ○ ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ ○
 p3            c           p2

    Vertex : p0, p1, p2, p3
    Edge   : a, b, c, d
    Rules  : {p0, p1, p2, p3} = {0, 0, 1, 0} <-----> {a, b, c, d} = {0, 1, 1, 0}

    // Not Make Any Line
    {0, 0, 0, 0} <-----> {0, 0, 0, 0} = 0
    {1, 1, 1, 1} <-----> {0, 0, 0, 0} = 0

    // Make Horizontal & Vertical Line
    {1, 1, 0, 0} <-----> {0, 1, 0, 1} = 5
    {0, 0, 1, 1} <-----> {0, 1, 0, 1} = 5
    {0, 1, 1, 0} <-----> {1, 0, 1, 0} = 10
    {1, 0, 0, 1} <-----> {1, 0, 1, 0} = 10

    // Make One Diagonal Line 
    {1, 0, 0, 0} <-----> {1, 0, 0, 1} = 9
    {0, 1, 1, 1} <-----> {1, 0, 0, 1} = 9
    {0, 1, 0, 0} <-----> {1, 1, 0, 0} = 12
    {1, 0, 1, 1} <-----> {1, 1, 0, 0} = 12
    {0, 0, 1, 0} <-----> {0, 1, 1, 0} = 6
    {1, 1, 0, 1} <-----> {0, 1, 1, 0} = 6
    {0, 0, 0, 1} <-----> {0, 0, 1, 1} = 3
    {1, 1, 1, 0} <-----> {0, 0, 1, 1} = 3

    // Make Two Diagonal Line
    p0          a           p1              p0          a           p1
        ● ㅡ ● ㅡㅡㅡㅡㅡㅡ ○                      ○ ㅡㅡㅡㅡㅡ ● ㅡㅡㅡ ●
        |  /             |                      |           \     |
        | /              |                      ●            \    |
    d   |/               ●   b              d   | \           \   |   b
        ●               /|                      |  \           \  |
        |              / |                      |   \             ●
        |             /  |                      |    \            |
        ○ ㅡㅡㅡㅡㅡㅡ ● ㅡ ●                      ● ㅡㅡ ● ㅡㅡㅡㅡㅡㅡ ○
    p3          c           p2              p3          c           p2

    {0, 1, 0, 1} <-----> {1, 1, 1, 1} = 15
    {1, 0, 1, 0} <-----> {1, 1, 1, 1} = 16
```

  <table>
      <tr>
         <td> Case 1 : Not Make Any Line </td>
         <td> 
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/not_draw.jpg" alt="">
         </figure> 
         </td>
      </tr>
      <tr>
         <td> Case 2 : Make Horizontal & Vertical Line </td>
         <td> 
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/horizontal_vertical_line.jpg" alt="">
         </figure> 
         </td>
      </tr>
      <tr> 
         <td> Case 3 : Make One Diagonal Line </td>
         <td> 
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/one_diagonal_line.jpg" alt="">
         </figure> 
         </td>
      </tr>
      <tr>
         <td> Case 4 : Make Two Diagonal Line  </td>
         <td>
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/two_diagonal_lines.jpg" alt="">
         </figure> 
         </td>
      </tr>
  </table>

## 5. Results 
<table>
    <tr>
       <td> Result w/ vertices (grid interval: 10) </td>
       <td> Result w/o vertices (grid interval: 10) </td>
    </tr>
    <tr>
       <td>
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_grid_img.png" alt="">
        </figure> 
       </td>
       <td>
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_line_img.png" alt="">
        </figure> 
      </td>
    </tr>
    <tr>
       <td> Result w/ vertices (grid interval: 20) </td>
       <td> Result w/o vertices (grid interval: 20) </td>
    </tr>
    <tr>
      <td>
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_grid_img_20.png" alt="">
        </figure> 
       </td>
       <td>
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_line_img_20.png" alt="">
        </figure> 
      </td>
    </tr>
    <tr>
       <td> Result w/ vertices (grid interval: 30) </td>
       <td> Result w/o vertices (grid interval: 30) </td>
    </tr>
    <tr>
        <td>
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_grid_img_30.png" alt="">
        </figure> 
       </td>
       <td>
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_line_img_30.png" alt="">
        </figure> 
      </td>
    </tr>
    <tr>
       <td> Result w/ vertices (grid interval: 40) </td>
       <td> Result w/o vertices (grid interval: 40) </td>
    </tr>
    <tr>
        <td>
         <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_grid_img_40.png" alt="">
        </figure> 
       </td>
       <td>
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/marching_squares/marching_line_img_40.png" alt="">
        </figure> 
      </td>
    </tr>
</table>

## 6. References
[1] [https://bbtarzan12.github.io/Marching-Squares-Part-1/](https://bbtarzan12.github.io/Marching-Squares-Part-1/) \
[2] [https://github.com/tmryan/MarchingSquares/tree/master](https://github.com/tmryan/MarchingSquares/tree/master)
---
title: "[Marching Squares] How to make 2D surface in scalar field?"
excerpt: "Understand Marching Squares Algorithm"
---
# [Marching Squares] How to make 2D surface in scalar field?

---

- **[Goal]**
    - Computer Graphics에 나오는 기본적인 mesh generation 기법 중 하나인 marching squares을 이해하고 임의의 grid에서 구현할 수 있다.

---

- **[Marching Squares]**
    - Definition
        - Scalar field에서 2D surface reconstruction을 위한 Computer Graphics 기초 기법
    
    ---
    
    - Brief Process
        - Vertex의 density가 있을 경우, 이를 지나는 선을 이어 표면을 생성한다.
        - 2D 표면을 생성할 때, Vertex → Edge Table(Rule)을 setting하고 bit연산을 사용한다.
    
    ---
    
    - Process
        - [Step 0] Assume Vertex and Edge
            - 다음과 같은 그림을 가정
                <figure class="align-center">
                  <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled.jpeg" alt="">
                </figure>                   
        
        ---
        
        - [Step 1] Make Random Grid
            - 당연히 image를 사용하여 surface를 만들 수 있지만 훨씬 간편하게 implement하기 위해서 임의의 grid를 만드는 방식을 적용
                
                ```cpp
                cv::Mat gen_grid_img(int width, int height)
                {
                    // Init Grid Img
                    cv::Mat grid_img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
                
                    // Make Random Grid Img
                    std::srand(time(0)); 
                    for(int y = 1; y < height - 1; y++)
                    {
                        for(int x = 1; x < width - 1; x++)
                        {
                            if(x % DIFF_X == 0 && y % DIFF_Y == 0)
                            {
                                double value = (std::rand() % 2) * 255;
                                double r, g, b;
                                r = g = b = value;
                
                                cv::Vec3b rgb(r, g, b);
                                grid_img.at<cv::Vec3b>(y, x) = rgb;
                            }
                        }
                    }
                        
                    return grid_img;
                }
                ```
                
        
        ---
        
        - [Step 2] Find Vertex density
            - Grid 안에 point들이 몇개가 차있는지 check하는 과정
            - Rule이 있기 때문에 어느 위치의 Pixel density가 차있는지 중요하고 순서 또한 중요하다.
                - Rule → {p0, p1, p2, p3} ↔ xxxx {x = 0 or 1}
        
        ---
        
        - [Step 3] Make Rule
            - Vertex density 에 따라 사용되는 edge를 역시 2진수로 만들어서 변환하는 작업이 필요하다.
            - 변환하기 위해서는 다음과 같은 rule setting 이 필요하다.
                - Vertex가 총 4개가 있으니깐 2x2x2x2 = 16개의 경우가 나올 수 있다.
                    - 선이 없는 경우
                      <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 1.jpeg" alt="">
                      </figure>   
                        
                    - 1개의 대각선을 만드는 경우
                      <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 2.jpeg" alt="">
                      </figure>   
                                                
                    - 가로 or 세로 선을 만드는 경우
                      <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 3.jpeg" alt="">
                      </figure>   
                        
                    - 2개의 대각선을 만드는 경우
                      <figure class="align-center">
                        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 4.jpeg" alt="">
                      </figure>   
                                
        ---
        
        - [Step 4] Edge Case를 find하여 선을 그리는 작업 진행
            - 이때, 원래의 경우에는 isovalue를 설정해서 등선에 대해서 유동적으로 linear interpolation을 진행해주는데 저자는 두 점의 중점으로 설정하였음

---

- **[Results]**
    - Generate Random Grid Image
        <figure class="align-center">
          <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled.png" alt="">
        </figure>           
    
    ---
    
    - [Case 1] Grid interval = 10
        <table>
        <tr>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 1.png" alt="">
          </figure>     
        </td>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 2.png" alt="">
          </figure>  
        </td>
        </tr>
        </table>          
    
    ---
    
    - [Case 2] Grid interval = 20
        <table>
        <tr>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 3.png" alt="">
          </figure>     
        </td>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 4.png" alt="">
          </figure>  
        </td>
        </tr>
        </table>          
    
    ---
    
    - [Case 3] Grid interval = 30
        <table>
        <tr>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 5.png" alt="">
          </figure>     
        </td>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 6.png" alt="">
          </figure>  
        </td>
        </tr>
        </table>          
    
    ---
    
    - [Case 4] Grid interval = 40
        <table>
        <tr>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 7.png" alt="">
          </figure>     
        </td>
        <td>
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 8.png" alt="">
          </figure>  
        </td>
        </tr>
        </table>  
        

---

- **[Problems]**
    - Ambiguity
        - 단순하고 빠른 2D surface reconstruction이지만 다음 그림과 같은 모호성이 존재한다.
          <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/marching_squares/Untitled 5.jpeg" alt="">
          </figure>  
            
        - 만약, 해당 문제를 수정하지 않으면 정확한 surface reconstruction 복원이 어려우나 상세한 case 구현만 해주면 해당 문제는 해결이 가능해 보인다.

---

- **[Further Study]**
    - Marching Squares를 확장시킨 알고리즘인 Marching Cubes를 구현할 수 있다.
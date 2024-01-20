---
title: "[Delaunay Triangulation] For 2D or 3D Mesh Generation"
excerpt: "Make basic delaunay triangulation"
# last_modified_at: 2022-08-01T18:35:05-04:00
# toc: true
classes: wide
---
# [Delaunay Triangulation] For 2D or 3D Mesh Generation

---

- **[Goal]**
    - Visual-Inertial Algorithm에서는 delaunay triangulation으로 3D mesh 및 reconstruction을 할 수 있다.

---

- **[Reference Site]**
    - Paper
        
        (1) Jim Ruppert, “A Delaunay Refinement Algorithm for Quality 2-Dimensional Mesh Generation,” *Journal of Algorithms,* pp.548–585, May 1995.
        
        (2) Jonathan Richard Shewchuk, "Triangle: Engineering a 2D Quality Mesh Generator
        and Delaunay Triangulator," *Applied Computational Geometry Towards Geometric Engineering,* pp.203–222, 1996.
        
    
    ---
    
    - Site
        
        (1) [https://en.wikipedia.org/wiki/Delaunay_triangulation](https://en.wikipedia.org/wiki/Delaunay_triangulation)
        
        (2) [https://en.wikipedia.org/wiki/Bowyer–Watson_algorithm](https://en.wikipedia.org/wiki/Bowyer%E2%80%93Watson_algorithm)
        
        (3) [https://hellowoori.tistory.com/30](https://hellowoori.tistory.com/30)
        
    
    ---
    
    - Open Source
        
        - [GitHub - bl4ckb0ne/delaunay-triangulation: C++ version the delaunay triangulation](https://github.com/Bl4ckb0ne/delaunay-triangulation)
        
        - [GitHub - robustrobotics/flame: FLaME: Fast Lightweight Mesh Estimation](https://github.com/robustrobotics/flame)
        

---

- **[Preliminary]**
    - 알아야 할 용어들
        
        (1) Vertex, Segment, Edge, Polygon
        
        (2) Delaunay Triangulation
        

---

- **[Delaunay Triangulation]**
    - Computational Geometry에서 주로 사용되는 용어로서 Delaunay Triangulation은 Delone Triangulation이라고도 부름
    - Delaunay Triangulation은 discrete points $P$가 있을 때, 결과는 다음과 같이 표기됨; $\mathcal{DT}(P)$
    
    ---
    
    - Delaunay Triangulation은 다음과 같은 조건들이 만족하는 알고리즘
        
        **(1) 삼각형 안에 다른 (외접) point들이 없어야 함** 
        
        **(2) 삼각형을 이루는 3개의 각들이 모두 최소 각 $\alpha$ 보다 커야함 (maximize acute or skinny angle)**
        
        - Refinement 알고리즘에서는 약 30도정도 이상을 설정한다고 작성되어 있음
    
    ---
    
    - Delaunay Triangulation이 제대로 작성되지 않은 경우가 다음과 같음
        
        **(1) 하나의 line에 동일한 point들이 여러개가 있다면 $\mathcal{DT}$ 생성 안됨** 
        
        **(2) Point 3개로 외접원을 만들었는데 그 외접원에 다른 point도 들어가 있는 경우** 
        

---

- **[Overall Methodology]**
    - Part 1. Generate Delaunay Triangulation
        - 먼저 Delaunany triangulation을 생성하는데 acute한 각이 생성되도 무관하게 generation
            - 이 삼각형을 silver triangle이라고 부름 **(하나 또는 두개의 각이 극도록 예각인 삼각형)**
        
        ---
        
        - 기본적인 Delaunay Triangulation 생성 알고리즘은 Bowyer-Watson 알고리즘을 이용함
            - **알고리즘 brief한 개요**
                
                (1) 이미지 상 keypoint 추출 (이 point들을 vertex의 복수형인 vertices 라고 부름)
                
                (2) 그 point들을 이용하여 super-triangle 생성; 모든 vertices들을 포함하는 가장 큰 삼각형
                
                (3) SuperTriangle을 이용하여 새로운 vertex 가 들어오면 외심의 성질을 이용하여 point가 삼각형 안에 있는지 체크 (이 방법을 사용)
                
                - [[Python] 좌표공간에 주어진 삼각형에 외접하는 원의 중심구하기](https://hiddenbeginner.github.io/python/mathematicalprogramming/2019/08/29/%ED%8C%8C%EC%9D%B4%EC%8D%AC%EC%9C%BC%EB%A1%9C_%EC%A2%8C%ED%91%9C%EA%B3%B5%EA%B0%84%EC%97%90_%EC%A3%BC%EC%96%B4%EC%A7%84_%EC%82%BC%EA%B0%81%ED%98%95%EC%97%90_%EC%99%B8%EC%A0%91%ED%95%98%EB%8A%94_%EC%9B%90%EC%9D%98_%EC%A4%91%EC%8B%AC_%EA%B5%AC%ED%95%98%EA%B8%B0.html)
                
                (4) 그래서 안에 들어와 있으면 해당 triangle 지우고 vertex를 포함하여 triangle을 다시 생성
                
            
            ---
            
            - 해당 코드는 여기! **(delaunay-triangulation/dt/delaunay.cpp)**
                - [https://github.com/Bl4ckb0ne/delaunay-triangulation](https://github.com/Bl4ckb0ne/delaunay-triangulation)
                    
                    ```cpp
                    template<typename T>
                    const std::vector<typename Delaunay<T>::TriangleType>&
                    Delaunay<T>::triangulate(std::vector<VertexType> &vertices)
                    {
                    	// Store the vertices locally
                    	_vertices = vertices;
                    
                    	// Determinate the super triangle
                    	T minX = vertices[0].x;
                    	T minY = vertices[0].y;
                    	T maxX = minX;
                    	T maxY = minY;
                    
                    	for(std::size_t i = 0; i < vertices.size(); ++i)
                    	{
                    		if (vertices[i].x < minX) minX = vertices[i].x;
                    		if (vertices[i].y < minY) minY = vertices[i].y;
                    		if (vertices[i].x > maxX) maxX = vertices[i].x;
                    		if (vertices[i].y > maxY) maxY = vertices[i].y;
                    	}
                    
                    	const T dx = maxX - minX;
                    	const T dy = maxY - minY;
                    	const T deltaMax = std::max(dx, dy);
                    	const T midx = (minX + maxX) / 2;
                    	const T midy = (minY + maxY) / 2;
                    
                    	const VertexType p1(midx - 20 * deltaMax, midy - deltaMax);
                    	const VertexType p2(midx, midy + 20 * deltaMax);
                    	const VertexType p3(midx + 20 * deltaMax, midy - deltaMax);
                    
                    	// Create a list of triangles, and add the supertriangle in it
                    	_triangles.push_back(TriangleType(p1, p2, p3));
                    
                    	for(auto p = begin(vertices); p != end(vertices); p++)
                    	{
                    		std::vector<EdgeType> polygon;
                    
                    		for(auto & t : _triangles)
                    		{
                    			if(t.circumCircleContains(*p))
                    			{
                    				t.isBad = true;
                    				polygon.push_back(Edge<T>{*t.a, *t.b});
                    				polygon.push_back(Edge<T>{*t.b, *t.c});
                    				polygon.push_back(Edge<T>{*t.c, *t.a});
                    			}
                    		}
                    
                    		_triangles.erase(std::remove_if(begin(_triangles), end(_triangles), [](TriangleType &t){
                    			return t.isBad;
                    		}), end(_triangles));
                    
                    		for(auto e1 = begin(polygon); e1 != end(polygon); ++e1)
                    		{
                    			for(auto e2 = e1 + 1; e2 != end(polygon); ++e2)
                    			{
                    				if(almost_equal(*e1, *e2))
                    				{
                    					e1->isBad = true;
                    					e2->isBad = true;
                    				}
                    			}
                    		}
                    
                    		polygon.erase(std::remove_if(begin(polygon), end(polygon), [](EdgeType &e){
                    			return e.isBad;
                    		}), end(polygon));
                    
                    		for(const auto e : polygon)
                    			_triangles.push_back(TriangleType(*e.v, *e.w, *p));
                    
                    	}
                    
                    	_triangles.erase(std::remove_if(begin(_triangles), end(_triangles), [p1, p2, p3](TriangleType &t){
                    		return t.containsVertex(p1) || t.containsVertex(p2) || t.containsVertex(p3);
                    	}), end(_triangles));
                    
                    	for(const auto t : _triangles)
                    	{
                    		_edges.push_back(Edge<T>{*t.a, *t.b});
                    		_edges.push_back(Edge<T>{*t.b, *t.c});
                    		_edges.push_back(Edge<T>{*t.c, *t.a});
                    	}
                    
                    	return _triangles;
                    }
                    ```
                    
        
        ---
        
        - **Results**
            <figure class="align-center">
                <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/delaunay/Untitled.gif" alt="">
            </figure> 
            
			<figure>
  				<img src="{{ '/assets/images/blog/delaunay/Untitled.gif' | relative_url }}" alt="creating a new branch on GitHub">
			</figure>

			![results]({{ "/assets/images/blog/delaunay/Untitled.gif" | relative_url }})

            - **[문제] 보면 silver triangle이 생성되는 것이 보임**
                
                → 그래서 acute 한 각을 setting하는 것이 필요!!; Delaunay Triangulation Refinement 알고리즘 진행!! 
                
    
    ---
    
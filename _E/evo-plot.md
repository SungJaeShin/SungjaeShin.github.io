---
title: "[evo plot] Plotting package for odometry"
excerpt: "Visualization and ATE & RTE calculation package"
---
# [evo plot] Plotting package for odometry

## [Goal] euroc dataset 의 형식이나 kitti dataset 형식들의 결과를 논문이나 보고를 드릴 때 사용하는 방법

---

- **[Github]**
    - Reference Site: [https://github.com/MichaelGrupp/evo](https://github.com/MichaelGrupp/evo)
        

---

- **[Reference Site]**
    - (1) Virtual Env install
        - Reference Site: [https://github.com/MichaelGrupp/evo/blob/ 9ec1444a9f26a24261e1c126cd4ee2d7877cc395/doc/install_in_virtualenv.md](https://github.com/MichaelGrupp/evo/blob/9ec1444a9f26a24261e1c126cd4ee2d7877cc395/doc/install_in_virtualenv.md)
            
            
    - (2) evo format
        - Reference Site: [https://github.com/MichaelGrupp/evo/wiki/Formats](https://github.com/MichaelGrupp/evo/wiki/Formats)
            
            
    - (3) evo plotting setting
        - Reference Site: [https://github.com/MichaelGrupp/evo/wiki/Plotting](https://github.com/MichaelGrupp/evo/wiki/Plotting)

            
    - (4) plot multiple results
        - Reference Site: [https://github.com/MichaelGrupp/evo/issues/137](https://github.com/MichaelGrupp/evo/issues/137)
            
            
    - (5) change background plot results
        - Reference Site: [https://github.com/MichaelGrupp/evo/issues/14](https://github.com/MichaelGrupp/evo/issues/14)


---

- **[Error]**
    - (1) "ValueError: could not convert string to float" & "TUM trajectory files must have 8 entries per row and no trailing delimiter at the end of the rows (space)"
        - A) convert .csv file to .tum file
            
            ```bash
            # Convert .csv to .tum
            $ evo_traj euroc ~.csv --save_as_tum
            ```
            

---

- **[Plot trajectory]**
    
    ```bash
    # Example of plot trajectory
    $ evo_traj tum Ground_Truth.tum Visual_Inertial_Odometry.tum -p --plot_mode=xy
    ```
    

---

- **[Plot Absolute Pose Error (APE)]**
    
    ```bash
    # It must compare two .tum files (In this case, Ground_truth and VIO) 
    $ evo_ape tum Ground_Truth.tum Visual_Inertial_Odometry.tum --plot --plot_mode=xy
    ```
    

---

- **[Plot Relative Pose Error (RPE)]**
    
    ```bash
    # It must compare two .tum files (In this case, Ground_truth and VIO) 
    $ evo_rpe tum Ground_Truth.tum Visual_Inertial_Odometry.tum --plot --plot_mode=xy
    ```
    

---

- **[Plot multiple results from a metric]**
    - before start, we must select compare result which chosen APE or RPE !!
    - Make each zip file first
        
        ```bash
        # For example, Let's assume compare APE
        $ evo_ape tum Ground_truth.tum Case_1.tum --save_results Case1.zip
        $ evo_ape tum Ground_truth.tum Case_2.tum --save_results Case2.zip
        $ evo_ape tum Ground_truth.tum Case_3.tum --save_results Case3.zip
        
        ```
        
    - Then plot all cases
        
        ```bash
        # Plot three Case*.zip
        $ evo_res {folder name/*.zip} -p --save_table results/table.csv
        ```
        

---

- **[TIPS]**
    - **(Save various extension files)**
        - more good resolution extension files like .eps & .pdf
    - **(Modify plot result)**
        - [1] go to .zip file and click "info.json" file
        - [2] revised "title" & "ref_name" & "est_name"
        - [3] Re-compressed folder
        - [4] Repeat evo_res process, then we can see changed plot legend
    - **(Change plot background)**
        - It can change four type for background like **"dark, darkgrid, white, whitegrid"**
            
            ```bash
            # Change blackgrid to whitegrid
            $ evo_config set plot_seaborn_style whitegrid
            ```
            

---

- **[Result]**
    - (1) Plot trajectory
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled.png" alt="">
        </figure> 

        
    - (2) Plot Absolute Pose Error (APE)
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 1.png" alt="">
        </figure> 
        
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 2.png" alt="">
        </figure> 
        
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 3.png" alt="">
        </figure> 
                
    - (3) Plot Relative Pose Error (RPE)
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 4.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 5.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 6.png" alt="">
        </figure> 
        
    - (4) Plot multiple results from a metric
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 7.png" alt="">
        </figure>         

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 8.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 9.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 10.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 11.png" alt="">
        </figure> 
        
    - (5) Change plot background
        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 12.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 13.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 14.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 15.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 16.png" alt="">
        </figure> 

        <figure class="align-center">
            <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/evo/Untitled 17.png" alt="">
        </figure> 
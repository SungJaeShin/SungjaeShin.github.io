---
title: "[Convert Local to Global Coordinate]"
classes: wide
---

## Github Code 
[![Github_Page]({{ site.url }}{{ site.baseurl }}/assets/images/github_logo.png)](https://github.com/SungJaeShin/TF_conversion.git)*Click Image!*

**[Goal] In order to calculate RMSE, we must change coordinate from local to global !!** 

## 1. Dependencies
- OpenCV & Eigen

## 2. Compare Global Coordinate to Local Coordinate (Not convert to global)
 <table>
    <tr>
       <td> Not convert (xy)</td>
       <td> Not convert (xyz) </td>
    </tr> 
    <tr>
    <td>
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/tf_convert/not_tf_xy.png" alt="">
    </figure> 
    </td>
    <td>
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/tf_convert/not_tf_xyz.png" alt="">
    </figure> 
    </td>
    </tr>
 </table>
   
## 3. Compare Global Coordinate to Local Coordinate (Convert to global)
 <table>
    <tr>
       <td> Convert (xy)</td>
       <td> Convert (xyz) </td>
    </tr> 
    <tr>
    <td>
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/tf_convert/tf_xy.png" alt="">
    </figure> 
    </td>
    <td>
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/tf_convert/tf_xyz.png" alt="">
    </figure> 
    </td>
    </tr>
 </table>

## 4. Reference Site
[1] https://github.com/MichaelGrupp/evo
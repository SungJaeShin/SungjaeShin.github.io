---
title: "[Visualized Pose Tracking]"
classes: wide
---

## Github Code 
<!-- <figure class="align-center">
    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/github_logo.png" alt="">
    <figcaption>https://github.com/SungJaeShin/pose_movement.git</figcaption>
</figure>  -->

[![Github_Page](https://github.com/SungJaeShin/SungjaeShin.github.io/blob/main/assets/images/github_logo.png)](https://github.com/SungJaeShin/pose_movement.git)

# Real-time HDF5 Dataset Generation
Follow [HDF5 Basic Concept](https://heathered-freon-621.notion.site/HDF5-How-to-use-h5py-for-making-dataset-or-database-DB-in-python-24dc231719f34e96bc1e5d8d24f7bd1f) 

---
## Descriptrion of CASE 1 and CASE 2 in test.py code
- [CASE 1] Make Several Dataset which save 'N' subPano Images
- [CASE 2] Make Only One Dataset that save all subPano Images

---
## Usage of global parameter **USE_H5**
- [CASE 1] **"USE_H5 = True"**: Save subPano images in h5py extension file.
- [CASE 2] **"USE_H5 = False"**: Save subPano images in local folder.
---
title: "[Python] Basic grammar of python"
excerpt: "Python Basic"
---
# [Python] Basic concept of python

## [Goal] python 을 하면서 기본적인 것들을 살펴보자

---

- **Type**
    
    ```python
    # ****Case 1****: Print argument type
    print("Type of image: ", type(image)) # output: <class 'numpy.ndarray'>
    
    # ****Case 2****: Change argument type
    img_width = 480
    step_size = 12
    
    des_col = int(img_width/step_size)
    
    print("Result of des_col: ", des_col) # output: 40
    ```
    

---

- **List**
    
    ```python
    # ****Example 1**** 
    tmp_img = []
    tmp_img.append(np.array([0, des]))
    tmp_img.append(np.array([1, des1]))
    # des and des1 shape: [num_of_descriptor, dimension]
    
    print("Type of tmp_img: ", type(tmp_img)) # output: <class 'list'>
    print("Length of tmp_img: ", len(tmp_img)) # output: 2
    
    # ****Example 2****
    tmp_img2 = [0, des]
    tmp_img2 = [tmp_img2, [1, des1]]
    
    print("Type of tmp_img2: ", type(tmp_img2)) # output: <class 'list'>
    print("Length of tmp_img2: ", len(tmp_img2)) # output: 2
    
    # ****Example 3****
    tmp_img3 = [[0, des], [1, des1]]
    
    print("Type of tmp_img3: ", type(tmp_img3)) # output: <class 'list'>
    
    # ****Example 4****
    tmp_img4 = []
    tmp_img4.append(np.array(des))
    tmp_img4.append(np.array(des1))
    
    print("Type of tmp_img4: ", type(tmp_img4)) # output: <class 'list'>
    print("Length of tmp_img4: ", len(tmp_img4)) # output: 2
    print("Length of tmp_img4[0]: ", len(tmp_img4[0])) # output: 938
    print("Size of tmp_img4[0]: ", tmp_img4[0].shape) # output: (938, 128)
    print("Type of tmp_img4[0]: ", type(tmp_img4[0])) # output: <class 'numpy.ndarray'>
    ```
    

---

- **Numpy.ndarray**
    
    ```python
    # ****Case 1****: Convert List to numpy using numpy.array
    tmp_img = []
    tmp_img.append(np.array(des))
    tmp_img.append(np.array(des1))
    
    tmp_img2 = np.array(tmp_img, dtype=object)
    
    print("Type of tmp_img2: ", type(tmp_img2)) # output: <class 'numpy.ndarray'>
    print("Size of tmp_img2: ", tmp_img2.shape) # output: (2, )
    print("Type of tmp_img2[0]: ", type(tmp_img2[0])) # output: <class 'numpy.ndarray'>
    print("Size of tmp_img2[0]: ", tmp_img2[0].shape) # output: (938, 128)
    print("Size of tmp_img2[0][1]: ", tmp_img2[0][1].shape) # output: (128, )
    print("Size of tmp_img2[0][900]: ",tmp_img2[0][900].shape) # output: (128, )
    print("Type of tmp_img2[0][1]: ", type(tmp_img2[0][1])) # output: <class 'numpy.ndarray'>
    
    # ****Case 2****: Create numpy using zeros
    tmp_img = np.zeros((img_width, img_height), "float32")
    
    # ****Case 3****: Concatenate numpy
    collect_des = des
    collect_des = np.concatenate((collect_des, des1), axis=0)
    # [des] + [des1] = [[des], [des1]]
    
    print("Size of collect_des: ", collect_des.shape) # output: (2104, 128)
    print("Type of collect_des: ", type(collect_des)) # output: <class 'numpy.ndarray'>
    ```
    

---

- **Size**
    
    ```python
    # ****Case 1****: size of numpy using shape
    collect_des = des
    collect_des = np.concatenate((collect_des, des1), axis=0)
    
    print("Size of collect_des: ", collect_des.shape) # output: (2104, 128)
    
    # ****Case 2****: size of list using len
    tmp_img = []
    tmp_img.append(np.array([0, des]))
    tmp_img.append(np.array([1, des1]))
    # des and des1 shape: [num_of_descriptor, dimension]
    
    print("Length of tmp_img: ", len(tmp_img)) # output: 2
    ```
    

---

- **Transpose**
    
    ```python
    # ****Case 1****: numpy transpose assume that x shape is [m, n]
    y = np.transpose(x) 
    
    print("Size of y: ", y.shape) # output: [n, m]
    
    # ****Case 2****: numpy .T assume that x shape is [m, n]
    z = x.T
    
    print("Size of z: ", z.shape) # output: [n, m]
    ```
    

---

- **Where & Smallest Value**
    
    ```python
    collect_center = cyvlfeat.kmeans.kmeans(collect_des, 50) 
    dist = euclidean_dist(imgs[0], collect_center) 
    smallest_value = np.amin(dist[0]) 
    smallest_index = np.where(dist[0] == np.amin(dist[0]))
    
    print("Type of collect_center: ", type(collect_center)) # output: <class 'numpy.ndarray'>
    print("Size of collect_center: ", center.shape) # output: (50, 128)
     
    print("Type of dist: ", type(dist)) # output: <class 'numpy.ndarray'> 
    print("Size of dist: ", dist.shape) # output: (938, 50)
    
    print("Find smallest value: ", smallest_value) # output: 301.1271
    print("Type of smallest value: ", type(smallest_value)) # output: int
    print("Size of smallest value: ", smallest_value.size) # output: (1, )
    print("Find smallest index: ", smallest_index[0]) # output: 40
    ```
    

---

- **Multiple Return Function**
    
    ```python
    # **Only use second return value**
    # Assume: return of cyvlfeat.sift.sift() is frames and descriptors
    _, des = cyvlfeat.sift.sift(imgs[i].T, float_descriptors=True, compute_descriptor=True)
    ```
    

---

- **Numpy Matrix**
    
    ```python
    # ****Case 1**: Convert 2D to 1D matrix**
    arr = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]])
    flat_array = np.ravel(arr)
    
    print("Output of flat_array: ", flat_array) # output: [0, 1, 2, 3, 4, 5, 6, 7, 8]
    
    # ****Case 2**: Get specific values of 1D matrix**
    letters = np.array([1, 3, 5, 7, 9, 7, 5])
    
    print("Output of letters[2:5]: ", letters[2:5]) # output: [5, 7, 9]
    print("Output of letters[:-5]: ", letters[:-5]) # output: [1, 3]   
    print("Output of letters[5:]: ", letters[5:]) # output:[7, 5]
    print("Output of letters[:]: ", letters[:]) # output:[1, 3, 5, 7, 9, 7, 5]
    print("Output of letters[::-1]: ", letters[::-1]) # output:[5, 7, 9, 7, 5, 3, 1] *****Reverse List*****
    
    # ****Case 3**: Get specific values of 2D matrix**
    A = np.array([[1, 4, 5, 12, 14], [-5, 8, 9, 0, 17], [-6, 7, 11, 19, 21]])
    
    print("Output of A[:, 0]: ", A[:, 0]) # output: [1 -5 -6]
    print("Output of A[:, 3]: ", A[:, 3]) # output: [12 0 19]
    print("Output of A[:, -1]: ", A[:, -1]) # output: [14 17 21] *****Last Column***** 
    print("Output of A[:2, :4]: ", A[:2, :4]) # output: [[1 4 5 12] [-5 8 9 0]]
    print("Output of A[:1,]: ", A[:1,]) # output: [[1 4 5 12 14]]
    print("Output of A[:,2]: ", A[:,2]) # output: [5 9 11]
    print("Output of A[:, 2:5]: ", A[:, 2:5]) # output: [[5 12 14] [9 0 17] [11 19 21]]
    ```
    

---

- **For Loop**
    
    ```python
    # ****Case 1****: Start at zero
    for i in range(10):
    	print("Output of i: ", i) # output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    
    # ****Case 2****: Start at one
    for i in range(1, 10):
    	print("Output of i: ", i) # output: 1, 2, 3, 4, 5, 6, 7, 8, 9
    
    # ****Case 3****: Start at one using nums
    for i in nums(1, 5):
    	print("Output of i: ", i) # output: 1, 2, 3, 4, 5
    ```
    

---

- **Path Join**
    
    ```python
    data_dir = os.path.join("practical-category-recognition-2013a", "data")
    des_path = os.path.join(data_dir, 'dsift_1024', 'train_des.npy')
    
    print("Result of des_path: ", des_path) 
    # output: /practical-category-recognition-2013a/data/dsift_1024/train_des.npy
    ```
    

---
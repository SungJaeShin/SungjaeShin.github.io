---
title: "[HDF5] How to use h5py for making dataset or database(DB) in python?"
excerpt: "HDF5 Database Tutorial"
---
# [HDF5] How to use h5py for making dataset or database(DB) in python?

## [Goal] 다량의 데이터셋을 카테고리 또는 속성별로 저장하고 만들수 있다.

---

- **[Github Site]**
    
    - [https://github.com/SungJaeShin/h5py_tutorial.git](https://github.com/SungJaeShin/h5py_tutorial.git)
    

---

- **[Reference Site]**
    
    - [How to resize an HDF5 array with `h5py`](https://stackoverflow.com/questions/22998248/how-to-resize-an-hdf5-array-with-h5py)
    
    - [h5py 사용법 HDF5 예제 - HiSEON](https://hiseon.me/python/h5py-hdf5/)
    
    - [점프 투 파이썬](https://wikidocs.net/33241)
    
    - [H5py adding more data to an existing dataset](https://forum.hdfgroup.org/t/h5py-adding-more-data-to-an-existing-dataset/6784/3)
    
    - [Datasets - h5py 3.7.0 documentation](https://docs.h5py.org/en/stable/high/dataset.html#resizable-datasets)
    
    - [Example of resizing an array with h5py using a Python class.](https://gist.github.com/cmbiwer/f689f22a02927394c23c12430b866f85)
    

---

- **[Advantage of h5py]**
    - 대용량의 데이터를 빠르게 읽고 쓸수 있는 장점을 가지고 있다.
        - **[Reference Site]** Comparing storage and read time between PNG files and HDF5 **(highly recommend !!!)**
            
            - [Three Ways of Storing and Accessing Lots of Images in Python - Real Python](https://realpython.com/storing-images-in-python/#experiment-for-storing-many-images)
            
    - 쉽게 카테고리별 또는 속성별로 데이터를 나뉘서 저장할 수 있는 장점을 가지고 있다.

---

- **[Component of h5py]**
    
    <figure class="align-center">
        <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/h5py/Untitled.png" alt="">
    </figure> 
    
    - 중요 개념: **Group, Dataset, Attribute**
        - (1) Group: 항상 HDF5 파일에는 **root group(’/’) 이 존재**한다. 그리고 이는 하나의 컨테이너로서의 역할을 한다. ‘**/’을 붙여서 절대 경로를 만들어 줄 수 있다.** 그 뒤에 하위 트리구조로 다른 그룹도 생성할 수 있다.
        - (2) Dataset: 해당 group에 맞는 dataset을 만들기 위해 하고 이에 해당하는 데이터를 저장할 수 있다.

---

- **[Mode of h5py]**
    
    
    | Mode | Description |
    | --- | --- |
    | r | Read only, file must exist (default) |
    | r+ | Read/ write, file must exist |
    | w | Create file, truncate if exists |
    | w- or x | Create file, fail if exists |
    | a | Read/ write if exists, create otherwise |

---

- **[Function of h5py]**
    
    
    | Function | Description |
    | --- | --- |
    | h5py.File(path, mode) | path 경로에 mode에 따라서 읽고 쓰기 위한 함수, ‘type(path) = string’ |
    | create_group(name) | name 이름을 가진 group을 생성, ‘type(name) = string’ |
    | create_dataset(name) | name 이름을 가진 dataset을 생성, ‘type(name) = string’ |
    | name.attrs[attribute] = ~ | name 이름을 가진 group의 ~라는 속성을 attribute를 넣음, ‘type(attribute) = string’   |
    | name | 해당 파일 이름을 반환 |
    | keys() | 해당 경로에 속해 있는 내용들을 반환 |
    | values() | 해당 경로에 대한 정보 및 하위 내용들을 반환  |
    | close() | 해당 h5py 관련 memory 및 변수 제거 |

---

- **[Example of h5py]**
    - 위의 그림처럼 구성한 code
        
        ```python
        # Set file path and make h5py file
        h5_filename = "~"
        
        # Write h5py file and write 
        hw = h5py.File(h5_filename, 'w')
        
        # Make group named /subPano
        hw.create_group('/subPano')
        
        # Make dataset named "/subPano/1" and put data1
        idx1 = "/subPano" + "/1"
        data1 = np.arange(10)
        hw.create_dataset(idx1, data=data1)
        
        # Make dataset named "/subPano/2" and put data2
        idx2 = "/subPano" + "/2"
        data2 = np.arange(20)
        hw.create_dataset(idx2, data=data2)
        
        # Erase h5py memory
        hw.close()
        
        # Read h5py file alreay existed
        h5 = h5py.File(h5_filename, 'r')
        ```
        
    
    ---
    
    - 위의 그림을 기반하여 debugging을 한 결과
        
        ```python
        ########################################################
        $ (Pdb) h5 
        	-> <HDF5 file "subpanoDB.h5" (mode r)>
        
        $ (Pdb) type(h5)
        	-> <class 'h5py._hl.files.File'>
        
        $ (Pdb) h5.name
        	-> '/'
         
        $ (Pdb) h5.keys()
        	-> <KeysViewHDF5 ['subPano']>
        
        $ (Pdb) h5.values()
        	-> ValuesViewHDF5(<HDF5 file "subpanoDB.h5" (mode r)>)
        ########################################################
        $ (Pdb) type(h5['subPano'])
        	-> <class 'h5py._hl.dataset.Dataset'>
        
        $ (Pdb) h5['subPano'].name
        	-> '/subPano'
        
        $ (Pdb) h5['subPano'].keys()
        	-> <KeysViewHDF5 ['1', '2']>
        
        $ (Pdb) h5['subPano'].values()
          -> ValuesViewHDF5(<HDF5 group "/subPano" (2 members)>)
        ########################################################
        $ (Pdb) h5['subPano']['1']
        	-> <HDF5 dataset "1": shape (10,), type "<i8">
        
        $ (Pdb) h5['/subPano/1']
        	-> <HDF5 dataset "1": shape (10,), type "<i8">
        
        $ (Pdb) h5['subPano']['2']
        	-> <HDF5 dataset "2": shape (20,), type "<i8">
        
        $ (Pdb) h5['/subPano/2']
        	-> <HDF5 dataset "2": shape (20,), type "<i8">
        
        $ (Pdb) h5['/subPano/1'].shape
        	-> (10,)
        
        $ (Pdb) h5['/subPano/2'].shape
        	-> (20,)
        ########################################################
        $ (Pdb) type(h5['/subPano/1'][:])
        	-> <class 'numpy.ndarray'>
        
        $ (Pdb) h5['/subPano/1'][:]
        	-> array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        
        $ (Pdb) h5['/subPano/1'][0]
        	-> 0
        
        $ (Pdb) h5['/subPano/1'][1]
        	-> 1
        
        $ (Pdb) h5['/subPano/2'][:]
        	-> array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19])
        
        $ (Pdb) h5['/subPano/2'][0]
        	-> 0
        
        $ (Pdb) h5['/subPano/2'][1]
        	-> 1
        ########################################################
        ####################### ERROR ! ########################
        $ (Pdb) h5['subpano'].values()
        	-> *** AttributeError: 'Dataset' object has no attribute 'values'
        
        $ (Pdb) h5['subpano'].keys()
        	-> *** AttributeError: 'Dataset' object has no attribute 'keys'
        
        $ (Pdb) h5['/subPano/1'][]
        	-> *** SyntaxError: invalid syntax
        
        $ (Pdb) h5['/subPano/2/0']
        	-> *** KeyError: 'Unable to open object (message type not found)'
        
        $ (Pdb) h5['/subPano/2/1']
        	-> *** KeyError: 'Unable to open object (message type not found)'
        
        $ (Pdb) h5['subpano'][:].values()
        	-> *** AttributeError: 'numpy.ndarray' object has no attribute 'values'
        
        $ (Pdb) h5['subpano'][:].name
        	-> *** AttributeError: 'numpy.ndarray' object has no attribute 'name'
        
        $ (Pdb) h5['subpano'][:].keys()
        	-> *** AttributeError: 'numpy.ndarray' object has no attribute 'keys'
        ########################################################
        ```
        

---

- **[Resize h5py]**
    - 생성된 dataset에 data들이 저장되어 있을 경우, 이 **size는 이미 고정되어 있기 때문**에 같은 dataset 경로에 추가적으로 data를 넣기 위해서는 다음과 같은 방법으로 해결할 수 있다.
        - **[Step 0] h5py.File(path, mode)**
            - h5py를 write도 하지만 read도 할 수 있도록 mode를 ‘a’로 지정
        - **[Step 1] maxshape = (None, )**
            - create_dataset(path, data, maxshape, ~) 에서 dataset의 크기를 고정하지 않도록 maxshape를 None으로 설정
        - **[Step 2] Dataset.resize()**
            - h5py[’group_name’].resize(~) 으로 원하는 size로 변경
    - **(주의)** 이미지를 같은 dataset에 넣을 때, 이미지 size를 같은 size로 고정시켜야 추가가 가능하다.
        - Reference Site: [https://www.activeloop.ai/resources/hdf-5-hierarchical-data-format-5-vs-hub-creating-performant-computer-vision-datasets/](https://www.activeloop.ai/resources/hdf-5-hierarchical-data-format-5-vs-hub-creating-performant-computer-vision-datasets/)
            
    
    ---
    
    - Sample Code
        
        ```python
        # Set file path and make h5py file
        h5_filename = "~"
        
        # Initialization
        path = 'subpano'
        data1 = np.arange(10)
        data2 = np.arange(20)
        lend1 = data1.shape[0]
        lend2 = data2.shape[0]
        
        # Set initial dataset
        h5 = h5py.File(h5_filename, 'w')
        h5.create_dataset(path, data=data1, maxshape=(None,))
        h5.close()
        
        # Change dataset size
        h5 = h5py.File(h5_filename, 'a')
        
        total_len = lend1 + lend2
        total_len = np.array([total_len])
        
        h5[path].resize(total_len)
        h5[path][lend1:] = data2
        
        ########################################################
        ###################### RESULT ! ########################
        $ h5[path]
        	-> <HDF5 dataset "subpano": shape (30,), type "<i8">
        
        $ h5[path][:]
        	-> array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  0,  1,  2,  3,  4,  5,  6,
                7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19])
        
        $ print("h5: ", h5)
        	-> h5:  <HDF5 file "subpanoDB.h5" (mode r+)>    
        
        $ print("h5 type: ", type(h5))
        	-> h5 type:  <class 'h5py._hl.files.File'>
        
        $ print("h5[path]: ", h5[path])
        	-> h5[path]:  <HDF5 dataset "subpano": shape (30,), type "<i8">
        
        $ print("h5[path] type: ", type(h5[path]))
        	-> h5[path] type:  <class 'h5py._hl.dataset.Dataset'>
        
        $ print("h5[paht].shape: ", h5[path].shape)
        	-> h5[paht].shape:  (30,)
        
        $ print("h5[path][:]: ", h5[path][:])
        	-> h5[path][:]:  [ 0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19]
        
        $ print("h5[path][:] type: ", type(h5[path][:]))
        	-> h5[path][:] type:  <class 'numpy.ndarray'>
        ```
        

---

- **[Error List]**
    - **[ValueError]** Unable to create dataset (name already exists)
        - **[Solution]** 이미 해당 경로에 같은 이름의 파일이 이미 생성되어 있어 h5 파일을 만들기 위해 해당 파일을 지워야한다.
    
    ---
    
    - **[TypeError]** Only chunked datasets can be resized
        - **[Solution]** dataset을 생성할 때, maxshape를 None으로 할 뿐만 아니라 chunk 설정을 통해 가변적으로 변할 수 있음을 flag(**”chunks=True”**)를 통해서 설정해준다.
            
            ```python
            create_dataset(path, data, maxshape, **chunks=True**)
            ```
            
        - [Reference Site]
            - [https://docs.h5py.org/en/stable/high/dataset.html](https://docs.h5py.org/en/stable/high/dataset.html)
                
            - [https://github.com/h5py/h5py/issues/1202](https://github.com/h5py/h5py/issues/1202)
                
            - [https://github.com/h5py/h5py/issues/1685](https://github.com/h5py/h5py/issues/1685)
                
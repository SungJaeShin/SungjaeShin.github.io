---
title: "[Visualization Tools] Introduction 3D Pointcloud and Mesh Visualization Tools"
excerpt: "Visualization Pointcloud and Mesh"
---
# [Visualization Tools] Introduction 3D Pointcloud and Mesh Visualization Tools

---

- **[Goal]**
    - 3차원 pointcloud 정보 (with Mesh)를 시각화해줄 수 있는 tools들을 사용하고 결과를 복원할 수 있다.

---

- **[PLY file]**
    - Pointcloud 및 mesh 정보(**called Polygon file**)를 담고 있는 file 형식 중 ply file은 다음과 같은 구조를 가지고 있다.
        - **기본 구조 (ASCII version of cube)**
            
            ```yaml
            ply
            format ascii 1.0           { **ascii/binary**, format version number }
            comment made by Greg Turk  { comments keyword specified, like all lines }
            comment this file is a cube
            element vertex 8           { define "vertex" element, 8 of them in file }
            property float x           { vertex contains float "x" coordinate }
            property float y           { y coordinate is also a vertex property }
            property float z           { z coordinate, too }
            element face 6             { there are 6 "face" elements in the file }
            property list uchar int vertex_index { "vertex_indices" is a list of ints }
            end_header                 { delimits the end of the header }
            0 0 0                      { start of vertex list }
            0 0 1
            0 1 1
            0 1 0
            1 0 0
            1 0 1
            1 1 1
            1 1 0
            4 0 1 2 3                  { start of face list }
            4 7 6 5 4
            4 0 4 5 1
            4 1 5 6 2
            4 2 6 7 3
            4 3 7 4 0
            ```
            
            - Note that the face list generates triangles in the order of a TRIANGLE FAN, not a TRIANGLE STRIP. In the example above, the first face 4 0 1 2 3 is composed of **the triangles 0,1,2 and 0,2,3 and not 0,1,2 and 1,2,3. (0,1,2 & 0,2,3 으로 triangle 생성)**
    - “ASCII ply” vs “Binary ply”
        - ASCII 형식으로 작성된 ply 파일의 경우, 꼭지점과 면이 각각 space 및 공백으로 구분된 숫자로 한줄에 작성된다.
        - Binary 형식으로 작성된 ply 파일의 경우, endian으로 encoding되어 서로 밀집하게 연관된 숫자로 작성된다. 예를 들면 맨 첫번째 숫자는 vertex들의 숫자이고 나머지 숫자들은 prev vertex들의 index 등등.. 우리가 가시적으로 볼 수 없는 데이터로 작성된다.
    - Reference Site
        
        - [PLY - Polygon File Format](https://paulbourke.net/dataformats/ply/)
        
        - [PLY (file format)](https://en.wikipedia.org/wiki/PLY_(file_format))
        

---

- **[PLY Visualization Tools]**
    - **[Case 1] Open3d**
        - Installation
            
            ```bash
            $ pip3 install open3d
            ```
            
        - Simple code for visualized .ply file
            
            ```python
            from open3d import *    
            
            def main():
                ply_path = "~/xxx.ply" 
                cloud = io.read_point_cloud(ply_path) # Read point cloud
                visualization.draw_geometries([cloud])    # Visualize point cloud      
            
            if __name__ == "__main__":
                main()
            ```
            
        - Result
            <figure class="align-center">
                <video src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled.mp4" alt="">
            </figure>
            
        - Reference Site
            
            - [Python - Display 3D Point Cloud](https://stackoverflow.com/questions/50965673/python-display-3d-point-cloud)
            
            - [Mesh — Open3D 0.17.0 documentation](http://www.open3d.org/docs/release/tutorial/geometry/mesh.html)
            
    
    ---
    
    - **[Case 2] pptk**
        - Installation
            
            ```bash
            $ pip3 install pptk
            $ pip3 install plyfile
            ```
            
        - Simple code for visualized .ply file
            
            ```python
            import pptk
            import numpy as np
            import plyfile
            
            ply_path = "~/xxx.ply" 
            data = plyfile.PlyData.read(ply_path)['vertex']
            xyz = np.c_[data['x'], data['y'], data['z']]
            rgb = np.c_[data['red'], data['green'], data['blue']]
            n = np.c_[data['nx'], data['ny'], data['nz']]
            
            v = pptk.viewer(xyz)
            v.attributes(rgb / 255., 0.5 * (1 + n))
            ```
            
        - Result
            - only supported up to python version 3.6
            - So, there is no result in my environment !!
        - Reference Site
            
            - [GitHub - heremaps/pptk: The Point Processing Toolkit (pptk) is a Python package for visualizing and processing 2-d/3-d point clouds.](https://github.com/heremaps/pptk)
            
            - [pip install not working with python 3.9  · Issue #54 · heremaps/pptk](https://github.com/heremaps/pptk/issues/54)
            
    
    ---
    
    - **[Case 3] pyntcloud**
        - Installation
            
            ```bash
            $ pip3 install pyntcloud
            ```
            
        - Simple code for visualized .ply file
            - **[참고] Plot을 위한 여려 flag 및 argument들이 존재!!**
                
                ```python
                def plot(self, backend=None, scene=None, width=800, height=500,
                         background="black", mesh=False, use_as_color=["red", "green", "blue"],
                         initial_point_size=None, cmap="hsv", polylines=None,
                         linewidth=5, return_scene=False, output_name="pyntcloud_plot",
                         elev=0., azim=90., **kwargs):
                
                        """Visualize a PyntCloud  using different backends.
                
                        Parameters
                        ----------
                        backend: {"pythreejs", "threejs", "pyvista", "matplotlib"}, optional
                            Default: "pythreejs"
                            Used to select one of the available libraries for plotting.
                
                        width: int, optional
                            Default: 800
                
                        height: int, optional
                            Default: 500
                
                        background: str, optional
                            Default: "black"
                            Used to select the default color of the background.
                            In some backends, i.e "pythreejs" the background can be dynamically changed.
                
                        use_as_color: str or ["red", "green", "blue"], optional
                            Default: ["red", "green", "blue"]
                            Indicates which scalar fields will be used to colorize the rendered
                            point cloud.
                
                        initial_point_size: the initial size of each point in the rendered point cloud.
                            Can be adjusted after rendering using slider.
                
                        cmap: str, optional
                            Default: "hsv"
                            Color map that will be used to convert a single scalar field into rgb.
                            Check matplotlib cmaps.
                
                        return_scene: bool, optional
                            Default: False.
                            Used with "pythreejs" backend in order to return the pythreejs.Scene object
                
                        polylines: list of dict, optional
                            Default None.
                            List of dictionaries defining polylines with the following syntax:
                            polylines=[
                                {
                                    "color": "0xFFFFFF",
                                    "vertices": [[0, 0, 0], [0, 0, 1]]
                                },
                                {           {
                                    "color": "red",
                                    "vertices": [[0, 0, 0], [0, 0, 1], [0, 2, 0]
                                }
                            ]
                        elev: float
                            Elevation angle in the z plane. Used for matplotlib
                        azim: float
                            Azimuth angle in the x,y plane.
                        """
                ```
                
            
            ```python
            from pyntcloud import PyntCloud
            import matplotlib
            
            ply_path = "./xxx.ply" 
            cloud = PyntCloud.from_file(ply_path)
            
            # There are a lot of visualization method (library) !! -> **"pythreejs", "threejs", "pyvista", "matplotlib"**
            # And this package also can use in Jupyter notebook !!
            # cloud.plot(use_as_color="x", cmap="cool", backend="matplotlib")
            cloud.plot(backend="matplotlib", mesh=True, cmap="hsv")
            ```
            
        - Result **(mesh file 크기가 클수록 시간이 오래걸림… 근데 이게 너무 심하게 오래 걸림…)**
            - Example 1 (w/o mesh) → `cloud.plot(use_as_color="x", cmap="cool", backend="matplotlib")`
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/viz_3d_pc/Untitled.png" alt="">
                </figure>
                
            - Example 2 (w/ mesh) → `cloud.plot(backend="matplotlib", mesh=True, cmap="hsv")`
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/blog/viz_3d_pc/Untitled 1.png" alt="">
                </figure>
                                
        - Reference Site
            
            - [GitHub - daavoo/pyntcloud: pyntcloud is a Python library for working with 3D point clouds.](https://github.com/daavoo/pyntcloud)
            
            - [jupyter widget problem: pythreejs · Issue #1698 · pyvista/pyvista](https://github.com/pyvista/pyvista/issues/1698)
            
            - [https://github.com/daavoo/pyntcloud/blob/master/examples/%5Bvisualization%5D%20PyntCloud.ipynb](https://github.com/daavoo/pyntcloud/blob/master/examples/%5Bvisualization%5D%20PyntCloud.ipynb)
            
    
    ---
    
    - **[Case 4] Custom Github page (Using OpenGL)**
        - **Only for ASCII format ! (Not allow Binary format !)**
            - **필요한 형식이 담겨 있는 code ([3DPlyModel.cpp](https://github.com/mateus558/PLY-models-viewer/blob/master/3DPlyModel.cpp#L107-L131))**
                
                ```cpp
                for(auto itr = items.begin(); itr != items.end(); itr++){
                		if(*(itr) == string("format") && *(itr+1) != string("ascii")){
                			cerr << "Codification not supported. (Ply loading)" << endl;
                			return;
                		}
                		
                		if(*(itr) == string("vertex")){
                			np = stoi(*(itr+1));
                			points.resize(np);
                		}
                		if(*(itr) == string("face")){
                			nf = stoi(*(itr+1));
                			faces.resize(nf);
                		}
                		if(*(itr) == string("nx")){
                			hasNormals = true;
                			normals.resize(np);
                		}
                		if(*(itr) == string("s")){
                			hasTexture = true;
                			uv_coordinates.resize(np);
                		}
                	}
                	items.clear();
                
                ```
                
            - If you want to convert binary_little_endian to ascii, please convert ply file first !!
            - **작성자 기준: 1.7GB binary ply file → 5.4GB ascii ply file (용량이 클수록 시간이 오래 걸림)**
                - Installation
                    
                    ```bash
                    $ pip3 install plyfile
                    ```
                    
                - Convert BINARY to ASCII ply file !
                    
                    ```python
                    import plyfile
                    
                    data = plyfile.PlyData.read('./xxx.ply')
                    data.text = True
                    data.write('xxx_ascii.ply')
                    ```
                    
                - Reference Site
                    
                    - [GitHub - dranjan/python-plyfile: NumPy-based text/binary PLY file reader/writer for Python](https://github.com/dranjan/python-plyfile/tree/master)
                    
                    - [Python package for conversion of binary_liitle_endian to ascii for a ply format file?](https://stackoverflow.com/questions/75166197/python-package-for-conversion-of-binary-liitle-endian-to-ascii-for-a-ply-format)
                    
        - Installation
            
            ```bash
            $ git clone https://github.com/mateus558/PLY-models-viewer.git
            $ cd ./PLY-models-viewer
            $ g++ -g --std=c++11 main.cpp 3DPlyModel.cpp glcTexture.cpp  -o main -lpng -lGL -lGLU -lglut
            $ ./main <PLY file>
            ```
            
        - Result
            - Computing gouraud… (이렇게 나오면 올바르게 동작 !)
                - 그런데 이렇게만 나오고 사이즈가 크니깐 결과가 하루가 지나도 안나타남! 별로…인듯?
        - Reference Site
            
            - [GitHub - mateus558/PLY-models-viewer: Tool to visualize geometries in the .ply file format on OpenGL.](https://github.com/mateus558/PLY-models-viewer)
            

---

- **[OBJ Visualization Tools]**
    - **[Case 1] vedo**
        - Installation
            
            ```bash
            $ pip3 install vedo
            ```
            
        - Simple code for visualized .obj file
            - **Color를 setting하는 flag 및 argument들이 존재!!**
                - [https://github.com/marcomusy/vedo/blob/master/examples/basic/mesh_coloring.py](https://github.com/marcomusy/vedo/blob/master/examples/basic/mesh_coloring.py)
                
                ```python
                """Specify a colors for cells
                and points of a Mesh"""
                from vedo import dataurl, Plotter, Mesh
                
                plt = Plotter(N=3, axes=11)
                
                ##################################### add a cell array
                man1 = Mesh(dataurl+"man_low.vtk").linewidth(0.1)
                nv = man1.ncells                           # nr. of cells
                scals = range(nv)                          # coloring by the index of cell
                
                man1.cmap("Paired", scals, on='cells').add_scalarbar("cell nr")
                plt.at(0).show(man1, __doc__, elevation=-60)
                
                ##################################### Point coloring
                man2 = Mesh(dataurl+"man_low.vtk")
                scals = man2.points()[:, 0] + 37           # pick x coordinates of vertices
                
                man2.cmap("hot", scals)
                man2.add_scalarbar(horizontal=True)
                plt.at(1).show(man2, "mesh.cmap()")
                
                ##################################### Cell coloring
                man3 = Mesh(dataurl+"man_low.vtk")
                scals = man3.cell_centers()[:, 2] + 37     # pick z coordinates of cells
                man3.cmap("afmhot", scals, on='cells')
                
                # add a fancier 3D scalar bar embedded in the scene
                man3.add_scalarbar3d(size=[None,3])
                man3.scalarbar.rotate_x(90).y(0.2)
                plt.at(2).show(man3, "mesh.cmap(on='cells')")
                
                plt.interactive().close()
                ```
                
            
            ```python
            from vedo import *
            
            mesh = Mesh("~/xxx.obj",)
            
            mesh.show()
            ```
            
        - Result
            - **[Plot 1] Default**
                <figure class="align-center">
                    <video src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled 1.mp4" alt="">
                </figure>
                
            - **[Plot 2 ~ 4] Results**
                <figure class="align-center">
                    <img src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled 2.png" alt="">
                </figure>
                
            - **[Plot 2] Plot cell array (coloring by the index of cell)**
                <figure class="align-center">
                    <video src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled 2.mp4" alt="">
                </figure>
                
            - **[Plot 3] Point coloring (pick x coordinates of vertices)**
                <figure class="align-center">
                    <video src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled 3.mp4" alt="">
                </figure>
                
            - **[Plot 4] Plot cell coloring (pick z coordinates of cells)**
                <figure class="align-center">
                    <video src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled 4.mp4" alt="">
                </figure>
                
        - Reference Site
            
            - [How can I open .obj files with color in python?](https://stackoverflow.com/questions/70729974/how-can-i-open-obj-files-with-color-in-python)
            
            - [vedo](https://vedo.embl.es/)
            
            - [https://github.com/marcomusy/vedo/blob/master/examples/basic/mesh_coloring.py](https://github.com/marcomusy/vedo/blob/master/examples/basic/mesh_coloring.py)
            
    
    ---
    
    - **[Case 2] Open3d**
        - Installation
            
            ```bash
            $ pip3 install open3d
            ```
            
        - Simple code for visualized .obj file
            
            ```python
            from open3d import *    
            
            def main():
                ply_path = "~/xxx.obj" 
                cloud = io.read_triangle_mesh(ply_path) # Read point cloud
                vis = visualization.Visualizer()
                vis.create_window()
                vis.add_geometry(cloud)    # Visualize point cloud      
                vis.run()
                vis.destroy_window()
            
            if __name__ == "__main__":
                main()
            ```
            
        - Result
            <figure class="align-center">
                <video src="{{ site.url }}{{ site.baseurl }}/assets/images/paper/viz_3d_pc/Untitled 5.mp4" alt="">
            </figure>
            
        - Reference Site
            
            - [How to your load 3D Model (.obj) in Open3D?](https://stackoverflow.com/questions/60104854/how-to-your-load-3d-model-obj-in-open3d)
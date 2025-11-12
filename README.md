# Single Voter Spreading for Efficient Correspondence Grouping and 3D Registration (SVOS)


### C++ implementation 
We provide a simple demo in C++
## Build
SVOS only depends on [PCL](https://github.com/PointCloudLibrary/pcl/tags) (`>= 1.10.1`, version `1.10.1` is recommended). Please install the library first.

To build SVOS, you need [CMake](https://cmake.org/download/) (`>= 3.23`) and, of course, a compiler that supports `>= C++11`. The code in this repository has been tested on Windows (MSVC `=2022` `x64`), and Linux (GCC `=10.4.0`). Machines nowadays typically provide higher [support](https://en.cppreference.com/w/cpp/compiler_support), so you should be able to build MAC on almost all platforms.

We provide the CMakeLists file. Simply use CMake to compile and link the PCL library, and you're done.

### Python implementation
We will provide the python code.


## Data
We provide the correspondence data for 3DMatch and 3DLoMatch datasets under the FPFH descriptor. The data can be downloaded at https://pan.baidu.com/s/1s0KrOw7Rpoy-X87RoWyS0g?pwd=7mgk.

SVOS processes initial correspondence from feature matching and outputs the filtered correspondence. What's more, SVOS also provides a ransac method for registration guided by triangles.

### 3DMatch & 3DLoMatch Results
In our C++ code, Please change "resultPath" and "datasetPath" to your paths. After running, you will get:

"3dmatch"
-----------------------------
	total:
		RR:1385/1623 0.8534
		RE:2.1283
		TE:6.7867
-----------------------------

"3dlomatch"
-----------------------------
	total:
		RR:783/1781 0.4396
		RE:4.0107
		TE:10.7088
-----------------------------
## Citation
If you find this code useful for your work or use it in your project, please consider citing:

```shell
@article{quan2025single,
  title={Single Voter Spreading for Efficient Correspondence Grouping and 3D Registration},
  author={Quan, Siwen and Zeng, Zhao and Zhang, Xiyu and Yang, Jiaqi},
  journal={IEEE Transactions on Pattern Analysis and Machine Intelligence},
  year={2025},
  publisher={IEEE}
}
```


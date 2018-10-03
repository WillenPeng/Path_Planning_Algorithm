1. under ~/Desktop/Path_Planning_Algorithm/HybridAstar_MPC/build:
$ make (if it returns error, delete the build folder and make a new one, then type: cd build && cmake .. && make)

2. under ~/Desktop/Path_Planning_Algorithm/HybridAstar_MPC/build:
$ ./main

3. under ~/Desktop/Path_Planning_Algorithm/HybridAstar_MPC/simulation:
$ python animation.py

Results:
	After running cpp, path planning and control results will be saved in .dat file. It can be visualized ater running animation.py

Dependency:
	Eigen-3.3, Ipopt-3.12.7

Efforts:
	All the MPC scripts are implemented by Hanxiang Li, the vehicle model is adopted from https://github.com/jeremy-shannon/CarND-MPC-Project
	All the path plannign scripts are implemented by Weilun Peng
	Main function and animation.py are implemented by Weilun Peng

Improvement:
	Validation on 3D simulator (Carla, Airsim)

# ndt_omp
put into the [catkin_ws/src] use catkin_make to build it 

after catkin_make the exe will show in the catkin_ws/devel/lib/ndt_omp,name align 
just run it with ./align

use pcl1.8  should add the *set(PCL_DIR "/usr/local/include/pcl-1.8/pcl")* to tell the cmake that where is the pcl1.8, use pcl1.10 will have the compile error.

change the read file method, now is auto read all the files.

the align function has a loop that will run 10 times, delete it will be faster. and i found that the result in 1 time and in 10 time is similar, and 1 time (direct7 * 1 time align) maybe have a better performance than the icp.

the main function will call 2 time align, one is the normal ndt and the other is ndt_omp(direct7), comment the first one will be more faster.



#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <fstream>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<pcl/registration/icp.h>

#include <dirent.h> //使用DIR的头文件
#include <time.h> //

using namespace std;


//when change the dataset,change this path is ok

const string path = "/home/yezotemp/dataset/2011_09_26/sync_0009/velodyne_points/";

bool getPcds(vector<string>& allCloud,string path)
{
    string filepath = path + "pcds/";
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(filepath.c_str())) == NULL) {
        cout<<"failed to get the pcds! The dictory is wrong!"<<endl;
        return false;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);
        //cout<<name<<endl;
        if ( name != "."  &&  name != ".."  &&  (name.substr(name.size() - 3, name.size()) == "pcd") ){ //because search will read the . and .. 
	      cout << name << endl;
          while(name.length() < 9){ //full the file name to sort 
              name = "0" + name;
          }
            allCloud.push_back(name);
        }
    }
    closedir(dp);
    std::sort(allCloud.begin(), allCloud.end());
    for(int q = 0;q < allCloud.size();q++){
    
      cout << allCloud[q] << endl;
    }
    cout<<"Got the images."<<endl;
    return true;
}


//compile
//& in the catkin_ws 
// run -> catkin_make
// then the exe will put in the devel/lib/yourExe
// the kitti dataset is 10hz  timestamp = 0.1s

// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud ) {
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  auto t1 = ros::WallTime::now();
  registration->align(*aligned);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  /*
  for(int i=0; i<10; i++) {
    registration->align(*aligned);
  }
  */
  auto t3 = ros::WallTime::now();
  //std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;

  return aligned;
}


struct pose  //位姿信息

{

double x, y, z;

double pitch, roll, yaw;

};


int main(int argc, char** argv) {
  /*
  if(argc != 3) {
    std::cout << "usage: align target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1]; //here is just the string name of the pcd
  std::string source_pcd = argv[2];
  */
  if(argc != 1){
    std::cout << "usage: ./align" << std::endl;
    return 1;
  }
  
  /*  the pre read file method, should provide a txt with have all the pcd name
  //std::string pcd_file = argv[1]; // get the file name of all the pcd
  ifstream fin(pcd_file);
  if(!fin){
    cout << "can't open the file " << pcd_file << endl;
    return 1;
  }
  
  std::vector<std::string> allCloud; //save all the pcd file name

    
  while(!fin.eof()){  //read all the file
    std::string temp;
    fin >> temp;
    allCloud.push_back(temp);
  }
  */

  std::vector<std::string> allCloud; //save all the pcd file name 

  if(getPcds(allCloud,path)){
    cout << "Get all the clouds!" << endl;
  }
  else{
    cout << "can not get the clouds!" << endl;
  }

  std::vector<pose> allPose; //save all the pose 

  cout << allCloud.size() << endl;
  
  std::vector<std::string>::iterator it = allCloud.begin();
  std::vector<Eigen::Matrix4f> allTran;
  
  int vectorSize = allCloud.size();
  /* test
  std::cout << vectorSize << endl;
  for(;it != allCloud.end();it++){
    cout << *it << endl;
  }
  */


  for(int q = 0;q < vectorSize-2;q++){ //why read the txt will get the count which is true count+1

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>()); //construct a pcd member
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    std::string target_pcd = path + "pcds/" + *it;
    it++;
    std::string source_pcd = path + "pcds/" + *it;
    
    if(pcl::io::loadPCDFile(target_pcd, *target_cloud)) { //read the pcd
      std::cerr << "failed to load 1 " << target_pcd << std::endl;
      return 0;
    }
    if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
      std::cerr << "failed to load 2 " << source_pcd << std::endl;
      return 0;
    }


    // downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*downsampled); //对点云进行采样
    *target_cloud = *downsampled;
    

    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*downsampled);
    source_cloud = downsampled;

    ros::Time::init();

    // benchmark  normal ndt's time too long so commnet it
    /*
    std::cout << "the " << q << " loop --- pcl::NDT ---" <<  std::endl;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt->setResolution(1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = align(ndt, target_cloud, source_cloud);
    */

    std::vector<int> num_threads = {1, omp_get_max_threads()};
    std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
      {"KDTREE", pclomp::KDTREE},
      {"DIRECT7", pclomp::DIRECT7},
      {"DIRECT1", pclomp::DIRECT1}
    };

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setResolution(1.0);

    //for(int n : num_threads) {
      //for(const auto& search_method : search_methods) {
        //std::cout << "--- pclomp::NDT (" << search_method.first << ", " << n << " threads) ---" << std::endl;
        int n = omp_get_max_threads(); //get the max num of the threads
        cout << "max threads:" << n << endl;
        ndt_omp->setNumThreads(n);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = align(ndt_omp, target_cloud, source_cloud);
        Eigen::Matrix4f trans = ndt_omp->getFinalTransformation();
        allTran.push_back(trans);
        //cout << trans << endl;
      //}
    //}

    /*
    //icp
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud); 
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    cout << "icp has converged: " << icp.hasConverged() << endl;
    */

  //-------------------------------------
  
  //Eigen::Matrix4f t_localizer(ndt_omp->getFinalTransformation()); //by the ndt-omp
  //Eigen::Matrix4f icp_tran(icp.getFinalTransformation()); //by the icp 

  //out << "translate by the icp,No: " << q << endl;
  //out << icp_tran << endl;

  /* 
  vector<Eigen::Matrix3d> mat_l;
  
  mat_l.setValue(
  
  static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),static_cast<double>(t_localizer(0, 2)),
  
  static_cast<double>(t_localizer(1, 0)),static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
  
  static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),static_cast<double>(t_localizer(2, 2))
  
  );
  */

  cout << "For cloud "  << q << " and " << q+1 << " is finished." << endl; 

} //run the loop of each 2 pcd compare
 



  //打开文件
  ofstream out;
  string filepath = path + "resultInL.csv";
  out.open(filepath,ios::out);
  for(int q = 0; q < allTran.size();q++){
    out << "translate by the ndt_omp,No: " << q << endl;
    out << allTran[q] << endl ;
  }



  // visulization
  /*
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(aligned, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(aligned, aligned_handler, "aligned");
  vis.spin();
  }
 */
    /*
    ofstream out;
    out.open("resultInL.csv",ios::out);
    out << "time" << ',' << "z" << ',' << "y" << ',' << "x" << endl;
    std::vector<pose>::iterator it2;
    it2 = allPose.begin();
    float q = 0;
    for(;it2 != allPose.end();it2++){
       out << q << ',' << (*it2).roll << ',' << (*it2).yaw << ',' << (*it2).pitch << endl;
       q += 0.1;
    }
    */

  return 0;
}


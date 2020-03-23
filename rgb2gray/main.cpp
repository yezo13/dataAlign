#include<iostream>
#include <eigen3/Eigen/Dense> //before the opencv include
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>
#include <dirent.h>

using namespace cv;
using namespace std;

vector<string> allimg;

bool getImages(string path,vector<string>& files)
{
    string imgPath = path;
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(imgPath.c_str())) == NULL) {
        cout<<"failed to get the images!"<<endl;
        return false;;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string name = std::string(dirp->d_name);
        //cout<<name<<endl;
        if ( name != "."  &&  name != ".."  &&  (name.substr(name.size() - 3, name.size()) == "png" || name.substr(name.size() - 3, name.size()) == "jpg") ){ //because search will read the . and .. 
	   // cout << name << endl;
	  /*
	  while(name.length() < 9){ //full the file name to sort 
	      name = "0" + name;
	  }
	  */
          files.push_back(name);
	}
    }
    closedir(dp);
    std::sort(files.begin(), files.end());
    for(int q = 0;q < files.size();q++){
    
      cout << files[q] << endl;
    }
    cout<<"Got the images."<<endl;
    return true;
}


int main()
{
  string writepath = "/home/yezotemp/dataset/2011_09_26/school_01/image_00/data/";
  string readpath = "/home/yezotemp/dataset/school/stereo_school1/undistortedleft/";
  getImages(readpath,allimg);
  for(int q = 0;q < allimg.size();q++){
    string currimg = readpath + allimg[q];
    Mat src = imread(currimg,1);
    Mat dst;
    cvtColor(src,dst,CV_BGR2GRAY);
    string tempName = allimg[q];
    while(tempName.length() < 9){
      tempName = "0" + tempName;
    }
    string writefile = writepath + tempName;
    imwrite(writefile,dst);
    cout << "finish " << q << " frame change" << endl; 
  }
  
  return 0;
}
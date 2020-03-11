#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <thread>
#include "OpticalSLAM.h"
#include "params.h"

using namespace std;

const string path = "/home/yezotemp/dataset/2011_09_26/sync_0009/";

bool getImages(vector<string>& files)
{
    string imgPath = path+"image_00/data/";
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
	  while(name.length() < 9){ //full the file name to sort 
	      name = "0" + name;
	  }
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
	bool is_first_img = true;
	vector<string> sImgs;
	if(!getImages(sImgs))
	{
		cout<<"Failed to get the images!"<<endl;
		return -1;
	}

	set_lab_Params();
	OpticalSLAM slam_system;
	slam_system.initialize();
	 //Mat img_to_display;
	std::thread slam_thread([&](){
		for(int i = 0;i < sImgs.size(); ++i)
		{
			string strLeftImg = path + "image_02/data/" + sImgs[i];
			string strRightImg = path + "image_03/data/" + sImgs[i];
			cout << strLeftImg << endl << strRightImg << endl;
			/* use gray img
			Mat leftImg = imread(strLeftImg, 0);;
			Mat rightImg = imread(strRightImg, 0);;
			*/
			Mat leftrgbImg = imread(strLeftImg);  //imread function second params means 0-gary   1-rgb
			Mat rightrgbImg = imread(strRightImg);
			Mat leftImg;
			Mat rightImg;
			cvtColor(leftrgbImg,leftImg,CV_RGB2GRAY);
			cvtColor(rightrgbImg,rightImg,CV_RGB2GRAY);
			cout << leftImg.type() << endl;
			//img_to_display = leftImg.clone();
			//imshow("display", img_to_display);
			if(is_first_img)
			{
				slam_system.setFirstFrame(leftImg, rightImg);
				is_first_img = false;
			}
			else
				slam_system.trackStereoFrames(leftImg, rightImg);
			if(i >= WINDOW_SIZE)
				std::this_thread::sleep_for(chrono::milliseconds(200)); 
			if(i == sImgs.size() -1){
				vector<Matrix3d>::iterator it;
				vector<Matrix3d>::iterator it2;
				int count = 1;
				//it2 = slam_system.state_estimator.img_manager.allRp.begin();
				for(it = slam_system.Rp.begin();it != slam_system.Rp.end();it++){
				  slam_system.state_estimator.img_manager.allRp.push_back(*it); 
				}
				//because of the last 10 matrix do not be replaced, so we push it in the allRP vector when the last cycle, which make the allRp save all the 106 trans
				vector<Vector3d> euler_angles;
				it2 = slam_system.state_estimator.img_manager.allRp.begin();
				
				for(;it2 != slam_system.state_estimator.img_manager.allRp.end();it2++){
				  cout << "Rp rotation matrix: " << count << endl   << (*it2).matrix() << endl;
				  euler_angles.push_back((*it2).matrix().eulerAngles(2,1,0));
				  count++; 
				}
				
				/*
				for(it = euler_angles.begin();it != euler_angles.end();it++){
				  cout << "The time is " << frame_time[count++] << "and the angle is" << (*it).transpose() << endl;
				}
				*/
				
				/*
				out << "x" << ',' << "y" << ',' << "z" << endl;
				it3 = euler_angles.begin();
				for (int q = 0; q < euler_angles.size()-1;q++){
				    Eigen::Vector3d last_angle = (*it3).transpose(); // yaw-x   pitch-y  roll-z
				    it3++;
				    Eigen::Vector3d curr_angle = (*it3).transpose();
				    Eigen::Vector3d div_angle = curr_angle - last_angle;
				    double div_time = 0.1;
				    //cout << "and the angle speed is " << div_angle / div_time << endl;
				    //judge if the speed is reliable or not.
				    if(((div_angle(0)/div_time < 2) && (div_angle(1)/div_time < 2) && (div_angle(2)/div_time < 2))
				      && ((div_angle(0)/div_time > -2) && (div_angle(1)/div_time > -2) && (div_angle(2)/div_time > -2))
				    )
				      out << div_angle(0)/div_time << ',' << div_angle(1)/div_time << ','<< div_angle(2)/div_time << endl;

				}
				*/
				ofstream out;
				out.open("resultRp.csv",ios::out);
				it2 = slam_system.state_estimator.img_manager.allRp.begin();
				count = 0;
				for(;it2 != slam_system.state_estimator.img_manager.allRp.end();it2++){
				  out << "rotation matrix Rp: " << count << endl  << (*it2).matrix() << endl;
				  count++; 
				}
				out.close();

				out.open("resultRpnp.csv",ios::out);
				it2 = slam_system.state_estimator.img_manager.allRpnp.begin();
				count = 0;
				for(;it2 != slam_system.state_estimator.img_manager.allRpnp.end();it2++){
				  out << "rotation matrix Rpnp: " << count << endl  << (*it2).matrix() << endl;
				  count++; 
				}
				out.close();
				
				out.open("resultRpnp2.csv",ios::out);
				it2 = slam_system.state_estimator.img_manager.allRpnp2.begin();
				count = 0;
				for(;it2 != slam_system.state_estimator.img_manager.allRpnp2.end();it2++){
				  out << "rotation matrix Rpnp: " << count << endl  << (*it2).matrix() << endl;
				  count++; 
				}
				out.close();
				
				out.open("TpResult.csv",ios::out);
				vector<Vector3d>::iterator it3;
				it3 = slam_system.state_estimator.img_manager.allTp.begin();
				count = 0;
				for(;it3 != slam_system.state_estimator.img_manager.allTp.end();it3++){
				  out << "Tp: " << count << endl  << (*it3) << endl;
				  count++; 
				}
				out.close();
			    
			}
		}

	});
	

	

	slam_system.display();

	slam_thread.join();
	return 0;
}
#include "converter.h"
#include "localmapping.h"
#include "map.h"
#include "utils.h"
#include "visualize.h"
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	cout<<"Hello Mapping Module 2"<<endl;

	int n_path = 3;

	vector<string> csvs_path_pose;
	csvs_path_pose.push_back("../pose_csv/path1-vins_estimator-camera_pose.csv");
	csvs_path_pose.push_back("../pose_csv/path2-vins_estimator-camera_pose.csv");
	csvs_path_pose.push_back("../pose_csv/path3-vins_estimator-camera_pose.csv");

	vector<string> csvs_path_vertice;
	csvs_path_vertice.push_back("../imgs/path1_undistorted.csv");
	csvs_path_vertice.push_back("../imgs/path2_undistorted.csv");
	csvs_path_vertice.push_back("../imgs/path3_undistorted.csv");

	vector<string> imgs_paths;
	imgs_paths.push_back("../imgs/path1_undistorted/");
	imgs_paths.push_back("../imgs/path2_undistorted/");
	imgs_paths.push_back("../imgs/path3_undistorted/");

	vector<double> timestarts;
	timestarts.push_back(1554873625.05);
	timestarts.push_back(1554873724.96);
	timestarts.push_back(1554873903.91);

	vector<double> timeends;
	timeends.push_back(1554873636.11);
	timeends.push_back(1554873731.34);
	timeends.push_back(1554873908.16);
	
	vector<Mat> imgs_total;
	vector<Mat> Tcws_total;
	vector<vector<Point2f>> verticess_total;
	for(int i=0; i<n_path; i++)
	{
		//read camera pose
		vector<string> imgtimes; 
		vector<Mat> Twcs; 
    	Utils::ReadCameraTimeAndPose(csvs_path_pose[i], timestarts[i], timeends[i], imgtimes, Twcs);
		
		//read vertices amd imgnames
		vector<string> imgnames; 
		vector<vector<Point2f>> verticess;
		Utils::ReadImgNameAndVertices(csvs_path_vertice[i], imgtimes, imgnames, verticess);
		
		//get imgs
		vector<Mat> imgs;
		for(int j=0; j<imgnames.size(); j++)
		{
			imgs.push_back(imread(imgs_paths[i]+imgnames[j], 1));
		}

		//inverse Twcs
		vector<Mat> Tcws;
		for(int j=0; j<Twcs.size(); j++)
		{
			Tcws.push_back(Twcs[j].inv());
		}

		for(int j=0; j<imgs.size(); j++)
		{	
			imgs_total.push_back(imgs[j].clone());
			Tcws_total.push_back(Tcws[j].clone());
			verticess_total.push_back(verticess[j]);
		}
	}	


	//check
	cout<<"number of frames: "<<imgs_total.size()<<endl;
	// for(int i=0; i<imgs_total.size(); i++)
	// {
	// 	Mat img = imgs_total[i];
	// 	for(int j=0; j<verticess_total[i].size(); j++)
	// 		cv::circle(img, verticess_total[i][j], 2, cv::Scalar(255,0,0), -1);

	// 	imshow("img", img);
	// 	waitKey(0);
	// }

	//K
    cv::Mat K = cv::Mat(3,3, CV_32F);
    K.at<float>(0,0) = 712.58465576171875000; K.at<float>(0,1) = 0; K.at<float>(0,2) = 613.71289062500000000;
    K.at<float>(1,0) = 0; K.at<float>(1,1) = 713.57849121093750000; K.at<float>(1,2) = 386.50469970703125000;
    K.at<float>(2,0) = 0; K.at<float>(2,1) = 0; K.at<float>(2,2) = 1;


	//mapping
	Map* map = new Map();
	LocalMapping* localmapping = new LocalMapping(map, K, imgs_total, Tcws_total, verticess_total);
	//localmapping->Run();
	localmapping->RunAllBA();
	//localmapping->RunRansacAllBA(0.01, 300);
	
	map = localmapping->GetMap();
	std::vector<MapPoint*> mappoints =  map->GetAllMappoints();
	
	// //visualize
	Visualize::VisualizeMapPoint(mappoints[0]);


	//                  1 2
	// ground truth 
	//                  4 3
	vector<float> gt;
	gt.push_back(4.1804);
	gt.push_back(4.2600);
	gt.push_back(4.2032);
	gt.push_back(4.1300);

	for(int i=0; i<4; i++)
	{
		cv::Point3f pose = mappoints[i]->GetWorldPose() + Point3f(0,0,0.186);
		cout<<"vertice: "<<i<<"   3D pose: "<<pose<<"   "<<"error:  "<<abs(gt[i]-norm(pose))<<"m   "<<abs(gt[i]-norm(pose))/gt[i]*100<<"percent "<<endl;
	}

	delete localmapping;
	delete map;

	return 0;
}
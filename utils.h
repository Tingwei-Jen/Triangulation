#pragma once
#ifndef UTILS_H
#define UTILS_H
#include <opencv2/opencv.hpp>
#include <string>  

class Utils
{
public:
    static cv::Mat Quaternion2RotM(const float& x, const float& y, const float& z, const float& w);
    
    
    static void ReadCameraTimeAndPose(const std::string& csv_path_pose, const double& timestart, const double& timeend,
                                        std::vector<std::string>& imgtimes, std::vector<cv::Mat>& Twcs); 

    static void ReadImgNameAndVertices(const std::string& csv_path_vertice, const std::vector<std::string>& imgtimes, 
                                        std::vector<std::string>& imgnames, std::vector<std::vector<cv::Point2f>>& verticess);


};
#endif //UTILS_H

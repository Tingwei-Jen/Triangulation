#pragma once
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H
#include "map.h"
#include <opencv2/opencv.hpp>

class LocalMapping
{
public:
	LocalMapping(Map* map, const cv::Mat& K, 
                const std::vector<cv::Mat>& imgs, 
                const std::vector<cv::Mat>& Tcws,
                const std::vector<std::vector<cv::Point2f>>& verticess);

	void Run();       //Seperate vertice for BA, once a vertice into BA.
    void RunAllBA();  //BA all vertice together 

    void RunRansacAllBA(float avg_dist_thres = 0.01 ,int nIterations = 10); //Dist error is average of 4 vertices

	Map* GetMap(){ return mMap; }

private:
    bool Triangulation(const std::vector<cv::Mat>& Tcws, const std::vector<cv::Point3f>& pts_cam, cv::Point3f& x3Dp);
    void ComputeError(std::vector<Frame*> frames, const int& idx, const cv::Point3f& pt3d, float& avg_dist, float& avg_px_error);
    void ComputeError(std::vector<Frame*> frames, const std::vector<cv::Point3f>& pts3d, 
                        std::vector<float>& avg_dists, std::vector<float>& avg_px_errors);

private: 
    float Dist3Dpoint2Pxline(const cv::Point3f& x3Dp, Frame* frame, const int& index);
    void RandomNumbers(const int& min, const int& max, const int& n, const int& nIter, int arr[]);
    cv::Point3f Px2Cam(const cv::Point2f& px, const float& fx, const float& fy, const float& cx, const float& cy);
    cv::Point2f Cam2Px(const cv::Point3f& p_cam, const float& fx, const float& fy, const float& cx, const float& cy);
    cv::Point3f World2Cam(const cv::Point3f& p_world, const cv::Mat& Tcw);
    cv::Point3f Cam2World(const cv::Point3f& p_cam, const cv::Mat& Twc);

private:
	Map* mMap;
    cv::Mat mK;
    float fx, fy, cx, cy;
    int mNframes;
    int mNvertices;
    std::vector<cv::Mat> mImgs;
    std::vector<cv::Mat> mTcws;
    std::vector<std::vector<cv::Point2f>> mVerticess;

};
#endif //LOCALMAPPING_H
#pragma once
#ifndef FRAME_H
#define FRAME_H
#include <opencv2/opencv.hpp>

class Frame
{
public:
	Frame(const cv::Mat& img, const cv::Mat &K, const std::vector<cv::Point2f>& Vertices);
	void SetPose(const cv::Mat& Tcw);
    cv::Mat GetImg(){ return mImg.clone(); }
	cv::Mat GetPose(){ return mTcw.clone(); }
    cv::Mat GetCameraCenter(){ return mOw.clone(); }
    cv::Mat GetRotation(){ return mRcw.clone(); }
    cv::Mat GetTranslation(){ return mtcw.clone(); }
    cv::Mat GetRotationInverse(){ return mRwc.clone(); }
    std::vector<cv::Point2f> GetVertices(){ return mVertices; }

public:
    static int frame_counter;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    int mId;
    cv::Mat mK;

private:
	cv::Mat mImg;
	cv::Mat mTcw;                                          ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵
    cv::Mat mRcw;                                          ///< Rotation from world to camera
    cv::Mat mRwc;                                          ///< Rotation from camera to world
    cv::Mat mtcw;                                          ///< Translation from world to camera   
    cv::Mat mOw;                                           ///< mtwc,Translation from camera to world
    std::vector<cv::Point2f> mVertices;
};
#endif //FRAME_H

#include "frame.h"

int Frame::frame_counter = 0;
float Frame::fx, Frame::fy, Frame::cx, Frame::cy, Frame::invfx, Frame::invfy;

Frame::Frame(const cv::Mat& img, const cv::Mat &K, const std::vector<cv::Point2f>& Vertices)
:mImg(img.clone()), mK(K.clone())
{
	this->mId = frame_counter++;

    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    invfx = 1.0/fx;
    invfy = 1.0/fy;

    for(int i=0; i<Vertices.size(); i++)
        this->mVertices.push_back(Vertices[i]);
}

void Frame::SetPose(const cv::Mat& Tcw)
{
	this->mTcw = Tcw.clone();
    this->mRcw = this->mTcw.rowRange(0,3).colRange(0,3);
    this->mRwc = this->mRcw.t();
    this->mtcw = this->mTcw.rowRange(0,3).col(3);
    this->mOw = -this->mRcw.t()*this->mtcw;
}
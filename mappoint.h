#pragma once
#ifndef MAPPOINT_H
#define MAPPOINT_H
#include "frame.h"

class MapPoint
{
public:
    MapPoint();
    void SetWorldPose(const cv::Point3f& P);
    void AddObservation(Frame* frame, const cv::Point2f& px);
    cv::Point3f GetWorldPose(){ return mP; }
	std::vector<Frame*> GetObservationFrames(){ return mObservationFrames; }
	std::vector<cv::Point2f> GetObservationPxs(){ return mObservationPxs; }

public:
	static int mappoint_counter;
	int mId;  
	int nObs;

private:
    cv::Point3f mP;
	std::vector<Frame*> mObservationFrames;
	std::vector<cv::Point2f> mObservationPxs;

};
#endif //MAPPOINT_H
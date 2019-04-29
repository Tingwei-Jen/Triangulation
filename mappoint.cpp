#include "mappoint.h"

int MapPoint::mappoint_counter = 0;

MapPoint::MapPoint():nObs(0)
{
	this->mId = mappoint_counter++;
}

void MapPoint::SetWorldPose(const cv::Point3f& P)
{
    this->mP = P;
}

void MapPoint::AddObservation(Frame* frame, const cv::Point2f& px)
{
	this->mObservationFrames.push_back(frame);
	this->mObservationPxs.push_back(px);
	this->nObs++;
}
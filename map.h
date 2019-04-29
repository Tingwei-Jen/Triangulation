#pragma once
#ifndef MAP_H
#define MAP_H
#include "frame.h"
#include "mappoint.h"

class Map
{
public:
	Map();
	void AddFrame(Frame* frame);
	void AddMapPoint(MapPoint* mappoint);
	std::vector<Frame*> GetAllFrames(){ return mFrames; }
	std::vector<MapPoint*> GetAllMappoints(){ return mMappoints; }

private:
	std::vector<Frame*> mFrames;
	std::vector<MapPoint*> mMappoints;

};
#endif //MAP_H
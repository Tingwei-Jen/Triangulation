#include "map.h"

Map::Map()
{
}

void Map::AddFrame(Frame* frame)
{
	this->mFrames.push_back(frame);
}

void Map::AddMapPoint(MapPoint* mappoint)
{
	this->mMappoints.push_back(mappoint);
}

// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#include "map.h"

namespace simple_vo
{
	
void Map::insertKeyFrame(Frame::Ptr frame)
{
	cout<<"Size of Keyframes is :"<<Keyframes_.size()<<endl;
	if(Keyframes_.find(frame->id_) == Keyframes_.end())
	{
		Keyframes_.insert(make_pair(frame->id_, frame));
	}
	else
	{
		Keyframes_[frame->id_] = frame;
	}
}

void Map::insertMapPoint(Mappoint::Ptr map_point)
{
	if(map_points_.find(map_point->id_) == map_points_.end())
	{
		map_points_.insert(make_pair(map_point->id_, map_point));
	}
	else
	{
		map_points_[map_point->id_] = map_point;
	}
}

}
// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#ifndef MAP_H
#define MAP_H

#include "common.h"
#include "frame.h"
#include "mappoint.h"

namespace simple_vo
{

class Map
{
public:
	typedef std::shared_ptr<Map> Ptr;
	unordered_map<unsigned long, Mappoint::Ptr> map_points_;
	unordered_map<unsigned long, Frame::Ptr> Keyframes_;

	Map(){}

	void insertKeyFrame(Frame::Ptr frame);

	void insertMapPoint(Mappoint::Ptr mao_point);

};

}



#endif
// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common.h"

namespace simple_vo
{

class Frame;
class Mappoint
{
public:
	static unsigned long              factory_id_;
	bool 							  good_;
	Eigen::Vector3d					  norm_;
	list<Frame*> 					  observed_frames_;


public:
	int  							  matched_times_;	
	cv::Mat  						  descriptor_;
	int 							  visible_times_;	
	Eigen::Vector3d  				  pos_;
	unsigned long 					  id_;

	typedef std::shared_ptr<Mappoint> Ptr;

	Mappoint();
	Mappoint(unsigned long id, const Eigen::Vector3d& pos, const Eigen::Vector3d& norm, Frame* frame=nullptr, const cv::Mat& descriptor=cv::Mat());

	inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }



	static Mappoint::Ptr createMapPoint();
	static Mappoint::Ptr createMapPoint(
		const Eigen::Vector3d& pos_world,
		const Eigen::Vector3d& norm,
		const cv::Mat& descriptor,
		Frame* frame);

};

}


#endif
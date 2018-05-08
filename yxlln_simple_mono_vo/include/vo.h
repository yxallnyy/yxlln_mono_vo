// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#ifndef VO_H
#define VO_H

#include "common.h"
#include "frame.h"
#include "map.h"
#include <opencv2/features2d/features2d.hpp>

namespace simple_vo
{

class VO
{
public:
	enum VOState
	{
		INIT = -1,
		OK = 0,
		LOST
	};

	VOState                         state_;
	Map::Ptr      			        map_;
	Frame::Ptr                      ref_;
	Frame::Ptr                	    curr_;

	cv::Ptr<cv::ORB>          		orb_;
	std::vector<cv::KeyPoint> 		KeyPoints_curr_;
	cv::Mat 						descriptors_curr_;
	cv::FlannBasedMatcher           matcher_flann_;
	std::vector<Mappoint::Ptr>      match_3dpts_;
	std::vector<int>                match_2dkp_index_;

	Sophus::SE3                     T_c_w_estimated_;
	int 							num_inliers_;
	int                             num_losts_;

	int 							num_of_features_;
	double                          scale_factor_;
	int 							level_pyramid_;
	float 							match_ratio_;
	int 							max_num_losts_;
	int 							min_inliners_;
	double 							key_frame_min_rotation_;
	double 							key_frame_min_transpose_;
	double 							map_point_erase_ratio_;

public:
	typedef std::shared_ptr<VO> Ptr;

	VO();
	~VO();
	bool addFrame(Frame::Ptr frame);

protected:
	void extractKeyPoints();
	void computeDescriptors();
	void featuresMatching();
	void poseEstimationPnP();
	void optimizeMap();

	void addKeyFrame();
	void addMappoints();
	bool checkEstimationPose();
	bool checkKeyFrame();

	double getViewAngle(Frame::Ptr frame, Mappoint::Ptr point);
};



}

#endif
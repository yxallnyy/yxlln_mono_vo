// Author : yxlln
// Date : 2018.5.5
//Description : simple mono vo 

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include "vo.h"
#include "config.h"
#include "g2o_types.h"

namespace simple_vo
{

VO::VO():
state_(INIT), ref_(nullptr), curr_(nullptr), map_(new Map), num_losts_(0), num_inliers_(0), matcher_flann_(new cv::flann::LshIndexParams(5,10,2))
{
	num_of_features_ = Config::get<int>("number_of_features");
	scale_factor_ = Config::get<double>("scale_factor");
	level_pyramid_ = Config::get<int>("level_pyramid");
	match_ratio_ = Config::get<float>("match_ratio");
	max_num_losts_ = Config::get<int>("max_num_lost");
	min_inliners_ = Config::get<int>("min_inliers");
	key_frame_min_rotation_ = Config::get<double>("keyframe_rotation");
	key_frame_min_transpose_ = Config::get<double>("keyframe_translation");
	map_point_erase_ratio_ = Config::get<double>("map_point_erase_ratio");
	orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

VO::~VO()
{

}

bool VO::addFrame(Frame::Ptr frame)
{	
	num_losts_ = 0;

	switch(state_)
	{
	case INIT:
	{
		state_ = OK;
		curr_ = ref_ = frame;
		extractKeyPoints();
		computeDescriptors();
		addKeyFrame();
		break;
	}
	case OK:
	{
		curr_ = frame;
		curr_->T_c_w_ = ref_->T_c_w_;
		extractKeyPoints();
		computeDescriptors();
		featuresMatching();
		poseEstimationPnP();
		if(checkEstimationPose() == true)
		{
			curr_->T_c_w_ = T_c_w_estimated_;
			optimizeMap();
			//num_losts_ = 0;
			if(checkKeyFrame() == true)
			{
				addKeyFrame();
			}
		}
		else
		{
			num_losts_++;
			if(num_losts_ > max_num_losts_)
			{
				state_ = LOST;
			}
			return false;
		}
		break;
	}
	case LOST:
	{
		cout<<"LOST!!!!"<<endl;
		break;
	}
	}
	return true;
}

void VO::extractKeyPoints()
{
	boost::timer timer;

	orb_->detect(curr_->color_, KeyPoints_curr_);

	cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;
}

void VO::computeDescriptors()
{
	boost::timer timer;

	orb_->compute(curr_->color_, KeyPoints_curr_, descriptors_curr_);

    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}

void VO::featuresMatching()
{
	boost::timer timer;
	std::vector<cv::DMatch> matches;
	cv::Mat mat_descriptor;
	std::vector<Mappoint::Ptr> candidate_points;

	for(auto& points:map_->map_points_)
	{
		Mappoint::Ptr& p = points.second;
		if(curr_->isInFrame(p->pos_))
		{
			p->visible_times_++;
			candidate_points.push_back(p);
			mat_descriptor.push_back(p->descriptor_);
		}
	}

	matcher_flann_.match(mat_descriptor, descriptors_curr_, matches);

	float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();

    for(cv::DMatch& m:matches)
    {
    	if(m.distance < max<float>(min_dis * match_ratio_, 30.0))
    	{
    		match_3dpts_.push_back(candidate_points[m.queryIdx]);
    		match_2dkp_index_.push_back(m.trainIdx);
    	}
    }

    cout<<"good matches: "<<match_3dpts_.size() <<endl;
    cout<<"match cost time:"<<timer.elapsed()<<endl;
}

void VO::poseEstimationPnP()
{
	std::vector<cv::Point3f> pts3d;
	std::vector<cv::Point2f> pts2d;

	for(int index:match_2dkp_index_)
	{
		pts2d.push_back(KeyPoints_curr_[index].pt);
	}

	for(Mappoint::Ptr p:match_3dpts_)
	{
		pts3d.push_back(p->getPositionCV());
		//pts3d.push_back( pt->getPositionCV() );
	}

	cv::Mat K = (cv::Mat_<double>(3,3) << 
				ref_->camera_->fx_, 0, ref_->camera_->cx_,
				0, ref_->camera_->fy_, ref_->camera_->cy_,
				0,0,1);

	cv::Mat rvec, tvec, inliers;

    cv::solvePnPRansac ( pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );

    num_inliers_ = inliers.rows;
    cout<<"inliers is :"<<num_inliers_<<endl;

    T_c_w_estimated_ = Sophus::SE3(
    							 Sophus::SO3(rvec.at<double>(0,0), rvec.at<double>(1,0),rvec.at<double>(2,0)),
    							 Eigen::Vector3d(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0)));

    //BA
    //vertex
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
    							   T_c_w_estimated_.rotation_matrix(),T_c_w_estimated_.translation()));
    optimizer.addVertex(pose);

    //edges
    for(int i=0;i<inliers.rows;i++)
    {
    	int index = inliers.at<int>(i,0);
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
    	edge->setVertex(0,pose);
    	edge->camera_ = curr_->camera_.get();         //???
    	edge->point_ = Eigen::Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
    	edge->setMeasurement(Eigen::Vector2d(pts2d[index].x, pts2d[index].y));
    	edge->setInformation ( Eigen::Matrix2d::Identity() );
    	optimizer.addEdge ( edge );
    	match_3dpts_[index]->matched_times_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    T_c_w_estimated_ = Sophus::SE3 (
        							pose->estimate().rotation(),
        							pose->estimate().translation());
    
    cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;

}

void VO::optimizeMap()
{
    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        
        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }
    
    if ( match_2dkp_index_.size()<100 )
        addMappoints();
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}

double VO::getViewAngle(Frame::Ptr frame, Mappoint::Ptr point)
{
    Eigen::Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos(n.transpose()*point->norm_);
}

void VO::addKeyFrame()
{
	if(map_->Keyframes_.empty())
	{
		for(size_t i=0;i<KeyPoints_curr_.size();i++)
		{
			double d = curr_->findDepth(KeyPoints_curr_[i]);
			if(d<0)
			{
				continue;
			}
			Eigen::Vector3d p_world = ref_->camera_->pixel2world(Eigen::Vector2d(KeyPoints_curr_[i].pt.x, KeyPoints_curr_[i].pt.y), curr_->T_c_w_, d);
			Eigen::Vector3d n = p_world - ref_->getCamCenter();
			n.normalize();
			Mappoint::Ptr map_point = Mappoint::createMapPoint(p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
			map_->insertMapPoint( map_point );
		}
	}

	map_->insertKeyFrame ( curr_ );
    ref_ = curr_;
}

void VO::addMappoints()
{
	std::vector<bool> matched(KeyPoints_curr_.size(), false); 
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    for ( int i=0; i<KeyPoints_curr_.size(); i++)
    {
        if ( matched[i] == true)   
            continue;
        double d = ref_->findDepth(KeyPoints_curr_[i]);
        if ( d<0 )  
            continue;
        Eigen::Vector3d p_world = ref_->camera_->pixel2world(Eigen::Vector2d (KeyPoints_curr_[i].pt.x, KeyPoints_curr_[i].pt.y), 
            										   curr_->T_c_w_, d);
        Eigen::Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        Mappoint::Ptr map_point = Mappoint::createMapPoint(p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
        map_->insertMapPoint(map_point);
    }
}
bool VO::checkEstimationPose()
{
	if(num_inliers_<min_inliners_)
	{
		cout<<"error:too small inliers..."<<endl;
		return false;
	}

	Sophus::SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
	Sophus::Vector6d d = T_r_c.log();
	if(d.norm() > 5.0)
	{
		cout<<"reject because motion is too large: "<<d.norm() <<endl;
		return false;
	}
	return true;
}

bool VO::checkKeyFrame()
{
	Sophus::SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
	Sophus::Vector6d d = T_r_c.log();
	Eigen::Vector3d trans = d.head<3>();
	Eigen::Vector3d rotat = d.tail<3>();
	if(trans.norm() > key_frame_min_transpose_ || rotat.norm() > key_frame_min_rotation_)
	{
		return true;
	}
	return false;
}

}
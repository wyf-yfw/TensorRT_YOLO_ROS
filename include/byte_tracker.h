#pragma once

#include "strack.h"
#include <InferResult.h>
#include <Results.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
class detect_result
{
public:
    int classId;
    float conf;
    cv::Rect_<float> box;
};


class BYTEtracker
{
public:
    BYTEtracker(int frame_rate = 30, int track_buffer = 30);
	~BYTEtracker();
    std::vector<strack> update(std::vector<tensorrt_yolo::InferResult>& objects);
    cv::Scalar get_color(int idx);
	 std::vector<strack*> joint_stracks(std::vector<strack*> &tlista, std::vector<strack> &tlistb);
	 std::vector<strack> joint_stracks(std::vector<strack> &tlista, std::vector<strack> &tlistb);

	 std::vector<strack> sub_stracks(std::vector<strack> &tlista, std::vector<strack> &tlistb);
	void remove_duplicate_stracks(std::vector<strack> &resa, std::vector<strack> &resb, std::vector<strack> &stracksa, std::vector<strack> &stracksb);

	void linear_assignment( std::vector< std::vector<float> > &cost_matrix, int cost_matrix_size, int cost_matrix_size_size, float thresh,
		 std::vector< std::vector<int> > &matches,  std::vector<int> &unmatched_a,  std::vector<int> &unmatched_b);
	 std::vector< std::vector<float> > iou_distance(std::vector<strack*> &atracks, std::vector<strack> &btracks, int &dist_size, int &dist_size_size);
	 std::vector< std::vector<float> > iou_distance(std::vector<strack> &atracks, std::vector<strack> &btracks);
	 std::vector< std::vector<float> > ious( std::vector< std::vector<float> > &atlbrs,  std::vector< std::vector<float> > &btlbrs);

	static double lapjv(const  std::vector< std::vector<float> > &cost,  std::vector<int> &rowsol,  std::vector<int> &colsol,
		bool extend_cost = false, float cost_limit = LONG_MAX, bool return_cost = true);

private:

	float track_thresh;
	float high_thresh;
	float match_thresh;
	int frame_id;
	int max_time_lost;

	 std::vector<strack> tracked_stracks;
	 std::vector<strack> lost_stracks;
	 std::vector<strack> removed_stracks;
	byte_kalman::ByteKalmanFilter kalman_filter;
};

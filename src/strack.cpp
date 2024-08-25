#include "strack.h"
#include <ros/ros.h>
#include <utility>

strack::strack(std::vector<float> tlwh_, float score, int class_id,std::vector<float> kpts, std::vector<tensorrt_yolo::KeyPoint> vKpts)
{
	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

    vKpts_ = std::move(vKpts);
    kpts_ = std::move(kpts);
	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	classId_ = class_id;
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	start_frame = 0;
}

strack::~strack()
{
}

void strack::activate(byte_kalman::ByteKalmanFilter &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id();

	 std::vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	 std::vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;

}

void strack::re_activate(strack &new_track, int frame_id, bool new_id)
{
	 std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
    this->classId_ = new_track.classId_;
    this->kpts_ = new_track.kpts_;
    this->vKpts_ = new_track.vKpts_;
	if (new_id)
		this->track_id = next_id();
}

void strack::update(strack &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	 std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->state = TrackState::Tracked;
	this->is_activated = true;

    // 更新 kpts 和 vKpts
    this->score = new_track.score;
    this->classId_ = new_track.classId_;
    this->kpts_ = new_track.kpts_;  // 更新关键点
    this->vKpts_ = new_track.vKpts_;  // 更新关键点

}

void strack::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void strack::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

 std::vector<float> strack::tlwh_to_xyah(std::vector<float> tlwh_tmp)
{
	 std::vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

 std::vector<float> strack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

 std::vector<float> strack::tlbr_to_tlwh(std::vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void strack::mark_lost()
{
	state = TrackState::Lost;
}

void strack::mark_removed()
{
	state = TrackState::Removed;
}

int strack::next_id()
{
	static int _count = 0;
	_count++;
	return _count;
}

int strack::end_frame()
{
	return this->frame_id;
}

void strack::multi_predict(std::vector<strack*> &stracks, byte_kalman::ByteKalmanFilter &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			stracks[i]->mean[7] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
	}
}
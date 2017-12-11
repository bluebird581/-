#ifndef LANE_TRACKING_LANE_H
#define LANE_TRACKING_LANE_H

#include "basic_struct.h"

struct BORDER {
    LANE left_border;
    LANE right_border;

    float conf;
};

class SLLaneTracking {
public:
    SLLaneTracking(
        const cv::Rect& roi_out,
        const int& start_channel,
        const IPMTransformer& ipm_transformer,
        const cv::Point2f& camera_pos_ipm) {
        _ret.channel = -1000;
        _ret.delta_x = -1000.0f;
        _ret.angle = 0;
        _ret_old.channel = start_channel;
        _ret_old.delta_x = -1000.0f;
        _ret_old.angle = 0;
        _minimal_record = 5;// 9;
        _minimal_dist = 40;// 20;
        _mininal_good_ratio = 0.4;
        _ipm_transformer = ipm_transformer;
        _camera_pos_ipm = camera_pos_ipm;
        _roi_out = roi_out;
    }
    int do_tracking(std::vector<LANE>& lanes, const int& channel_num);
    LANERET get_ret() {
        return _ret;
    }

private:
    LANE estimate_lane(const std::deque<LANE>& lanes);
    BORDER estimate_border();
    LANE init_lane_from_lanes(const std::vector<LANE>& lanes);
    LANE left_from_center_and_right(const LANE& center, const LANE& right);
    LANE right_from_left_and_center(const LANE& left, const LANE& center);

private:
    cv::Rect _roi_out;
    LANERET _ret;
    LANERET _ret_old;

    std::deque<BORDER> _borders;

    IPMTransformer _ipm_transformer;
    cv::Point2f _camera_pos_ipm;

    int _minimal_record; // the buffe length
    int _minimal_dist; // the distance between reference and test
    int _mininal_good_ratio; // the ratio of good samples
};

/////////////////////////// detailed function

LANE SLLaneTracking::right_from_left_and_center(const LANE& left, const LANE& center) {
    LANE right = center;
    right.start_pt_ipm = 2 * center.start_pt_ipm - left.start_pt_ipm;
    right.end_pt_ipm = 2 * center.end_pt_ipm - left.end_pt_ipm;
    _ipm_transformer.xy_to_rtheta(right.start_pt_ipm, right.end_pt_ipm, right.radium_ipm,
                                  right.theta_ipm, cv::Point2f(0, _roi_out.height / 2));
    right.start_pt_img = _ipm_transformer.pt_transformation(right.start_pt_ipm,
                         _ipm_transformer.get_ipm_to_img());
    right.end_pt_img = _ipm_transformer.pt_transformation(right.end_pt_ipm,
                       _ipm_transformer.get_ipm_to_img());
    std::vector<cv::Point2f> pts;
    cv::Vec4f vec;
    pts.push_back(right.start_pt_ipm);
    pts.push_back(right.end_pt_ipm);
    cv::fitLine(pts, vec, CV_DIST_L2, 0, 0.01, 0.01);

    if (vec[1] > 0) {
        vec[1] = -vec[1];
        vec[0] = -vec[0];
    }

    // calculate the distance, the minus is important
    right.dist_to_camera = -(-vec[1] * (_camera_pos_ipm.x - vec[2]) + vec[0] *
                             (_camera_pos_ipm.y - vec[3]));
    return right;
}

LANE SLLaneTracking::left_from_center_and_right(const LANE& center, const LANE& right) {
    LANE left = center;
    left.start_pt_ipm = 2 * center.start_pt_ipm - right.start_pt_ipm;
    left.end_pt_ipm = 2 * center.end_pt_ipm - right.end_pt_ipm;
    _ipm_transformer.xy_to_rtheta(left.start_pt_ipm, left.end_pt_ipm, left.radium_ipm,
                                  left.theta_ipm, cv::Point2f(0, _roi_out.height / 2));
    left.start_pt_img = _ipm_transformer.pt_transformation(left.start_pt_ipm,
                        _ipm_transformer.get_ipm_to_img());
    left.end_pt_img = _ipm_transformer.pt_transformation(left.end_pt_ipm,
                      _ipm_transformer.get_ipm_to_img());
    std::vector<cv::Point2f> pts;
    cv::Vec4f vec;
    pts.push_back(left.start_pt_ipm);
    pts.push_back(left.end_pt_ipm);
    cv::fitLine(pts, vec, CV_DIST_L2, 0, 0.01, 0.01);

    if (vec[1] > 0) {
        vec[1] = -vec[1];
        vec[0] = -vec[0];
    }

    // calculate the distance, the minus is important
    left.dist_to_camera = -(-vec[1] * (_camera_pos_ipm.x - vec[2]) + vec[0] *
                            (_camera_pos_ipm.y - vec[3]));
    return left;
}

LANE SLLaneTracking::init_lane_from_lanes(const std::vector<LANE>& lanes) {
    LANE lane;
    int sz = lanes.size();
    lane.id = 0;
    lane.status = 1;
    std::vector<cv::Point2f> pts;
    cv::Vec4f vec;
    // 1. fit line for start_pt
    pts.clear();

    for (int j = 0; j < sz; ++j) {
        pts.push_back(cv::Point2f(j * 1000, lanes[j].start_pt_ipm.x));
    }

    cv::fitLine(pts, vec, CV_DIST_L2, 0, 0.01, 0.01);
    lane.start_pt_ipm = cv::Point2f(vec[1] / vec[0] * (sz * 1000 - vec[2]) + vec[3], 0);
    // 2. fit line for end_pt
    pts.clear();

    for (int j = 0; j < sz; ++j) {
        pts.push_back(cv::Point2f(j * 1000, lanes[j].end_pt_ipm.x));
    }

    cv::fitLine(pts, vec, CV_DIST_L2, 0, 0.01, 0.01);
    lane.end_pt_ipm = cv::Point2f(vec[1] / vec[0] * (sz * 1000 - vec[2]) + vec[3], _roi_out.height);
    // 3. other parameters
    lane.start_pt_img = _ipm_transformer.pt_transformation(lane.start_pt_ipm,
                        _ipm_transformer.get_ipm_to_img());
    lane.end_pt_img = _ipm_transformer.pt_transformation(lane.end_pt_ipm,
                      _ipm_transformer.get_ipm_to_img());
    _ipm_transformer.xy_to_rtheta(lane.start_pt_ipm, lane.end_pt_ipm, lane.radium_ipm,
                                  lane.theta_ipm, cv::Point2f(0, _roi_out.height / 2));
    pts.clear();
    pts.push_back(lane.start_pt_ipm);
    pts.push_back(lane.end_pt_ipm);
    cv::fitLine(pts, vec, CV_DIST_L2, 0, 0.01, 0.01);

    if (vec[1] > 0) {
        vec[1] = -vec[1];
        vec[0] = -vec[0];
    }

    // calculate the distance, the minus is important
    lane.dist_to_camera = -(-vec[1] * (_camera_pos_ipm.x - vec[2]) + vec[0] *
                            (_camera_pos_ipm.y - vec[3]));
    // 4. average the conf and len
    lane.conf = 0;
    lane.len_actual = 0;

    for (int i = 0; i < sz; ++i) {
        lane.len_actual += lanes[i].len_actual;
        lane.conf += lanes[i].conf;
    }

    lane.len_actual /= sz;
    lane.conf /= sz;
    return lane;
}

BORDER SLLaneTracking::estimate_border() {
    int sz = _borders.size();
    BORDER border;
    border = _borders[sz - 1];
    border.conf = 0;
    std::vector<LANE> lane_inlier_left;
    std::vector<LANE> lane_inlier_right;
    int max_inlier_left = 0;
    int max_inlier_right = 0;

    // 1. deal with the left border
    for (int i = 0; i < sz; ++i) {
        std::vector<LANE> lane_inlier_tmp;

        for (int j = 0; j < sz; ++j) {
            if (j == i) {
                continue;
            }

            if (fabs(_borders[i].left_border.radium_ipm - _borders[j].left_border.radium_ipm) <=
                    _minimal_dist) {
                lane_inlier_tmp.push_back(_borders[j].left_border);
            }
        }

        if (lane_inlier_tmp.size() > max_inlier_left) {
            lane_inlier_left = lane_inlier_tmp;
            max_inlier_left = lane_inlier_tmp.size();
        }
    }

    // 2. deal with the right border
    for (int i = 0; i < sz; ++i) {
        std::vector<LANE> lane_inlier_tmp;

        for (int j = 0; j < sz; ++j) {
            if (j == i) {
                continue;
            }

            if (fabs(_borders[i].right_border.radium_ipm - _borders[j].right_border.radium_ipm) <=
                    _minimal_dist) {
                lane_inlier_tmp.push_back(_borders[j].right_border);
            }
        }

        if (lane_inlier_tmp.size() > max_inlier_right) {
            lane_inlier_right = lane_inlier_tmp;
            max_inlier_right = lane_inlier_tmp.size();
        }
    }

    if (max_inlier_left >= _minimal_record * _mininal_good_ratio
            && max_inlier_right >= _minimal_record * _mininal_good_ratio) {
        border.left_border = init_lane_from_lanes(lane_inlier_left);
        border.right_border = init_lane_from_lanes(lane_inlier_right);
        border.conf = 1.0f;
    }

    return border;
}

//LANE SLLaneTracking::estimate_lane(const std::deque<LANE> &lanes)
//{
//  LANE lane;
//  lane.id = -1;
//
//  int sz = lanes.size();
//
//  if (sz >= 2)
//  {
//      lane.id = 0;
//      lane.start_pt_ipm = cv::Point2f(0, 0);
//      lane.end_pt_ipm = cv::Point2f(0, 0);
//      lane.start_pt_img = cv::Point2f(0, 0);
//      lane.end_pt_img = cv::Point2f(0, 0);
//      lane.dist_to_camera = 0;
//      lane.theta_ipm = 0;
//      lane.len_actual = 0;
//
//      for (int i = 0; i < sz; ++i)
//      {
//          lane.start_pt_ipm += lanes[i].start_pt_ipm;
//          lane.end_pt_ipm += lanes[i].end_pt_ipm;
//
//          lane.start_pt_img += lanes[i].start_pt_img;
//          lane.end_pt_img += lanes[i].end_pt_img;
//
//          lane.dist_to_camera += lanes[i].dist_to_camera;
//          lane.theta_ipm += lanes[i].theta_ipm;
//      }
//
//      lane.start_pt_ipm.x /= sz;
//      lane.start_pt_ipm.y /= sz;
//      lane.end_pt_ipm.x /= sz;
//      lane.end_pt_ipm.y /= sz;
//
//      lane.start_pt_img.x /= sz;
//      lane.start_pt_img.y /= sz;
//      lane.end_pt_img.x /= sz;
//      lane.end_pt_img.y /= sz;
//
//      lane.dist_to_camera /= sz;
//      lane.theta_ipm /= sz;
//  }
//
//
//  return lane;
//}

//int SLLaneTracking::do_tracking(std::vector<LANE> &lanes, const int &channel_num)
//{
//  std::vector<LANE> lanes_bak(lanes);
//
//  LANE border_left_tmp;
//  border_left_tmp.id = -1;
//  LANE border_right_tmp;
//  border_right_tmp.id = -1;
//
//  // 1. if the left and right lane is full, calculate the delta directly
//  if (lanes.size() == 2)
//  {
//      lanes_bak[0].dist_to_camera += shift_x;
//      lanes_bak[0].radium_ipm += shift_x;
//      lanes_bak[1].dist_to_camera += shift_x;
//      lanes_bak[1].radium_ipm += shift_x;
//
//      //_border_left.push_back(lanes_bak[0]);
//      //_border_right.push_back(lanes_bak[1]);
//
//      border_left_tmp = lanes_bak[0];
//      border_right_tmp = lanes_bak[1];
//  }
//  // 2. if not full, interpolation and calculate
//  else if (lanes.size() == 1)
//  {
//      lanes_bak[0].dist_to_camera += shift_x;
//      lanes_bak[0].radium_ipm += shift_x;
//
//      // 2.1. if the left is full
//      if (lanes[0].dist_to_camera <= 0)
//      {
//          LANE lane_right = estimate_lane(_border_right);
//
//          if (lane_right.id != -1)
//          {
//              lane_right.status = 1;
//              lanes.push_back(lane_right);
//          }
//
//
//          //_border_left.push_back(lanes_bak[0]);
//
//          border_left_tmp = lanes_bak[0];
//      }
//      else
//      {
//          LANE lane_left = estimate_lane(_border_left);
//
//          if (lane_left.id != -1)
//          {
//              lane_left.status = 1;
//              lanes.insert(lanes.begin(), lane_left);
//          }
//
//
//          //_border_right.push_back(lanes_bak[0]);
//          border_right_tmp = lanes_bak[0];
//      }
//  }
//  // 3. if all empty
//  else
//  {
//      LANE lane_left = estimate_lane(_border_left);
//      LANE lane_right = estimate_lane(_border_right);
//      lane_left.status = 1;
//      lane_right.status = 1;
//
//      lanes.push_back(lane_left);
//      lanes.push_back(lane_right);
//  }
//
//  // 4. calculate the delta_x, and use old records to refine
//
//  float old_delta_x = -lanes[0].dist_to_camera / (-lanes[0].dist_to_camera + lanes[1].dist_to_camera);
//  float new_delta_x = old_delta_x;
//
//  _delta_x_original.push_back(old_delta_x);
//
//  int record_num = 9;
//  int sz = _delta_x_original.size();
//  if (sz >= record_num)
//  {
//      int start_idx = std::max(0, sz - record_num);
//
//      // 5.1. scan all combo
//      int max_inlier = 0;
//      std::vector<float> delta_x_inlier;
//
//      for (int i = start_idx; i < sz; ++i)
//      {
//          // 5.1.1. select the maximal inlier
//          float delta_x_tmp = _delta_x_original[i];
//          std::vector<float> delta_x_inlier_tmp;
//          int count = 1;
//          for (int j = start_idx; j < sz; ++j)
//          {
//              if (j == i)
//              {
//                  continue;
//              }
//
//              float dist = abs(_delta_x_original[i] - _delta_x_original[j]);
//              float dist_up = abs(_delta_x_original[i] - 1 - _delta_x_original[j]);
//              float dist_down = abs(_delta_x_original[i] + 1 - _delta_x_original[j]);
//              float dist_min = std::min(dist, std::min(dist_up, dist_down));
//
//              if (dist_min < 0.1)
//              {
//                  if (abs(dist_min - dist) < 0.0001)
//                  {
//                      delta_x_tmp += _delta_x_original[j];
//                      delta_x_inlier_tmp.push_back(_delta_x_original[j]);
//                  }
//                  else if (abs(dist_min - dist_up) < 0.0001)
//                  {
//                      delta_x_tmp += 1 + _delta_x_original[j];
//                      delta_x_inlier_tmp.push_back(1 + _delta_x_original[j]);
//                  }
//                  else
//                  {
//                      delta_x_tmp += -1 + _delta_x_original[j];
//                      delta_x_inlier_tmp.push_back(-1 + _delta_x_original[j]);
//                  }
//                  ++count;
//              }
//
//          }
//
//          if (count > max_inlier)
//          {
//              max_inlier = count;
//              delta_x_inlier = delta_x_inlier_tmp;
//          }
//      }
//
//      if (max_inlier > record_num * 0.5)
//      {
//          std::vector<cv::Point2f> pts;
//          for (int j = 0; j < delta_x_inlier.size(); ++j)
//          {
//              pts.push_back(cv::Point2f(j, delta_x_inlier[j]));
//          }
//          cv::Vec4f vec;
//          cv::fitLine(pts, vec, CV_DIST_L2, 0, 0.01, 0.01);
//
//          new_delta_x = vec[1] / vec[0] * (delta_x_inlier.size() - vec[2]) + vec[3];
//
//          new_delta_x = new_delta_x - floor(new_delta_x);
//      }
//      else
//      {
//          new_delta_x = _delta_x_original[sz - 1];
//      }
//  }
//
//
//  // 5. if the refinement is near, use old; or use refinement
//
//  if (abs(old_delta_x - new_delta_x) < 0.1
//      || abs(old_delta_x - new_delta_x + 1) < 0.1
//      || abs(old_delta_x - new_delta_x - 1) < 0.1)
//  {
//      new_delta_x = old_delta_x;
//
//      if (border_left_tmp.id != -1)
//      {
//          _border_left.push_back(border_left_tmp);
//      }
//      if (border_right_tmp.id != -1)
//      {
//          _border_right.push_back(border_right_tmp);
//      }
//  }
//  else
//  {
//      lanes.clear();
//
//      LANE lane_left = estimate_lane(_border_left);
//      LANE lane_right = estimate_lane(_border_right);
//      lane_left.status = 1;
//      lane_right.status = 1;
//
//      lanes.push_back(lane_left);
//      lanes.push_back(lane_right);
//  }
//
//  _delta_x_filtered.push_back(new_delta_x);
//
//
//  // 6. update the delta_x, angle and channel
//
//  _ret.delta_x = new_delta_x;
//  _ret.angle = (lanes[0].theta_ipm + lanes[1].theta_ipm) / 2;
//  if (_ret_old.channel > -100)
//  {
//      if (abs(_ret.delta_x + 1 - _ret_old.delta_x) < 0.3)
//      {
//          _ret.channel = _ret_old.channel + 1;
//      }
//      else if (abs(_ret_old.delta_x + 1 - _ret.delta_x) < 0.3)
//      {
//          _ret.channel = _ret_old.channel - 1;
//      }
//      else
//      {
//          _ret.channel = _ret_old.channel;
//      }
//  }
//
//  // 7. a trick: the solid lane determine all
//  int left_sum = 0;
//  for (int i = 0; i < _border_left.size(); ++i)
//  {
//      left_sum += _border_left[i].len_actual > 180;
//  }
//  int right_sum = 0;
//  for (int i = 0; i < _border_right.size(); ++i)
//  {
//      right_sum += _border_right[i].len_actual > 180;
//  }
//  if (left_sum > 3 && lanes[0].len_actual > 180)
//  {
//      _ret.channel = 1;
//  }
//  else if (channel_num != -1 && right_sum > 3 && lanes[1].len_actual > 180)
//  {
//      _ret.channel = channel_num;
//  }
//
//  // 8. update the shift_x
//
//
//  _ret_old = _ret;
//  if (_border_left.size() > 7)
//  {
//      _border_left.pop_front();
//  }
//  if (_border_right.size() > 7)
//  {
//      _border_right.pop_front();
//  }
//  if (_delta_x_original.size() > 100)
//  {
//      _delta_x_original.pop_front();
//  }
//  if (_delta_x_filtered.size() > 100)
//  {
//      _delta_x_filtered.pop_front();
//  }
//
//  return 0;
//}

int SLLaneTracking::do_tracking(std::vector<LANE>& lanes, const int& channel_num) {
    /*
    #ifdef __GNUC__
    struct timespec time1 = { 0, 0 };
    struct timespec time2 = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &time1);
    #else
    LARGE_INTEGER t1, t2, tc;
    QueryPerformanceFrequency(&tc);
    QueryPerformanceCounter(&t1);
    #endif

    // 1. initial the data
    std::vector<LANE> lanes_bak;
    int sz_lanes = lanes.size();


    // 2. only the _borders is full, can estimate
    if (_borders.size() >= _minimal_record)
    {
        BORDER border = estimate_border();

        // 2.1. if the lanes are full
        std::vector<LANE> candidates;

        LANE left = left_from_center_and_right(border.left_border, border.right_border);
        LANE right = right_from_left_and_center(border.left_border, border.right_border);
        candidates.push_back(border.left_border);
        candidates.push_back(border.right_border);
        candidates.push_back(left);
        candidates.push_back(border.left_border);
        candidates.push_back(border.right_border);
        candidates.push_back(right);

        // 2.4. prepare the 3 candidates
        bool flag = false;
        int idx = -1;

        for (int i = 0; i < 3; ++i)
        {
            // 2.4.1. find the center lane
            if (candidates[i * 2].dist_to_camera <= 0
                && candidates[i * 2 + 1].dist_to_camera > 0)
            {
                idx = i;
            }

            lanes_bak.clear();
            lanes_bak.push_back(candidates[i * 2]);
            lanes_bak.push_back(candidates[i * 2 + 1]);

            // 2.4.2. if full, two confidence lanes can represent the estimations
            if (sz_lanes == 2)
            {
                if (idx != i)
                {
                    if (fabs(lanes_bak[0].radium_ipm - lanes[0].radium_ipm) <= _minimal_dist
                        && fabs(lanes_bak[1].radium_ipm - lanes[1].radium_ipm) <= _minimal_dist)
                    {
                        lanes_bak[0] = lanes[0];
                        lanes_bak[1] = lanes[1];
                        flag = true;
                    }
                }
                else
                {
                    if (fabs(lanes_bak[0].radium_ipm - lanes[0].radium_ipm) <= _minimal_dist)
                    {
                        lanes_bak[0] = lanes[0];
                        flag = true;
                    }
                    if (fabs(lanes_bak[1].radium_ipm - lanes[1].radium_ipm) <= _minimal_dist)
                    {
                        lanes_bak[1] = lanes[1];
                        flag = true;
                    }
                }

            }
            // 2.4.3. if empty, i must be idx
            else if (sz_lanes == 1 && idx == i)
            {
                if (lanes[0].dist_to_camera <= 0)
                {
                    if (fabs(lanes_bak[0].radium_ipm - lanes[0].radium_ipm) <= _minimal_dist)
                    {
                        lanes_bak[0] = lanes[0];
                        flag = true;
                    }
                }
                else
                {
                    if (fabs(lanes_bak[1].radium_ipm - lanes[0].radium_ipm) <= _minimal_dist)
                    {
                        lanes_bak[1] = lanes[0];
                        flag = true;
                    }
                }
            }

            if (flag == true)
            {
                break;
            }
        }

        if (flag == false)
        {
            if (idx != -1 && border.conf >= 0.5)
            {
                lanes_bak.clear();
                lanes_bak.push_back(candidates[idx * 2]);
                lanes_bak.push_back(candidates[idx * 2 + 1]);
            }
            else
            {
                lanes_bak.clear();
            }

        }
    }


    // 3. calculate the old delta_x
    _ret.delta_x = -1;
    if (sz_lanes == 2)
    {
        _ret.delta_x = -lanes[0].dist_to_camera / (-lanes[0].dist_to_camera + lanes[1].dist_to_camera);

        BORDER border_new;
        border_new.left_border = lanes[0];
        border_new.right_border = lanes[1];
        _borders.push_back(border_new);
    }

    if (lanes_bak.size() == 2)
    {
        lanes = lanes_bak;
    }
    else
    {
        _ret = _ret_old;
    }


    // 5. the new delta_x
    if (lanes.size() == 2)
    {
        _ret.delta_x = -lanes[0].dist_to_camera / (-lanes[0].dist_to_camera + lanes[1].dist_to_camera);
        _ret.angle = (lanes[0].theta_ipm + lanes[1].theta_ipm) / 2;

        if (_ret_old.delta_x > -100)
        {
            float min_dist = fabs(_ret.delta_x - _ret_old.delta_x);
            float min_dist_up = fabs(_ret.delta_x + 1 - _ret_old.delta_x);
            float min_dist_down = fabs(_ret_old.delta_x + 1 - _ret.delta_x);
            float min_dist_all = std::min(std::min(min_dist, min_dist_up), min_dist_down);

            if (fabs(min_dist_up - min_dist_all) < 0.0001)
            {
                _ret.channel = _ret_old.channel + 1;
            }
            else if (fabs(min_dist_down - min_dist_all) < 0.0001)
            {
                _ret.channel = _ret_old.channel - 1;
            }
            else
            {
                _ret.channel = _ret_old.channel;
            }
        }
        else
        {
            _ret.channel = _ret_old.channel;
        }

        // 7. a trick: the solid lane determine all
        int sz_borders = _borders.size();
        if (sz_borders >= _minimal_record)
        {
            int left_sum = 0;
            for (int i = 0; i < sz_borders; ++i)
            {
                left_sum += _borders[i].left_border.len_actual > 180;
            }
            int right_sum = 0;
            for (int i = 0; i < sz_borders; ++i)
            {
                right_sum += _borders[i].right_border.len_actual > 180;
            }
            if (left_sum > _minimal_record * _mininal_good_ratio && lanes[0].len_actual > 180)
            {
                _ret.channel = 1;
            }
            else if (channel_num != -1 && right_sum > _minimal_record * _mininal_good_ratio && lanes[1].len_actual > 180)
            {
                _ret.channel = channel_num;
            }
        }
    }


    // 8. update all parameters
    if (_borders.size() > _minimal_record)
    {
        _borders.pop_front();
    }
    _ret_old = _ret;


    #ifdef __GNUC__
    clock_gettime(CLOCK_REALTIME, &time2);
    std::cout << "[info] time for tracking: " << (time2.tv_sec - time1.tv_sec) * 1000 + (time2.tv_nsec - time1.tv_nsec) / 1000000 << "ms" << std::endl;
    clock_gettime(CLOCK_REALTIME, &time1);
    #else
    QueryPerformanceCounter(&t2);
    std::cout << "[info] time for tracking: " << (t2.QuadPart - t1.QuadPart) * 1000.0 / tc.QuadPart << std::endl;
    QueryPerformanceCounter(&t1);
    #endif
    */
    return 0;
}

#endif
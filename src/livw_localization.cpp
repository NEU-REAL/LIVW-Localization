/*
 *  Copyright (c) 2024, Northeastern University
 *  All rights reserved.
 *
 *  Modifier: Jibo Wang
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Northeastern University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  Copyright (c) , The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: hku-mars
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
// #include <common_lib.h>
#include "IMU_Processing.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livw_localization/States.h>
#include <geometry_msgs/Vector3.h>
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vikit/camera_loader.h>
#include"lidar_selection.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "wheel_encoder_processing.hpp"
#include <livw_localization/encoder.h>

#ifdef USE_ikdtree
    #ifdef USE_ikdforest
    #include <ikd-Forest/ikd_Forest.h>
    #else
    #include <ikd-Tree/ikd_Tree.h>
    #endif
#else
#include <pcl/kdtree/kdtree_flann.h>
#endif

#define INIT_TIME           (0.5)
#define MAXN                (360000)
#define PUBFRAME_PERIOD     (20)

float DET_RANGE = 300.0f;
#ifdef USE_ikdforest
    const int laserCloudWidth  = 200;
    const int laserCloudHeight = 200;
    const int laserCloudDepth  = 200;
    const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
#else
    const float MOV_THRESHOLD = 1.5f;
#endif

mutex mtx_buffer;
condition_variable sig_buffer;

// mutex mtx_buffer_pointcloud;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic, img_topic, config_file;;
M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);
Vector3d Lidar_offset_to_IMU;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0,\
    effct_feat_num = 0, time_log_counter = 0, publish_count = 0;
int MIN_IMG_COUNT = 0;

double res_mean_last = 0.05;
double gyr_cov_scale = 0, acc_cov_scale = 0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
double last_timestamp_lidar_end_time = 0;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
double first_img_time=-1.0;
double kdtree_incremental_time = 0, kdtree_search_time = 0, kdtree_delete_time = 0.0;
int kdtree_search_counter = 0, kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;;
double copy_time = 0, readd_time = 0, fov_check_time = 0, readd_box_time = 0, delete_box_time = 0;
double T1[MAXN], T2[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN];

double match_time = 0, solve_time = 0, solve_const_H_time = 0;

bool lidar_pushed, flg_reset, flg_exit = false;
bool ncc_en;
int dense_map_en = 1;
int img_en = 1;
int lidar_en = 1;
int debug = 0;
bool fast_lio_is_ready = false;
int grid_size, patch_size;
double voxel_size_;
double outlier_threshold, ncc_thre;

vector<BoxPointType> cub_needrm;
vector<BoxPointType> cub_needad;
// deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
deque<PointCloudXYZI::Ptr>  lidar_buffer;
deque<double>          time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<cv::Mat> img_buffer;
deque<double>          img_time_buffer;
vector<bool> point_selected_surf; 
vector<vector<int>> pointSearchInd_surf; 
vector<PointVector> Nearest_Points; 
vector<double> res_last;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
vector<double> cameraextrinT(3, 0.0);
vector<double> cameraextrinR(9, 0.0);
double total_residual;
double LASER_POINT_COV, IMG_POINT_COV, cam_fx, cam_fy, cam_cx, cam_cy; 
int cam_width_, cam_height_;
bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr cube_points_add(new PointCloudXYZI());
PointCloudXYZI::Ptr map_cur_frame_point(new PointCloudXYZI());
PointCloudXYZI::Ptr sub_map_cur_frame_point(new PointCloudXYZI());

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI());
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

// globalmap
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> globalmap_S_SurfBuf;
bool received_map_for_first_time_ = false;

// wheel_encoder
string wheel_encoder_topic;

double last_timestamp_wheel_encoder = -1.0;
int    wheel_encoder_frequency;
double wheel_encoder_pulse_resolution;
double WHEEL_ENCODER_P_COV_ = 1.0, WHEEL_ENCODER_V_COV_ = 1.0;
std::deque<livw_localization::encoder::ConstPtr> wheel_encoder_buffer;
V3D Wheel_encoder_T_wrt_IMU(Zero3d);
M3D Wheel_encoder_R_wrt_IMU(Eye3d);

#ifdef USE_ikdtree
    #ifdef USE_ikdforest
    KD_FOREST ikdforest;
    #else
    KD_TREE ikdtree;
    #endif
#else
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
#endif

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
Eigen::Matrix3d Rcl;
Eigen::Vector3d Pcl;

//estimator inputs and output;
LidarMeasureGroup LidarMeasures;
// SparseMap sparse_map;
#ifdef USE_IKFOM
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;
#else
StatesGroup  state;
StatesGroup  prev_state;
#endif

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}


#ifdef USE_IKFOM
//project the lidar scan to world frame
void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
#endif

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    #ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    #else
    V3D p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    #endif

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    #ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    #else
    V3D p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    #endif
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    #ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    #else
    V3D p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    #endif
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity*10000;
}

#ifndef USE_ikdforest
int points_cache_size = 0;
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}
#endif

BoxPointType get_cube_point(float center_x, float center_y, float center_z)
{
    BoxPointType cube_points;
    V3F center_p(center_x, center_y, center_z);
    // cout<<"center_p: "<<center_p.transpose()<<endl;

    for(int i = 0; i < 3; i++)
    {
        cube_points.vertex_max[i] = center_p[i] + 0.5 * cube_len;
        cube_points.vertex_min[i] = center_p[i] - 0.5 * cube_len;
    }

    return cube_points;
}

BoxPointType get_cube_point(float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
    BoxPointType cube_points;
    cube_points.vertex_max[0] = xmax;
    cube_points.vertex_max[1] = ymax;
    cube_points.vertex_max[2] = zmax;
    cube_points.vertex_min[0] = xmin;
    cube_points.vertex_min[1] = ymin;
    cube_points.vertex_min[2] = zmin;
    return cube_points;
}

#ifndef USE_ikdforest
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    #ifdef USE_IKFOM
    //state_ikfom fov_state = kf.get_x();
    //V3D pos_LiD = fov_state.pos + fov_state.rot * fov_state.offset_T_L_I;
    V3D pos_LiD = pos_lid;
    #else
    V3D pos_LiD = state.pos_end;
    #endif
    if (!Localmap_Initialized){
        //if (cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE) throw std::invalid_argument("[Error]: Local Map Size is too small! Please change parameter \"cube_side_length\" to larger than %d in the launch file.\n");
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // printf("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", LocalMap_Points.vertex_min[0],LocalMap_Points.vertex_max[0],LocalMap_Points.vertex_min[1],LocalMap_Points.vertex_max[1],LocalMap_Points.vertex_min[2],LocalMap_Points.vertex_max[2]);
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);                     
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
    // printf("Delete time: %0.6f, delete size: %d\n",kdtree_delete_time,kdtree_delete_counter);
    // printf("Delete Box: %d\n",int(cub_needrm.size()));
}
#endif


void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();

    pcl::PointCloud<velodyne_ros::Point>::Ptr pointcloud_orig(new pcl::PointCloud<velodyne_ros::Point>);
    pcl::fromROSMsg(*msg, *pointcloud_orig);

    double scan_time = -1 * pointcloud_orig->points[0].time;
    double start_time_lidar = msg->header.stamp.toSec() - scan_time;

    if ( start_time_lidar < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    printf("[ INFO ]: get point cloud at time: %.6f and size: %d.\n", msg->header.stamp.toSec(), int(ptr->points.size()));
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(start_time_lidar);
    last_timestamp_lidar = start_time_lidar;
    last_timestamp_lidar_end_time = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    //cout<<"msg_in:"<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    
    double timestamp = msg->header.stamp.toSec();
    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv::Mat img;
  img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  
  if(cam_width_!=img.cols || cam_height_!=img.rows)
  {
    cv::resize(img,img,cv::Size(cam_width_, cam_height_),0,0,CV_INTER_LINEAR);
  }
    
  return img;
}

void img_cbk(const sensor_msgs::ImageConstPtr& msg)
{
    if (!img_en) 
    {
        return;
    }
    printf("[ INFO ]: get img at time: %.6f.\n", msg->header.stamp.toSec());
    if (msg->header.stamp.toSec() < last_timestamp_img)
    {
        ROS_ERROR("img loop back, clear buffer");
        img_buffer.clear();
        img_time_buffer.clear();
    }
    mtx_buffer.lock();
    // cout<<"Lidar_buff.size()"<<lidar_buffer.size()<<endl;
    // cout<<"Imu_buffer.size()"<<imu_buffer.size()<<endl;
    img_buffer.push_back(getImageFromMsg(msg));
    img_time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_img = msg->header.stamp.toSec();
    // cv::imshow("img", img);
    // cv::waitKey(1);
    // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void wheelEncoderHandler(const livw_localization::encoderConstPtr &wheel_encoder_msg_in)
{
    double timestamp = wheel_encoder_msg_in->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_wheel_encoder)
    {
        ROS_WARN("wheel_encoder loop back, clear buffer");
        wheel_encoder_buffer.clear();
    }

    last_timestamp_wheel_encoder = timestamp;

    wheel_encoder_buffer.push_back(wheel_encoder_msg_in);
    mtx_buffer.unlock();
    sig_buffer.notify_all();

}

void initialposeHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    if(msg->header.frame_id == "")
    {
        // This should be removed at some point
        ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
    }
    
    ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f", ros::Time::now().toSec(),
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    StatesGroup initialpose_state = state;
    Eigen::Quaterniond initialpose_quaternion;
    initialpose_quaternion.x() = msg->pose.pose.orientation.x;
    initialpose_quaternion.y() = msg->pose.pose.orientation.y;
    initialpose_quaternion.z() = msg->pose.pose.orientation.z;
    initialpose_quaternion.w() = msg->pose.pose.orientation.w;
    initialpose_quaternion.normalize(); // 四元数归一化
    M3D initialpose_Rotation = initialpose_quaternion.toRotationMatrix();  //四元数转为旋转矩阵
    initialpose_state.rot_end = initialpose_Rotation;
    initialpose_state.pos_end = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    initialpose_state.vel_end = Zero3d;
    initialpose_state.cov     = Matrix<double,DIM_STATE,DIM_STATE>::Identity() * INIT_COV;

    state = initialpose_state;
    prev_state = state;
    geoQuat.x = initialpose_quaternion.x();
    geoQuat.y = initialpose_quaternion.y();
    geoQuat.z = initialpose_quaternion.z();
    geoQuat.w = initialpose_quaternion.w();
}


void globalmap_S_SurfHandler(const sensor_msgs::PointCloud2ConstPtr &surfCloudMsg)
{
    mutex_lock.lock();
    globalmap_S_SurfBuf.push(surfCloudMsg);
    mutex_lock.unlock();
}

bool sync_packages(LidarMeasureGroup &meas)
{
    bool data_is_empty_flag = false;
    if(lidar_en)
    {
        if(lidar_buffer.empty())
        {
            // std::cout << "lidar_buffer is empty." << std::endl;
            data_is_empty_flag = true;
        }
    }
    if(img_en)
    {
        if(img_buffer.empty())
        {
            // std::cout << "img_buffer is empty." << std::endl;
            data_is_empty_flag = true;
        }
    }

        if(wheel_encoder_buffer.empty())
        {
            // std::cout << "wheel_encoder_buffer is empty."<< std::endl;
            data_is_empty_flag = true;
        }

    if(data_is_empty_flag)
    {
        return false;
    }

    if (meas.is_lidar_end) // If meas.is_lidar_end==true, means it just after scan end, clear all buffer in meas.
    {
        meas.measures.clear();
        meas.is_lidar_end = false;
    }
    
    if (!lidar_pushed) { // If not in lidar scan, need to generate new meas
        if (lidar_buffer.empty()) {
            // ROS_ERROR("out sync");
            return false;
        }
        meas.lidar = lidar_buffer.front(); // push the firsrt lidar topic
        if(meas.lidar->points.size() <= 1)
        {
            mtx_buffer.lock();
            if (img_buffer.size()>0) // temp method, ignore img topic when no lidar points, keep sync
            {
                lidar_buffer.pop_front();
                img_buffer.pop_front();
            }
            mtx_buffer.unlock();
            sig_buffer.notify_all();
            // ROS_ERROR("out sync");
            return false;
        }
        sort(meas.lidar->points.begin(), meas.lidar->points.end(), time_list); // sort by sample timestamp
        meas.lidar_beg_time = time_buffer.front(); // generate lidar_beg_time
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
        lidar_pushed = true; // flag
    }

    if (img_buffer.empty()) { // no img topic, means only has lidar topic
        if (last_timestamp_imu < lidar_end_time+0.02) { // imu message needs to be larger than lidar_end_time, keep complete propagate.
            // ROS_ERROR("out sync");
            return false;
        }
        struct MeasureGroup m; //standard method to keep imu message.
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        mtx_buffer.lock();
        while ((!imu_buffer.empty() && (imu_time<lidar_end_time))) {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > lidar_end_time) break;
            m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        lidar_pushed = false; // sync one whole lidar scan.
        meas.is_lidar_end = true; // process lidar topic, so timestamp should be lidar scan end.
        meas.measures.push_back(m);


            do
            {
                if (last_timestamp_wheel_encoder < lidar_end_time+0.02)
                {
                    // ROS_ERROR("out sync");
                    break;
                }
                double wheel_encoder_time = wheel_encoder_buffer.front()->header.stamp.toSec();
                meas.wheel_encoder.clear();
                mtx_buffer.lock();
                while ((!wheel_encoder_buffer.empty()) && (wheel_encoder_time < lidar_end_time))
                {
                    wheel_encoder_time = wheel_encoder_buffer.front()->header.stamp.toSec();
                    if(wheel_encoder_time > lidar_end_time) break;
                    meas.wheel_encoder.push_back(wheel_encoder_buffer.front());
                    wheel_encoder_buffer.pop_front();
                }
                mtx_buffer.unlock();
            }while(0);

        // ROS_ERROR("out sync");
        return true;
    }
    struct MeasureGroup m;

    if ((img_time_buffer.front()>lidar_end_time) )
    { // has img topic, but img topic timestamp larger than lidar end time, process lidar topic.
        if (last_timestamp_imu < lidar_end_time+0.02) 
        {
            // ROS_ERROR("out sync");
            return false;
        }
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        mtx_buffer.lock();
        while ((!imu_buffer.empty() && (imu_time<lidar_end_time))) 
        {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > lidar_end_time) break;
            m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        lidar_pushed = false;
        meas.is_lidar_end = true;
        meas.measures.push_back(m);
    }
    else 
    {
        double img_start_time = img_time_buffer.front(); // process img topic, record timestamp
        if (last_timestamp_imu < img_start_time) 
        {
            // ROS_ERROR("out sync");
            return false;
        }
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        m.img_offset_time = img_start_time - meas.lidar_beg_time; // record img offset time, it shoule be the Kalman update timestamp.
        m.img = img_buffer.front();
        mtx_buffer.lock();
        while ((!imu_buffer.empty() && (imu_time<img_start_time))) 
        {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > img_start_time) break;
            m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        meas.is_lidar_end = false; // has img topic in lidar scan, so flag "is_lidar_end=false" 
        meas.measures.push_back(m);

            do
            {
                if (last_timestamp_wheel_encoder < img_start_time)
                {
                    // ROS_ERROR("out sync");
                    break;
                }
                double wheel_encoder_time = wheel_encoder_buffer.front()->header.stamp.toSec();
                meas.wheel_encoder.clear();
                mtx_buffer.lock();
                while ((!wheel_encoder_buffer.empty()) && (wheel_encoder_time < img_start_time))
                {
                    wheel_encoder_time = wheel_encoder_buffer.front()->header.stamp.toSec();
                    if(wheel_encoder_time > img_start_time) break;
                    meas.wheel_encoder.push_back(wheel_encoder_buffer.front());
                    wheel_encoder_buffer.pop_front();
                }
                mtx_buffer.unlock();
            }while(0);
        }
    // ROS_ERROR("out sync");
    return true;
}


void update_with_wheel_encoder(double wheel_encoder_p_cov, double wheel_encoder_v_cov,const Eigen::Matrix3d &wheel_encoder_R_wrt_IMU, Eigen::Vector3d wheel_encoder_delta_distance_In_wheel, Eigen::Vector3d wheel_encoder_latest_vel_In_wheel)
{
    // std::cout <<"wheel_encoder_delta_distance_In_wheel = " << std::endl << wheel_encoder_delta_distance_In_wheel << std::endl;
    // std::cout <<"wheel_encoder_latest_vel_In_wheel = " << std::endl << wheel_encoder_latest_vel_In_wheel << std::endl;

    Eigen::Matrix<double, 6, 1> wheel_encoder_R_tmp;

    for (int i = 0; i < 3; ++i) 
    {
        wheel_encoder_R_tmp(i,0) = wheel_encoder_p_cov;
    }

    for (int i = 3; i < 6; ++i) 
    {
        wheel_encoder_R_tmp(i,0) = wheel_encoder_v_cov;
    }

    // std::cout <<"wheel_encoder_R_tmp = " << std::endl << wheel_encoder_R_tmp << std::endl;

    Eigen::Matrix<double, 6, 6> wheel_encoder_R = wheel_encoder_R_tmp.asDiagonal();

    Eigen::Matrix<double, 6, DIM_STATE> measurement_H = Eigen::Matrix<double, 6, DIM_STATE>::Zero();
    measurement_H.block(0, 3,  3, 3) = Eigen::Matrix3d::Identity();
    measurement_H.block(3, 6,  3, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d rotation_wrt_Global = state.rot_end;

    // 计算 z
    Eigen::Vector3d precdict_delta_distance_wrt_Global = state.pos_end - prev_state.pos_end;
    Eigen::Vector3d precdict_delta_distance_wrt_WHEEL = wheel_encoder_R_wrt_IMU.inverse() * rotation_wrt_Global.inverse() * precdict_delta_distance_wrt_Global;

    Eigen::Vector3d precdict_velocity_wrt_Global = state.vel_end;
    Eigen::Vector3d precdict_velocity_wrt_WHEEL = wheel_encoder_R_wrt_IMU.inverse() * rotation_wrt_Global.inverse() * precdict_velocity_wrt_Global;

    // 计算 y 
    Eigen::Matrix<double, 6, 1> measurement_error_y;
    measurement_error_y.block(0, 0,   3, 1) = rotation_wrt_Global * wheel_encoder_R_wrt_IMU * (wheel_encoder_delta_distance_In_wheel - precdict_delta_distance_wrt_WHEEL);
    measurement_error_y.block(3, 0,   3, 1) = rotation_wrt_Global * wheel_encoder_R_wrt_IMU * (wheel_encoder_latest_vel_In_wheel - precdict_velocity_wrt_WHEEL);
    //==========  ==========//

    // 计算 K
    Eigen::Matrix<double, DIM_STATE, 6> K_ = state.cov * measurement_H.transpose() * (measurement_H * state.cov * measurement_H.transpose() + wheel_encoder_R).inverse();
    
    Eigen::Matrix<double, DIM_STATE, 1> delat_state = K_ * measurement_error_y;

    Matrix<double, DIM_STATE, DIM_STATE> last_cov_ = state.cov;
    state = state + delat_state;   // 更新 状态
    // 更新 P_ = (I - KH) * P_
    state.cov = (Matrix<double,DIM_STATE,DIM_STATE>::Identity() - K_ * measurement_H) * last_cov_;

}


void gloablmap_ikdtree_rebuild(const PointCloudXYZI::Ptr &globalmapSurfMap_)
{
    double start_time = omp_get_wtime();
    int globalmap_surfmap_size = globalmapSurfMap_->points.size();
    ikdtree.Build(globalmapSurfMap_->points);
    double kdtree_rebuild_time = omp_get_wtime() - start_time;
}

// PointCloudXYZRGB::Ptr pcl_wait_pub_RGB(new PointCloudXYZRGB(500000, 1));
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());

PointCloudXYZI::Ptr pcl_nearest_searched_in_map(new PointCloudXYZI());

void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    // PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    // int size = laserCloudFullRes->points.size();
    // if(size==0) return;
    // PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

    // for (int i = 0; i < size; i++)
    // {
    //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
    //                         &laserCloudWorld->points[i]);
    // }
    uint size = pcl_wait_pub->points.size();
    // PointCloudXYZ::Ptr laserCloudWorld(new PointCloudXYZ(size, 1));
    // else
    // {
    //*pcl_wait_pub = *laserCloudWorld;
    // }
    // mtx_buffer_pointcloud.lock();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time().fromSec(last_timestamp_lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar) // ros::Time::now()
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}



void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(last_timestamp_lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar) // ros::Time::now()
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}


template<typename T>
void set_posestamp(T & out)
{
    #ifdef USE_IKFOM
    //state_ikfom stamp_state = kf.get_x();
    out.position.x = state_point.pos(0);
    out.position.y = state_point.pos(1);
    out.position.z = state_point.pos(2);
    #else
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
    #endif
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped, double &timestamp)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(timestamp);//.ros::Time()fromSec(last_timestamp_lidar);    // ros::Time::now()
    set_posestamp(odomAftMapped.pose.pose);

    Matrix<double, DIM_STATE, DIM_STATE> P = state.cov;
    for (int i = 0; i < 6; i ++)
    {
        odomAftMapped.pose.covariance[i*6 + 0] = P(3, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(4, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(5, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(0, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(1, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(2, 2);
    }
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );

}



void publish_mavros(const ros::Publisher & mavros_pose_publisher)
{
    msg_body_pose.header.stamp = ros::Time().fromSec(last_timestamp_lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar) // ros::Time::now()
    msg_body_pose.header.frame_id = "camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(last_timestamp_lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar) // ros::Time::now()
    msg_body_pose.header.frame_id = "camera_init";
    path.header.stamp = ros::Time().fromSec(last_timestamp_lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar) // ros::Time::now()
    path.header.frame_id ="camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}

#ifdef USE_IKFOM
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 
        //double search_start = omp_get_wtime();
        /* transform to world frame */
        //pointBodyToWorld_ikfom(&point_body, &point_world, s);
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
    #ifdef USE_ikdtree
        auto &points_near = Nearest_Points[i];
    #else
        auto &points_near = pointSearchInd_surf[i];
    #endif
        
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
        #ifdef USE_ikdtree
            #ifdef USE_ikdforest
                uint8_t search_flag = 0;                        
                search_flag = ikdforest.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5);                            
            #else
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            #endif
        #else
            kdtreeSurfFromMap->nearestKSearch(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        #endif

            point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;

        #ifdef USE_ikdforest
            point_selected_surf[i] = point_selected_surf[i] && (search_flag == 0);
        #endif
        }

        //kdtree_search_time += omp_get_wtime() - search_start;

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    // cout<<"pca time test: "<<pca_time1<<" "<<pca_time2<<endl;
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i] && (res_last[i] <= 2.0))
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    //MatrixXd H(effct_feat_num, 23);
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num); // = VectorXd::Zero(effct_feat_num);
    //VectorXd meas_vec(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be +s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C); // s.rot.conjugate() * norm_vec);
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
        //H.row(i) = Eigen::Matrix<double, 1, 23>::Zero();
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        //ekfom_data.h_x.block<1, 3>(i, 6) << VEC_FROM_ARRAY(A);
        //ekfom_data.h_x.block<1, 6>(i, 17) << VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        /*** Measuremnt: distance to the closest surface/corner ***/
        //meas_vec(i) = - norm_p.intensity;
        ekfom_data.h(i) = -norm_p.intensity;
    }
    //ekfom_data.h_x =H;
    solve_time += omp_get_wtime() - solve_start_;
    //return meas_vec;
}
#endif         

void readParameters(ros::NodeHandle &nh)
{
    nh.param<int>("dense_map_enable",dense_map_en,1);
    nh.param<int>("img_enable",img_en,1);
    nh.param<int>("lidar_enable",lidar_en,1);
    nh.param<int>("debug", debug, 0);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<bool>("ncc_en",ncc_en,false);
    nh.param<int>("min_img_count",MIN_IMG_COUNT,1000);    
    nh.param<int>("cam_width",cam_width_,640);
    nh.param<int>("cam_height",cam_height_,480);  
    nh.param<double>("cam_fx",cam_fx,453.483063);
    nh.param<double>("cam_fy",cam_fy,453.254913);
    nh.param<double>("cam_cx",cam_cx,318.908851);
    nh.param<double>("cam_cy",cam_cy,234.238189);
    nh.param<double>("localization/laser_point_cov",LASER_POINT_COV,0.001);
    nh.param<double>("camera/img_point_cov",IMG_POINT_COV,10);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/imu");
    nh.param<string>("camera/img_topic", img_topic,"/usb_cam/image_raw");
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<double>("localization/fov_degree",fov_deg,180);
    nh.param<double>("localization/gyr_cov_scale",gyr_cov_scale,1.0);
    nh.param<double>("localization/acc_cov_scale",acc_cov_scale,1.0);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, VELO16);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, 0);
    nh.param<vector<double>>("localization/extrinsic_T_Lidar_wrt_IMU", extrinT, vector<double>());
    nh.param<vector<double>>("localization/extrinsic_R_Lidar_wrt_IMU", extrinR, vector<double>());
    nh.param<vector<double>>("camera/extrinsic_T_Lidar_wrt_Cam", cameraextrinT, vector<double>());
    nh.param<vector<double>>("camera/extrinsic_R_Lidar_wrt_Cam", cameraextrinR, vector<double>());

    nh.param<int>("grid_size", grid_size, 40);
    nh.param<int>("patch_size", patch_size, 4);
    nh.param<double>("voxel_size", voxel_size_, 1.0);
    nh.param<double>("outlier_threshold",outlier_threshold,100);
    nh.param<double>("ncc_thre", ncc_thre, 100);

    nh.param<string>("wheel_encoder/wheel_encoder_topic", wheel_encoder_topic,"/wheel_encoder");
    nh.param<int>("wheel_encoder/frequency",wheel_encoder_frequency, 1);
    nh.param<double>("wheel_encoder/pulse_resolution",wheel_encoder_pulse_resolution, 0.0);
    std::vector<double> extrinT_Wheel_wrt_IMU(3, 0.0);
    std::vector<double> extrinR_Wheel_wrt_IMU(9, 0.0);
    nh.param<vector<double>>("wheel_encoder/extrinsic_R_Wheel_wrt_IMU", extrinR_Wheel_wrt_IMU, vector<double>());
    nh.param<vector<double>>("wheel_encoder/extrinsic_T_Wheel_wrt_IMU", extrinT_Wheel_wrt_IMU, vector<double>());
    nh.param<double>("wheel_encoder/wheel_encoder_p_cov", WHEEL_ENCODER_P_COV_, 1.0);
    nh.param<double>("wheel_encoder/wheel_encoder_v_cov", WHEEL_ENCODER_V_COV_, 1.0);
    Wheel_encoder_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR_Wheel_wrt_IMU);
    Wheel_encoder_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT_Wheel_wrt_IMU);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livw_localization");
    ros::NodeHandle nh;

    readParameters(nh);
    cout<<"debug:"<<debug<<" MIN_IMG_COUNT: "<<MIN_IMG_COUNT<<endl;
    pcl_wait_pub->clear();
    // pcl_visual_wait_pub->clear();
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_img = nh.subscribe(img_topic, 200000, img_cbk);

    ros::Publisher  img_pub = nh.advertise<sensor_msgs::Image>("/rgb_img", 1000);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/odometry", 10);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 10);

    ros::Subscriber subGlobalmap_S_SurfCloud = nh.subscribe<sensor_msgs::PointCloud2>("/globalmap_S_surf", 100, globalmap_S_SurfHandler);
    ros::Subscriber subInitialpose = nh.subscribe< geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, initialposeHandler);
    ros::Subscriber subWheelEncoder = nh.subscribe(wheel_encoder_topic, 2000, wheelEncoderHandler);

#ifdef DEPLOY
    ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
#endif
    
    path.header.stamp    = ros::Time().fromSec(last_timestamp_lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar) // ros::Time::now()
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    #ifndef USE_IKFOM
    VD(DIM_STATE) solution;
    MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
    V3D rot_add, t_add;
    StatesGroup state_propagat;
    PointType pointOri, pointSel, coeff;
    #endif
    //PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    #ifdef USE_ikdforest
        ikdforest.Set_balance_criterion_param(0.6);
        ikdforest.Set_delete_criterion_param(0.5);
        ikdforest.Set_environment(laserCloudDepth,laserCloudWidth,laserCloudHeight,cube_len);
        ikdforest.Set_downsample_param(filter_size_map_min);    
    #endif

    shared_ptr<ImuProcess> p_imu(new ImuProcess());

    V3D extT;
    M3D extR;
    extT<<VEC_FROM_ARRAY(extrinT);
    extR<<MAT_FROM_ARRAY(extrinR);
    Lidar_offset_to_IMU = extT;
    lidar_selection::LidarSelectorPtr lidar_selector(new lidar_selection::LidarSelector(grid_size, new SparseMap));
    if(!vk::camera_loader::loadFromRosNs("", lidar_selector->cam))
        throw std::runtime_error("Camera model not correctly specified.");
    // std::cout << "lidar_selector->cam->width()" << lidar_selector->cam->width() << std::endl;
    lidar_selector->MIN_IMG_COUNT = MIN_IMG_COUNT;
    lidar_selector->debug = debug;
    lidar_selector->patch_size = patch_size;
    lidar_selector->voxel_size = voxel_size_;
    lidar_selector->outlier_threshold = outlier_threshold;
    lidar_selector->ncc_thre = ncc_thre;
    lidar_selector->sparse_map->set_camera2lidar(cameraextrinR, cameraextrinT);
    lidar_selector->set_extrinsic(extT, extR);
    lidar_selector->state = &state;
    lidar_selector->state_propagat = &state_propagat;
    lidar_selector->NUM_MAX_ITERATIONS = NUM_MAX_ITERATIONS;
    lidar_selector->MIN_IMG_COUNT = MIN_IMG_COUNT;
    lidar_selector->img_point_cov = IMG_POINT_COV;
    lidar_selector->fx = cam_fx;
    lidar_selector->fy = cam_fy;
    lidar_selector->cx = cam_cx;
    lidar_selector->cy = cam_cy;
    lidar_selector->ncc_en = ncc_en;
    lidar_selector->init();
    
    p_imu->set_extrinsic(extT, extR);
    p_imu->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
    p_imu->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
    p_imu->set_gyr_bias_cov(V3D(0.00001, 0.00001, 0.00001));
    p_imu->set_acc_bias_cov(V3D(0.00001, 0.00001, 0.00001));

    std::shared_ptr<WheelEncoderProcess> p_wheel_encoder(new WheelEncoderProcess());
    p_wheel_encoder->set_extrinsic(Wheel_encoder_R_wrt_IMU, Wheel_encoder_T_wrt_IMU);
    p_wheel_encoder->set_wheel_encoder_frequency(wheel_encoder_frequency);
    p_wheel_encoder->set_wheel_encoder_pulse_resolution(wheel_encoder_pulse_resolution);


    #ifndef USE_IKFOM
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
    #endif

    #ifdef USE_IKFOM
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    #endif


    #ifdef USE_ikdforest
        ikdforest.Set_balance_criterion_param(0.6);
        ikdforest.Set_delete_criterion_param(0.5);
    #endif
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    int count = 0;
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();

        // initialization
        if(received_map_for_first_time_ == false)
        {
            count++;
            if(count > 20000000)
            {
                count = 0;
                ROS_WARN("Waiting for globalmap initialization....");
            }
            

            if(!globalmap_S_SurfBuf.empty())
            {
                received_map_for_first_time_ = true;

                PointCloudXYZI::Ptr globalmap_surfmap_(new PointCloudXYZI());
                globalmap_surfmap_->clear();
                pcl::fromROSMsg(*globalmap_S_SurfBuf.front(), *globalmap_surfmap_);
                globalmap_S_SurfBuf.pop();

                gloablmap_ikdtree_rebuild(globalmap_surfmap_);                
                pcl::copyPointCloud(*globalmap_surfmap_, *(lidar_selector->visualmap_points_from_lidarmap));
            }
            continue;
        }


        if(!sync_packages(LidarMeasures))
        {
            status = ros::ok();
            cv::waitKey(1);
            rate.sleep();
            continue;
        }

        /*** Packaged got ***/
        if (flg_reset)
        {
            ROS_WARN("reset when rosbag play back");
            p_imu->Reset();
            flg_reset = false;
            continue;
        }

        // double t0,t1,t2,t3,t4,t5,match_start, match_time, solve_start, solve_time, svd_time;
        double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

        match_time = kdtree_search_time = kdtree_search_counter = solve_time = solve_const_H_time = svd_time   = 0;
        t0 = omp_get_wtime();
        #ifdef USE_IKFOM
        p_imu->Process(LidarMeasures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        #else
        p_imu->Process2(LidarMeasures, state, feats_undistort); 
        state_propagat = state;
        #endif

        if (lidar_selector->debug)
        {
            LidarMeasures.debug_show();
        }

        if (feats_undistort->empty() || (feats_undistort == nullptr))
        {
            // cout<<" No point!!!"<<endl;
            if (!fast_lio_is_ready)
            {
                first_lidar_time = LidarMeasures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                LidarMeasures.measures.clear();
                cout<<"LIO is not ready"<<endl;
                continue;
            }
        }
        else
        {
            int size = feats_undistort->points.size();
        }
        fast_lio_is_ready = true;
        flg_EKF_inited = (LidarMeasures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                        false : true;


        if (! LidarMeasures.is_lidar_end) 
        {
            double vio_start_time = omp_get_wtime();
            cout<<"[ VIO ]: Raw feature num: "<<pcl_wait_pub->points.size() << "." << endl;
            
            if (first_lidar_time<10)
            {
                continue;
            }
            // cout<<"cur state:"<<state.rot_end<<endl;
            if (img_en) {
                euler_cur = RotMtoEuler(state.rot_end);
                

                // lidar_selector->detect(LidarMeasures.measures.back().img, pcl_wait_pub);
                lidar_selector->detect(LidarMeasures.measures.back().img, pcl_wait_pub, pcl_nearest_searched_in_map);
                // int size = lidar_selector->map_cur_frame_.size();
                int size_sub = lidar_selector->sub_map_cur_frame_.size();
                
                // map_cur_frame_point->clear();
                sub_map_cur_frame_point->clear();
                // for(int i=0; i<size; i++)
                // {
                //     PointType temp_map;
                //     temp_map.x = lidar_selector->map_cur_frame_[i]->pos_[0];
                //     temp_map.y = lidar_selector->map_cur_frame_[i]->pos_[1];
                //     temp_map.z = lidar_selector->map_cur_frame_[i]->pos_[2];
                //     temp_map.intensity = 0.;
                //     map_cur_frame_point->push_back(temp_map);
                // }
                for(int i=0; i<size_sub; i++)
                {
                    PointType temp_map;
                    temp_map.x = lidar_selector->sub_map_cur_frame_[i]->pos_[0];
                    temp_map.y = lidar_selector->sub_map_cur_frame_[i]->pos_[1];
                    temp_map.z = lidar_selector->sub_map_cur_frame_[i]->pos_[2];
                    temp_map.intensity = 0.;
                    sub_map_cur_frame_point->push_back(temp_map);
                }
                cv::Mat img_rgb = lidar_selector->img_cp;
                cv_bridge::CvImage out_msg;
                out_msg.header.stamp = ros::Time().fromSec(last_timestamp_img); // ros::Time().fromSec(last_timestamp_img) // ros::Time::now()
                // out_msg.header.frame_id = "camera_init";
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = img_rgb;
                img_pub.publish(out_msg.toImageMsg());


                Eigen::Vector3d wheel_encoder_delta_distance_In_WHEEL_ = Zero3d, wheel_encoder_latest_vel_In_WHEEL_ = Zero3d;

                    p_wheel_encoder->Process(LidarMeasures, wheel_encoder_delta_distance_In_WHEEL_, wheel_encoder_latest_vel_In_WHEEL_);
                    update_with_wheel_encoder(WHEEL_ENCODER_P_COV_, WHEEL_ENCODER_V_COV_, Wheel_encoder_R_wrt_IMU, wheel_encoder_delta_distance_In_WHEEL_, wheel_encoder_latest_vel_In_WHEEL_);


                prev_state = state;

                
                
                geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
                euler_cur = RotMtoEuler(state.rot_end);

            }
            double vio_end_time = omp_get_wtime();
            continue;
        }

        /*** Segment the map in lidar FOV ***/
        #ifndef USE_ikdforest            
            lasermap_fov_segment();
        #endif
        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
    #ifdef USE_ikdtree
        /*** initialize the map kdtree ***/
        #ifdef USE_ikdforest
        if (!ikdforest.initialized){
            if(feats_down_body->points.size() > 5){
                ikdforest.Build(feats_down_body->points, true, lidar_end_time);
            }
            continue;                
        }
        int featsFromMapNum = ikdforest.total_size;
        #else
        if(ikdtree.Root_Node == nullptr)
        {
            continue;
        }
        int featsFromMapNum = ikdtree.size();
        #endif
    #else
        if(featsFromMap->points.empty())
        {
            downSizeFilterMap.setInputCloud(feats_down_body);
        }
        else
        {
            downSizeFilterMap.setInputCloud(featsFromMap);
        }
        downSizeFilterMap.filter(*featsFromMap);
        int featsFromMapNum = featsFromMap->points.size();
    #endif
        feats_down_size = feats_down_body->points.size();
        cout<<"[ LIO ]: Raw feature num: "<<feats_undistort->points.size()<<" downsamp num "<<feats_down_size<<" Map num: "<<featsFromMapNum<< "." << endl;

        /*** ICP and iterated Kalman filter update ***/
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);
        //vector<double> res_last(feats_down_size, 1000.0); // initial //
        res_last.resize(feats_down_size, 1000.0);
        
        t1 = omp_get_wtime();
        if (lidar_en)
        {
            euler_cur = RotMtoEuler(state.rot_end);
            #ifdef USE_IKFOM
            //state_ikfom fout_state = kf.get_x();
            #endif
        }

    #ifdef USE_ikdtree
        if(0)
        {
            PointVector ().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
        }
    #else
        kdtreeSurfFromMap->setInputCloud(featsFromMap);
    #endif

        point_selected_surf.resize(feats_down_size, true);
        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();
        
        /*** iterated state estimation ***/
        #ifdef MP_EN
        printf("[ LIO ]: Using multi-processor, used core number: %d.\n", MP_PROC_NUM);
        #endif
        double t_update_start = omp_get_wtime();
        #ifdef USE_IKFOM
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        //state_ikfom updated_state = kf.get_x();
        state_point = kf.get_x();
        //euler_cur = RotMtoEuler(state_point.rot.toRotationMatrix());
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        // cout<<"position: "<<pos_lid.transpose()<<endl;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];
        #else

        if(img_en)
        {
            omp_set_num_threads(MP_PROC_NUM);
            #pragma omp parallel for
            for(int i=0;i<1;i++) {}
        }

        if(lidar_en)
        {
            for (iterCount = -1; iterCount < NUM_MAX_ITERATIONS && flg_EKF_inited; iterCount++) 
            {
                match_start = omp_get_wtime();
                PointCloudXYZI ().swap(*laserCloudOri);
                PointCloudXYZI ().swap(*corr_normvect);
                
                pcl_nearest_searched_in_map->clear();
                pcl_nearest_searched_in_map->reserve(feats_down_size);
                
                total_residual = 0.0; 

                /** closest surface search and residual computation **/
                #ifdef MP_EN
                    omp_set_num_threads(MP_PROC_NUM);
                    // #pragma omp parallel for
                #endif
                // normvec->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++)
                {
                    PointType &point_body  = feats_down_body->points[i];
                    PointType &point_world = feats_down_world->points[i];
                    V3D p_body(point_body.x, point_body.y, point_body.z);
                    /* transform to world frame */
                    pointBodyToWorld(&point_body, &point_world);
                    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                    #ifdef USE_ikdtree
                        auto &points_near = Nearest_Points[i];
                    #else
                        auto &points_near = pointSearchInd_surf[i];
                    #endif
                    uint8_t search_flag = 0;  
                    double search_start = omp_get_wtime();
                    if (nearest_search_en)
                    {
                        /** Find the closest surfaces in the map **/
                        #ifdef USE_ikdtree
                            #ifdef USE_ikdforest
                                search_flag = ikdforest.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5);
                            #else
                                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                            #endif
                        #else
                            kdtreeSurfFromMap->nearestKSearch(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                        #endif

                        point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;

                        #ifdef USE_ikdforest
                            point_selected_surf[i] = point_selected_surf[i] && (search_flag == 0);
                        #endif
                        kdtree_search_time += omp_get_wtime() - search_start;
                        kdtree_search_counter ++;                        
                    }


                    // if (!point_selected_surf[i]) continue;


                    if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS) continue;

                    VF(4) pabcd;
                    point_selected_surf[i] = false;
                    if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
                    {
                        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                        float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                        // if (s > 0.9 && pd2 < 0.2)
                        if (s > 0.9)
                        {
                            point_selected_surf[i] = true;
                            normvec->points[i].x = pabcd(0);
                            normvec->points[i].y = pabcd(1);
                            normvec->points[i].z = pabcd(2);
                            normvec->points[i].intensity = pd2;
                            res_last[i] = abs(pd2);


                            // save the point_nearest_searched_in_map
                            if (point_selected_surf[i] && (res_last[i] <= 2.0))
                            {
                                int selected_point_num_max = NUM_MATCH_POINTS;
                                // int selected_point_num_max = 2;
                                for (int j = 0; j < selected_point_num_max; j++)
                                {
                                    PointType point_nearest_searched_in_map;
                                    point_nearest_searched_in_map.x =  points_near[j].x;
                                    point_nearest_searched_in_map.y =  points_near[j].y;
                                    point_nearest_searched_in_map.z =  points_near[j].z;
                                    point_nearest_searched_in_map.intensity = 0.0;
                                    point_nearest_searched_in_map.normal_x = 0;
                                    point_nearest_searched_in_map.normal_y = 0;
                                    point_nearest_searched_in_map.normal_z = 0;
                                    pcl_nearest_searched_in_map->points.push_back(point_nearest_searched_in_map);
                                }
                            }

                        }
                    }
                }

                effct_feat_num = 0;
                laserCloudOri->resize(feats_down_size);
                corr_normvect->reserve(feats_down_size);
                for (int i = 0; i < feats_down_size; i++)
                {
                    if (point_selected_surf[i] && (res_last[i] <= 2.0))
                    {
                        laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                        corr_normvect->points[effct_feat_num] = normvec->points[i];
                        total_residual += res_last[i];
                        effct_feat_num ++;
                    }
                }

                res_mean_last = total_residual / effct_feat_num;
                match_time  += omp_get_wtime() - match_start;
                solve_start  = omp_get_wtime();
                
                /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                MatrixXd Hsub(effct_feat_num, 6);
                VectorXd meas_vec(effct_feat_num);

                for (int i = 0; i < effct_feat_num; i++)
                {
                    const PointType &laser_p  = laserCloudOri->points[i];
                    V3D point_this(laser_p.x, laser_p.y, laser_p.z);
                    point_this += Lidar_offset_to_IMU;
                    M3D point_crossmat;
                    point_crossmat<<SKEW_SYM_MATRX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = corr_normvect->points[i];
                    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

                    /*** calculate the Measuremnt Jacobian matrix H ***/
                    V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
                    Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                    /*** Measuremnt: distance to the closest surface/corner ***/
                    meas_vec(i) = - norm_p.intensity;
                }
                solve_const_H_time += omp_get_wtime() - solve_start;

                MatrixXd K(DIM_STATE, effct_feat_num);

                EKF_stop_flg = false;
                flg_EKF_converged = false;
                
                /*** Iterative Kalman Filter Update ***/
                if (!flg_EKF_inited)
                {
                    cout<<"||||||||||Initiallizing LiDar||||||||||"<<endl;
                    /*** only run in initialization period ***/
                    MatrixXd H_init(MD(9, DIM_STATE)::Zero());
                    MatrixXd z_init(VD(9)::Zero());
                    H_init.block<3,3>(0,0)  = M3D::Identity();
                    H_init.block<3,3>(3,3)  = M3D::Identity();
                    H_init.block<3,3>(6,15) = M3D::Identity();
                    z_init.block<3,1>(0,0)  = - Log(state.rot_end);
                    z_init.block<3,1>(0,0)  = - state.pos_end;

                    auto H_init_T = H_init.transpose();
                    auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + \
                                    0.0001 * MD(9, 9)::Identity()).inverse();
                    solution      = K_init * z_init;

                    // solution.block<9,1>(0,0).setZero();
                    // state += solution;
                    // state.cov = (MatrixXd::Identity(DIM_STATE, DIM_STATE) - K_init * H_init) * state.cov;

                    state.resetpose();
                    EKF_stop_flg = true;
                }
                else
                {
                    auto &&Hsub_T = Hsub.transpose();
                    auto &&HTz = Hsub_T * meas_vec;
                    H_T_H.block<6,6>(0,0) = Hsub_T * Hsub;
                    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
                    MD(DIM_STATE, DIM_STATE) &&K_1 = \
                            (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse();
                    G.block<DIM_STATE,6>(0,0) = K_1.block<DIM_STATE,6>(0,0) * H_T_H.block<6,6>(0,0);
                    auto vec = state_propagat - state;
                    solution = K_1.block<DIM_STATE,6>(0,0) * HTz + vec - G.block<DIM_STATE,6>(0,0) * vec.block<6,1>(0,0);

                    int minRow, minCol;
                    if(0)//if(V.minCoeff(&minRow, &minCol) < 1.0f)
                    {
                        VD(6) V = H_T_H.block<6,6>(0,0).eigenvalues().real();
                        cout<<"!!!!!! Degeneration Happend, eigen values: "<<V.transpose()<<endl;
                        EKF_stop_flg = true;
                        solution.block<6,1>(9,0).setZero();
                    }

                    state += solution;

                    rot_add = solution.block<3,1>(0,0);
                    t_add   = solution.block<3,1>(3,0);

                    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015))
                    {
                        flg_EKF_converged = true;
                    }

                    deltaR = rot_add.norm() * 57.3;
                    deltaT = t_add.norm() * 100;
                }
                euler_cur = RotMtoEuler(state.rot_end);
                

                /*** Rematch Judgement ***/
                nearest_search_en = false;
                if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
                {
                    nearest_search_en = true;
                    rematch_num ++;
                }

                /*** Convergence Judgements and Covariance Update ***/
                if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1)))
                {
                    if (flg_EKF_inited)
                    {
                        /*** Covariance Update ***/
                        // G.setZero();
                        // G.block<DIM_STATE,6>(0,0) = K * Hsub;
                        state.cov = (I_STATE - G) * state.cov;
                        total_distance += (state.pos_end - position_last).norm();
                        position_last = state.pos_end;
                        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                    (euler_cur(0), euler_cur(1), euler_cur(2));

                        VD(DIM_STATE) K_sum  = K.rowwise().sum();
                        VD(DIM_STATE) P_diag = state.cov.diagonal();

                    }
                    EKF_stop_flg = true;
                }
                solve_time += omp_get_wtime() - solve_start;

                if (EKF_stop_flg)   break;
            }
        }

        Eigen::Vector3d wheel_encoder_delta_distance_In_WHEEL_ = Zero3d, wheel_encoder_latest_vel_In_WHEEL_ = Zero3d;
            p_wheel_encoder->Process(LidarMeasures, wheel_encoder_delta_distance_In_WHEEL_, wheel_encoder_latest_vel_In_WHEEL_);
            update_with_wheel_encoder(WHEEL_ENCODER_P_COV_, WHEEL_ENCODER_V_COV_, Wheel_encoder_R_wrt_IMU, wheel_encoder_delta_distance_In_WHEEL_, wheel_encoder_latest_vel_In_WHEEL_);

        if (LidarMeasures.is_lidar_end) 
        {
            prev_state = state;
        }
        
        #endif
        
        double t_update_end = omp_get_wtime();
        /******* Publish odometry *******/
        euler_cur = RotMtoEuler(state.rot_end);
        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
        
        publish_odometry(pubOdomAftMapped, last_timestamp_lidar_end_time);

        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        
        t5 = omp_get_wtime();
        kdtree_incremental_time = t5 - t3 + readd_time;
        /******* Publish points *******/

        PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);          
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_pub = *laserCloudWorld;

        publish_frame_world(pubLaserCloudFullRes);
        
        publish_effect_world(pubLaserCloudEffect);
        
        publish_path(pubPath);
        #ifdef DEPLOY
        publish_mavros(mavros_pose_publisher);
        #endif

        /*** Debug variables ***/
        frame_num ++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
        aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
        #ifdef USE_IKFOM
        aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
        aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
        #else
        aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time)/frame_num;
        aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_const_H_time / frame_num;
        //cout << "construct H:" << aver_time_const_H_time << std::endl;
        #endif
        // aver_time_consu = aver_time_consu * 0.9 + (t5 - t0) * 0.1;
        T1[time_log_counter] = LidarMeasures.lidar_beg_time;
        s_plot[time_log_counter] = aver_time_consu;
        s_plot2[time_log_counter] = kdtree_incremental_time;
        s_plot3[time_log_counter] = kdtree_search_time/kdtree_search_counter;
        s_plot4[time_log_counter] = featsFromMapNum;
        s_plot5[time_log_counter] = t5 - t0;
        time_log_counter ++;
        // cout<<"[ LIO ]: time: fov_check "<< fov_check_time <<" fov_check and readd: "<<t1-t0<<" match "<<aver_time_match<<" solve "<<aver_time_solve<<" ICP "<<t3-t1<<" map incre "<<t5-t3<<" total "<<aver_time_consu << "icp:" << aver_time_icp << "construct H:" << aver_time_const_H_time <<endl;
        printf("[ LIO ]: time: fov_check: %0.6f fov_check and readd: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f icp: %0.6f construct H: %0.6f.\n",fov_check_time,t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
        if (lidar_en)
        {
            euler_cur = RotMtoEuler(state.rot_end);
            #ifdef USE_IKFOM

            #endif
        }
        
    }

    return 0;
}

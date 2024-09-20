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
#ifndef LIDAR_SELECTION_H_
#define LIDAR_SELECTION_H_

#include <common_lib.h>
#include <vikit/abstract_camera.h>
#include <frame.h>
#include <map.h>
#include <feature.h>
#include <point.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <set>

namespace lidar_selection {

class LidarSelector {
  public:
    int grid_size;
    double voxel_size;
    vk::AbstractCamera* cam;
    SparseMap* sparse_map;
    StatesGroup* state;
    StatesGroup* state_propagat;
    M3D Rli, Rci, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
    V3D Pli, Pci, Pcw;
    int* align_flag;
    int* grid_num;
    int* map_index;
    float* map_dist;
    float* map_value;
    float* patch_cache;
    float* patch_with_border_;
    int width, height, grid_n_width, grid_n_height, length;
    SubSparseMap* sub_sparse_map;
    double fx,fy,cx,cy;
    bool ncc_en;
    int debug, patch_size, patch_size_total, patch_size_half;
    int count_img, MIN_IMG_COUNT;
    int NUM_MAX_ITERATIONS;
    vk::robust_cost::WeightFunctionPtr weight_function_;
    float weight_scale_;
    double img_point_cov, outlier_threshold, ncc_thre;
    size_t n_meas_;                //!< Number of measurements
    deque< PointPtr > map_cur_frame_;
    deque< PointPtr > sub_map_cur_frame_;
    double computeH, ekf_time;
    double ave_total = 0.01;
    int frame_cont = 1;
    vk::robust_cost::ScaleEstimatorPtr scale_estimator_;

    Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
    MatrixXd H_sub, K;
    cv::flann::Index Kdtree;

    LidarSelector(const int grid_size, SparseMap* sparse_map);

    ~LidarSelector();

    void detect(cv::Mat img, PointCloudXYZI::Ptr pg, PointCloudXYZI::Ptr pg_nearest_searched_in_map);
    float CheckGoodPoints(cv::Mat img, V2D uv);
    void addFromSparseMap(cv::Mat img, PointCloudXYZI::Ptr pg);
    void addSparseMap(cv::Mat img, PointCloudXYZI::Ptr pg);
    void FeatureAlignment(cv::Mat img);
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void init();
    void getpatch(cv::Mat img, V3D pg, float* patch_tmp, int level);
    void getpatch(cv::Mat img, V2D pc, float* patch_tmp, int level);
    void dpi(V3D p, MD(2,3)& J);
    float UpdateState(cv::Mat img, float total_residual, int level);
    double NCC(float* ref_patch, float* cur_patch, int patch_size);

    void ComputeJ(cv::Mat img);
    void reset_grid();
    void addObservation(cv::Mat img);
    void reset();
    bool initialization(FramePtr cur_frame, PointCloudXYZI::Ptr pg);   
    void createPatchFromPatchWithBorder(float* patch_with_border, float* patch_ref);
    void getWarpMatrixAffine(
      const vk::AbstractCamera& cam,
      const Vector2d& px_ref,
      const Vector3d& f_ref,
      const double depth_ref,
      const SE3& T_cur_ref,
      const int level_ref,
      const int pyramid_level,
      const int halfpatch_size,
      Matrix2d& A_cur_ref);
    bool align2D(
      const cv::Mat& cur_img,
      float* ref_patch_with_border,
      float* ref_patch,
      const int n_iter,
      Vector2d& cur_px_estimate,
      int index);
    void AddPoint(PointPtr pt_new);
    int getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level);
    void display_keypatch(double time);
    void updateFrameState(StatesGroup state);
    V3F getpixel(cv::Mat img, V2D pc);

    void warpAffine(
      const Matrix2d& A_cur_ref,
      const cv::Mat& img_ref,
      const Vector2d& px_ref,
      const int level_ref,
      const int search_level,
      const int pyramid_level,
      const int halfpatch_size,
      float* patch);
    
    PointCloudXYZI::Ptr Map_points;
    PointCloudXYZI::Ptr Map_points_output;
    PointCloudXYZI::Ptr pg_down;
    PointCloudXYZI::Ptr visualmap_points_from_lidarmap;
    pcl::VoxelGrid<PointType> downSizeFilter;
    unordered_map<VOXEL_KEY, VOXEL_POINTS*> feat_map;
    unordered_map<VOXEL_KEY, float> sub_feat_map; //timestamp
    unordered_map<int, Warp*> Warp_map; // reference frame id, A_cur_ref and search_level

    vector<VOXEL_KEY> occupy_postions;
    set<VOXEL_KEY> sub_postion;
    vector<PointPtr> voxel_points_;
    vector<V3D> add_voxel_points_;


    cv::Mat img_cp, img_rgb;
    std::vector<FramePtr> overlap_kfs_;
    FramePtr new_frame_;
    FramePtr last_kf_;
    Map map_;
    enum Stage {
      STAGE_FIRST_FRAME,
      STAGE_DEFAULT_FRAME
    };
    Stage stage_;
    enum CellType {
      TYPE_MAP = 1,
      TYPE_POINTCLOUD,
      TYPE_UNKNOWN
    };

  private:
    struct Candidate 
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      PointPtr pt;       
      Vector2d px;    
      Candidate(PointPtr pt, Vector2d& px) : pt(pt), px(px) {}
    };
    typedef std::list<Candidate > Cell;
    typedef std::vector<Cell*> CandidateGrid;

    /// The grid stores a set of candidate matches. For every grid cell we try to find one match.
    struct Grid
    {
      CandidateGrid cells;
      vector<int> cell_order;
      int cell_size;
      int grid_n_cols;
      int grid_n_rows;
      int cell_length;
    };

    Grid grid_;
};
  typedef boost::shared_ptr<LidarSelector> LidarSelectorPtr;

} // namespace lidar_detection

#endif // LIDAR_SELECTION_H_
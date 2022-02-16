#include <numeric>

#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "VisCheck.hpp"

using visc::VisCheck;
using visc::Pt2d;
using visc::Pt3d;
using visc::Pt2dCloud;
using visc::Pt3dCloud;

void VisCheck::SetInputCloud(const Pt3dCloud::ConstPtr cloud_ptr){
    cloud_3d_ptr_ = cloud_ptr;
}

void VisCheck::SetCamera(const CamIntrinsics& intri, const RigidTransform6d& pose, unsigned int img_width, unsigned int img_height){
    cam_intri_ = intri;
    cam_pose_ = pose;
    img_width_ = img_width;
    img_height_ = img_height;
}

void VisCheck::ComputeVisibility(PtIndices& visible_pts){
    // Transform to camera frame
    Pt3dCloud::Ptr cloud_in_cam(new Pt3dCloud());
    pcl::transformPointCloud(*cloud_3d_ptr_, *cloud_in_cam, cam_pose_.matrix());

    // After projection, out-of-image points are discarded
    // Corresponging indices are stored in proj_indices
    Pt2dCloud::Ptr proj_pts(new Pt2dCloud);
    std::vector<unsigned int> proj_indices;
    ProjectToImageSpace(cloud_in_cam, proj_pts, proj_indices);

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(proj_pts);

    // vector for storing scores (init with 0.0)
    std::vector<float> vis_scores(proj_pts->size());
    std::fill(vis_scores.begin(), vis_scores.end(), 0.0);

    for(size_t i = 0; i < proj_pts->size(); ++i){
        const Pt2d& pt_search = proj_pts->at(i);

        // Results are stored in distance-increasing order
        // Reference: https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search
        std::vector<int> n_idx(k_);
        std::vector<float> n_squared_dis(k_);
        if(!(kdtree.nearestKSearch(pt_search, k_, n_idx, n_squared_dis) > 0)){
            continue;
        }

        std::vector<float> pt_depth(k_); // distance between 3d point and origin in camera frame
        for(size_t j = 0; j < k_; ++j){
            const Pt3d& pt_n = cloud_in_cam->at(proj_indices[n_idx[j]]);  // Point neighbor
            float euclidean_dis = ComputeEuclideanDistToOrigin(pt_n);
            pt_depth[j] = euclidean_dis;
        }
        
        // Add depth of the current point for searching
        float pt_search_depth = ComputeEuclideanDistToOrigin(cloud_in_cam->at(proj_indices[i]));
        pt_depth.push_back(pt_search_depth);

        // Find max and min depth
        float max_depth, min_depth;
        const auto minmax_res = std::minmax_element(std::begin(pt_depth), std::end(pt_depth));
        min_depth = *minmax_res.first;
        max_depth = *minmax_res.second;

        float vis_score = ComputeVisibilityScore(pt_search_depth, min_depth, max_depth);

        // if(vis_score < vis_score_thresh_){continue;}
        vis_scores[i] = vis_score;
        // visible_pts.indices.push_back(proj_indices[i]);
    }

    mean_vis_score_ = std::reduce(vis_scores.begin(), vis_scores.end()) / vis_scores.size();

    for(size_t i = 0; i < proj_pts->size(); ++i){
        if(vis_scores[i] < mean_vis_score_ + mean_shift_) { continue; }
        visible_pts.indices.push_back(proj_indices[i]);
    }
}

float VisCheck::ComputeVisibilityScore(float d, float d_min, float d_max){
    float num = (d - d_min) * (d - d_min);
    float denum = (d_max - d_min) * (d_max - d_min);
    return exp(- num / denum);
}

float VisCheck::ComputeEuclideanDistToOrigin(const Pt3d& pt){
    return sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
}

void VisCheck::ProjectToImageSpace(Pt3dCloud::ConstPtr cloud_3d, Pt2dCloud::Ptr cloud_proj, std::vector<unsigned int>& indice){
    if(!cloud_proj){
        cloud_proj = Pt2dCloud::Ptr(new Pt2dCloud());
    }

    cloud_proj->clear();

    for(size_t i = 0; i < cloud_3d->size(); ++i){
        const Pt3d& pt = cloud_3d->at(i);
        
        if(pt.z < 0.0f){continue;}
        
        Pt2d proj;
        proj.x = cam_intri_.fx * pt.x / pt.z + cam_intri_.cx;
        proj.y = cam_intri_.fy * pt.y / pt.z + cam_intri_.cy;

        if (proj.x < 0 || proj.x >= img_width_ || proj.y < 0 || proj.y >= img_height_) { continue; }

        cloud_proj->push_back(proj);
        indice.push_back(i);
    }
}
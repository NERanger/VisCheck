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

void VisCheck::SetCamera(const CamIntrinsics& intri, const RigidTransform6d& pose){
    cam_intri_ = intri;
    cam_pose_ = pose;
}

void VisCheck::ComputeVisibility(PtIndices& visible_pts){
    // Transform to camera frame
    Pt3dCloud::Ptr cloud_in_cam(new Pt3dCloud());
    pcl::transformPointCloud(*cloud_3d_ptr_, *cloud_in_cam, cam_pose_.matrix());

    // Projected points are stored in the same order with 3d cloud
    // Same index means same point
    Pt2dCloud::Ptr proj_pts;
    proj_pts = ProjectToImageSpace(cloud_in_cam);

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(proj_pts);

    for(size_t i = 0; i < proj_pts->size(); ++i){
        const Pt2d& pt_search = proj_pts->at(i);

        // Results are stored in distance-increasing order
        // Reference: https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html#kdtree-search
        std::vector<int> pt_idx(k_);
        std::vector<float> pt_squared_dis(k_);
        if(!kdtree.nearestKSearch(pt_search, k_, pt_idx, pt_squared_dis) > 0){
            continue;
        }

        std::vector<float> pt_depth(k_); // distance between 3d point and origin in camera frame
        for(size_t j = 0; j < k_; ++j){
            const Pt3d& pt_n = cloud_in_cam->at(pt_idx[j]);  // Point neighbor
            float euclidean_dis = ComputeEuclideanDistToOrigin(pt_n);
            pt_depth[j] = euclidean_dis;
        }
        
        // Add depth of the current point for searching
        float pt_search_depth = ComputeEuclideanDistToOrigin(cloud_in_cam->at(i));
        pt_depth.push_back(pt_search_depth);

        // Find max and min depth
        auto max_it = std::max_element(std::begin(pt_depth), std::end(pt_depth));
        auto min_it = std::min_element(std::begin(pt_depth), std::end(pt_depth));
        float max_depth = *max_it;
        float min_depth = *min_it;

        float vis_score = ComputeVisibilityScore(pt_search_depth, min_depth, max_depth);

        if(vis_score < vis_score_thresh_){continue;}

        visible_pts.indices.push_back(i);
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

Pt2dCloud::Ptr VisCheck::ProjectToImageSpace(Pt3dCloud::ConstPtr cloud_3d){
    Pt2dCloud::Ptr out(new Pt2dCloud());
    out->reserve(cloud_3d->size());
    for(const Pt3d& pt : cloud_3d->points){
        Pt2d proj;
        proj.x = cam_intri_.fx * pt.x / pt.z + cam_intri_.cx;
        proj.y = cam_intri_.fy * pt.y / pt.z + cam_intri_.cy;

        out->push_back(proj);
    }

    return out;
}
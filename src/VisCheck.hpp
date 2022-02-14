#pragma once

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace visc{

using Pt3d = pcl::PointXYZ;
using Pt2d = pcl::PointXY;
using Pt3dCloud = pcl::PointCloud<Pt3d>;
using Pt2dCloud = pcl::PointCloud<Pt2d>;

using PtIndices = pcl::PointIndices;

using RigidTransform6d = Eigen::Isometry3f;

struct CamIntrinsics{
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
};

class VisCheck{
public:
    VisCheck() = default;

    inline void SetK(unsigned int k) { k_ = k; } // Set number of nearest neighbor for knn search

    void SetInputCloud(const Pt3dCloud::ConstPtr cloud_ptr);
    void SetCamera(const CamIntrinsics& intri, const RigidTransform6d& pose);
    void ComputeVisibility(PtIndices& visible_pts);
private:
    float ComputeVisibilityScore(float d, float d_min, float d_max);
    float ComputeEuclideanDistToOrigin(const Pt3d& pt);

    Pt2dCloud::Ptr ProjectToImageSpace(Pt3dCloud::ConstPtr cloud_3d);

    Pt3dCloud::ConstPtr cloud_3d_ptr_ = nullptr;

    RigidTransform6d cam_pose_; // Transform from pointcloud frame to camera frame
    CamIntrinsics cam_intri_;

    unsigned int k_ = 7; // Number of nearest neighbor for knn search
    float vis_score_thresh_ = 0.95f;
};

}
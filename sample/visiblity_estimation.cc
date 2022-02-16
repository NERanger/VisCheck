#include <pcl/io/pcd_io.h>

#include "VisCheck.hpp"

using std::cout;
using std::endl;

int main(int argc, char const *argv[]){

    if(argc != 3){
        cout << "Usage: ./visibility_estimation <path-to-ptcloud> <path-for-output>" << endl;
        return EXIT_FAILURE;
    }

    std::string out_path(std::string(argv[2]) + "visibility.pcd");

    std::string ptcloud_path(argv[1]);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(ptcloud_path, *ptcloud);
    cout << "Loaded points: " << ptcloud->size() << endl;

    visc::Pt3dCloud::Ptr ptcloud_xyz(new visc::Pt3dCloud);
    pcl::copyPointCloud(*ptcloud, *ptcloud_xyz);

    visc::CamIntrinsics cam_intri{707.091, 707.091, 601.887, 183.11};
    visc::RigidTransform6d cam_pose;  // For trnsform ptcloud points into camera frame
    cam_pose.matrix() << -0.00185776,   -0.999966, -0.00804016, -0.00478403,
                         -0.00648159,  0.00805223,   -0.999946,  -0.0733743,
                         0.999977, -0.00180569, -0.00649607,   -0.333997,
                         0,           0,           0,           1;

    visc::VisCheck checker;
    checker.SetCamera(cam_intri, cam_pose, 1226, 370);
    checker.SetInputCloud(ptcloud_xyz);
    checker.SetK(50);
    checker.SetVisScoreThreshMeanShift(0.0f);

    visc::PtIndices result;
    checker.ComputeVisibility(result);

    cout << "visible point num: " << result.indices.size() << endl;
    cout << "Mean visibility score: " << checker.GetMeanVisScore() << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*ptcloud_xyz, *vis_cloud);

    uint32_t color_r = (static_cast<uint32_t>(255) << 16 |
                       static_cast<uint32_t>(0) << 8 | 
                       static_cast<uint32_t>(0));
    uint32_t color_g = (static_cast<uint32_t>(0) << 16 |
                       static_cast<uint32_t>(255) << 8 | 
                       static_cast<uint32_t>(0));
    for(auto &p : vis_cloud->points){p.rgb = *reinterpret_cast<float*>(&color_r);}

    for(const auto &idx : result.indices){
        vis_cloud->points[idx].rgb = *reinterpret_cast<float*>(&color_g);
    }

    pcl::io::savePCDFileBinary(out_path, *vis_cloud);

    cout << "Visibility cloud saved to: " << out_path << endl;

    return EXIT_SUCCESS;
}

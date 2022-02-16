# VisCheck

Given a point cloud and parameter of a camera (intrinsics, pose, image dimension), this repo can give an estimation of whether a point can be observed by the camera.

The following image is a sample output. Green point means "visible" while red points means "not visible"

![sample output](./docs/sample.png)

## Acknowledge

The algorithm in this repo is an implementation of [this paper](https://www.scitepress.org/Papers/2019/73086/pdf/index.html).

> Biasutti, P., Bugeau, A., Aujol, J. F., & Brédif, M. (2019). Visibility estimation in point clouds with variable density. VISIGRAPP 2019 - Proceedings of the 14th International Joint Conference on Computer Vision, Imaging and Computer Graphics Theory and Applications, 4, 27–35. https://doi.org/10.5220/0007308600270035

## Usage Sample

Check for the [sample program](./sample/visiblity_estimation.cc)

The hard-coded camera params can be used with the pointcloud in `data/`
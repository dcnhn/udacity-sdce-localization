#include "icp.h"

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations)
{
    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity ();

    return transformationMatrix;
}
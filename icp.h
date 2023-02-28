#ifndef ICP_H
#define ICP_H

#include <Eigen/Geometry>
#include "helper.h"

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);

#endif /* ICP_H */
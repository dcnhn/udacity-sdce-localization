#include "icp.h"
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>


Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations)
{
    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

    // align source with starting pose
    Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
    PointCloudT::Ptr transformSource(new PointCloudT); 
    pcl::transformPointCloud(*source, *transformSource, initTransform);


    pcl::console::TicToc time;
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(transformSource);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(2);

#if 0
    icp.setTransformationEpsilon(0.001);
    icp.setEuclideanFitnessEpsilon(.05);
    icp.setRANSACOutlierRejectionThreshold(10);
#endif

    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud
    icp.align(*cloud_icp);
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

    if (icp.hasConverged())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        transformationMatrix = icp.getFinalTransformation().cast<double>();
        transformationMatrix =  transformationMatrix * initTransform;
    }
    else
    {
        std::cout << "WARNING: ICP did not converge" << std::endl;
    }

    return transformationMatrix;
}
#pragma once
#include <iostream>
#include <string>
#include <test_bench.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/gicp.h>

template <typename PointType>
class GICPTest : public TestBench<PointType>
{
  public:
    /**
     * \brief fill your algorithm name behind alg_name
     */
	  GICPTest() : alg_name("GICP")
    {
        /*************start your code below*************/
    }

    void loadCandidateConfig()
    {
        YAML::Node alg_cfg;
        TestBench<PointType>::reg_candidate_name = alg_name;
        std::string path = TestBench<PointType>::cfg_file["gicp_cfg_path"].as<std::string>();
        alg_cfg = YAML::LoadFile(path);

        /*************start your code below*************/
		int max_iter = alg_cfg["max_iter"].as<int>();
		float max_corr_dis = alg_cfg["max_corr_dis"].as<float>();
		reg.setMaxCorrespondenceDistance(max_corr_dis);
		reg.setMaximumIterations(max_iter);
		reg.setTransformationEpsilon(1e-8);
    }

    bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud, const pcl::PointCloud<PointType> &tgt_cloud,
                     pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f &transform)
    {
		reg.setInputSource(src_cloud.makeShared());
		reg.setInputTarget(tgt_cloud.makeShared());
		reg.align(reg_cloud);
		transform = reg.getFinalTransformation();
        return reg.hasConverged(); // only return true if the registration is converge
    }

  private:
    std::string alg_name; // algorithm name, this will pass to the test bench
	pcl::GeneralizedIterativeClosestPoint<PointType, PointType> reg;
};
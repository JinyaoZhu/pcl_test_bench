#pragma once
#include <iostream>
#include <string>
#include <test_bench.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>

template <typename PointType>
class NDTTest : public TestBench<PointType>
{
  public:
    /**
     * \brief fill your algorithm name behind alg_name
     */
	  NDTTest() : alg_name("NDT")
    {
        /*************start your code below*************/
    }

    void loadCandidateConfig()
    {
        YAML::Node alg_cfg;
        TestBench<PointType>::reg_candidate_name = alg_name;
        std::string path = TestBench<PointType>::cfg_file["ndt_cfg_path"].as<std::string>();
        alg_cfg = YAML::LoadFile(path);

        /*************start your code below*************/
		int max_iter = alg_cfg["max_iter"].as<int>();
		float step_size = alg_cfg["step_size"].as<float>();
		float resolution = alg_cfg["resolution"].as<float>();
		float eps = alg_cfg["eps"].as<float>();
		reg.setMaximumIterations(max_iter);
		// Setting minimum transformation difference for termination condition.
		reg.setTransformationEpsilon(eps);
		// Setting maximum step size for More-Thuente line search.
		reg.setStepSize(step_size);
		//Setting Resolution of NDT grid structure (VoxelGridCovariance).
		reg.setResolution(resolution);
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

	double getFitness() {
		return reg.getFitnessScore();
	}

  private:
    std::string alg_name; // algorithm name, this will pass to the test bench
	pcl::NormalDistributionsTransform<PointType, PointType> reg;
};
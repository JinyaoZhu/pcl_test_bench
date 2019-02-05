#pragma once
#pragma warning( disable : 4996)
#include <iostream>
#include <string>
#include <test_bench.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

template <typename PointType>
class IcpTest : public TestBench<PointType> {

public:
	IcpTest():alg_name("ICP")
	{

	}


	void loadCandidateConfig() {

		YAML::Node alg_cfg;
		TestBench<PointType>::reg_candidate_name = alg_name;
		std::string path = TestBench<PointType>::cfg_file["icp_cfg_path"].as<std::string>();
		alg_cfg = YAML::LoadFile(path);

		int icp_max_iter = alg_cfg["icp_max_iter"].as<int>();
		float max_corr_dis = alg_cfg["max_corr_dis"].as<float>();
		float eps = alg_cfg["transformation_esp"].as<float>();
		icp.setMaximumIterations(icp_max_iter);
		icp.setMaxCorrespondenceDistance(max_corr_dis);
		icp.setTransformationEpsilon(eps);
	}

	bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud, const pcl::PointCloud<PointType> &tgt_cloud,
		pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f& transform) {

		typename pcl::PointCloud<PointType>::Ptr src_cloud_ptr = src_cloud.makeShared();
		typename pcl::PointCloud<PointType>::Ptr tgt_cloud_ptr = tgt_cloud.makeShared();

		
		icp.setInputSource(src_cloud_ptr);
		icp.setInputTarget(tgt_cloud_ptr);
		icp.align(reg_cloud);

		transform = icp.getFinalTransformation();

		return icp.hasConverged();
	}

	double getFitness() {
		return icp.getFitnessScore();
	}

private:
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	std::string alg_name;

};
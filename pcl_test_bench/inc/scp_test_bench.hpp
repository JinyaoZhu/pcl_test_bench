#pragma once
#include <iostream>
#include <string>
#include <test_bench.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#define ALGORITHM_NAME ("Sample Consensus Prerejective")

template <typename PointType>
class ScpTestBench : public TestBench<PointType> {

public:
	ScpTestBench():tree_ptr(new pcl::search::KdTree<PointType>()), is_first_loop(true)
	{
		TestBench<PointType>::reg_algorithm_name = std::string(ALGORITHM_NAME);
		norm_est.setSearchMethod(tree_ptr);
		fpfh_est.setSearchMethod(tree_ptr);
	}

	~ScpTestBench() {}

	void loadScpConfig(std::string path) {

		YAML::Node scp_cfg;
		float norm_est_radius_search;
		float fpfh_est_radius_search;
		float scp_max_corr_distance;
		int scp_max_iter;
		float scp_sim_threshold;
		int scp_corr_randomness;
		float scp_inlier_fraction;
		int scp_number_of_samples;

		scp_cfg = YAML::LoadFile(path);

		norm_est_radius_search = scp_cfg["norm_est_RadiusSearch"].as<float>();
		fpfh_est_radius_search = scp_cfg["fpfh_est_RadiusSearch"].as<float>();
		scp_max_corr_distance = scp_cfg["scp_max_corr_distance"].as<float>();
		scp_max_iter = scp_cfg["scp_max_iter"].as<int>();
		scp_sim_threshold = scp_cfg["scp_sim_threshold"].as<float>();
		scp_corr_randomness = scp_cfg["scp_corr_randomness"].as<int>();
		scp_inlier_fraction = scp_cfg["scp_inlier_fraction"].as<float>();
		scp_number_of_samples = scp_cfg["scp_number_of_samples"].as<int>();

		pcl::console::print_info("Load SCP parameter: norm_est_radius_search = %f \n", norm_est_radius_search);
		pcl::console::print_info("Load SCP parameter: fpfh_est_radius_search = %f \n", fpfh_est_radius_search);
		pcl::console::print_info("Load SCP parameter: scp_max_corr_distance = %f \n", scp_max_corr_distance);
		pcl::console::print_info("Load SCP parameter: scp_max_iter = %d \n", scp_max_iter);
		pcl::console::print_info("Load SCP parameter: scp_sim_threshold = %f \n", scp_sim_threshold);
		pcl::console::print_info("Load SCP parameter: scp_corr_randomness = %d \n", scp_corr_randomness);
		pcl::console::print_info("Load SCP parameter: scp_inlier_fraction = %f \n", scp_inlier_fraction);
		pcl::console::print_info("Load SCP parameter: scp_number_of_samples = %d \n", scp_number_of_samples);

		norm_est.setRadiusSearch(norm_est_radius_search);
		fpfh_est.setRadiusSearch(fpfh_est_radius_search);

		scp_reg.setMaxCorrespondenceDistance(scp_max_corr_distance);
		scp_reg.setMaximumIterations(scp_max_iter);
		scp_reg.setSimilarityThreshold(scp_sim_threshold);
		scp_reg.setCorrespondenceRandomness(scp_corr_randomness);
		scp_reg.setInlierFraction(scp_inlier_fraction);
		scp_reg.setNumberOfSamples(scp_number_of_samples);
	}

	bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud,const pcl::PointCloud<PointType> &tgt_cloud,
		             pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f& transform) {

		typename pcl::PointCloud<PointType>::Ptr src_cloud_ptr = src_cloud.makeShared();
		typename pcl::PointCloud<PointType>::Ptr tgt_cloud_ptr = tgt_cloud.makeShared();


		if(is_first_loop){
			// Estimate the normals and the FPFH features for the source cloud
			norm_est.setInputCloud(src_cloud_ptr);
			norm_est.compute(norm_est_src);

			fpfh_est.setInputCloud(src_cloud_ptr);
			fpfh_est.setInputNormals(norm_est_src.makeShared());
			fpfh_est.compute(features_src);
			is_first_loop = false;
		}
		
		// Estimate the normals and the FPFH features for the target cloud
		norm_est.setInputCloud(tgt_cloud_ptr);
		norm_est.compute(norm_est_tgt);

		fpfh_est.setInputCloud(tgt_cloud_ptr);
		fpfh_est.setInputNormals(norm_est_tgt.makeShared());
		fpfh_est.compute(features_tgt);

		scp_reg.setInputSource(src_cloud_ptr);
		scp_reg.setInputTarget(tgt_cloud_ptr);
		scp_reg.setSourceFeatures(features_src.makeShared());
		scp_reg.setTargetFeatures(features_tgt.makeShared());

		scp_reg.align(reg_cloud);

		transform = scp_reg.getFinalTransformation();

		return scp_reg.hasConverged();
	}

private:
	bool is_first_loop;

	pcl::SampleConsensusPrerejective<PointType, PointType, pcl::FPFHSignature33> scp_reg;
	pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
	pcl::PointCloud<pcl::FPFHSignature33> features_src, features_tgt;
	pcl::PointCloud<pcl::Normal> norm_est_src, norm_est_tgt;
	pcl::PointCloud<PointType> src_cloud;
	pcl::PointCloud<PointType> tgt_cloud;
	typename pcl::search::KdTree<PointType>::Ptr tree_ptr;

};
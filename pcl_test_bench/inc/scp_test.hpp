#pragma once
#include <iostream>
#include <string>
#include <test_bench.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/brute_force.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>


template <typename PointType>
class ScpTest : public TestBench<PointType> {

public:
	ScpTest() :alg_name("SCP")
	{
		typename pcl::search::KdTree<PointType>::Ptr tree_ptr = boost::make_shared<pcl::search::KdTree<PointType>>(kd_tree);
		//typename pcl::search::BruteForce< PointType >::Ptr tree_ptr(new pcl::search::BruteForce< PointType >);
		norm_est.setSearchMethod(tree_ptr);
		fpfh_est.setSearchMethod(tree_ptr);
	}


	void loadCandidateConfig() {

		YAML::Node alg_cfg;
		TestBench<PointType>::reg_candidate_name = alg_name;
		std::string path = TestBench<PointType>::cfg_file["scp_cfg_path"].as<std::string>();
		alg_cfg = YAML::LoadFile(path);

		float norm_est_radius_search;
		float fpfh_est_radius_search;
		float scp_max_corr_distance;
		int scp_max_iter;
		float scp_sim_threshold;
		int scp_corr_randomness;
		float scp_inlier_fraction;
		int scp_number_of_samples;

		norm_est_radius_search = alg_cfg["norm_est_radius_search"].as<float>();
		fpfh_est_radius_search = alg_cfg["fpfh_est_radius_search"].as<float>();
		scp_max_corr_distance = alg_cfg["scp_max_corr_distance"].as<float>();
		scp_max_iter = alg_cfg["scp_max_iter"].as<int>();
		scp_sim_threshold = alg_cfg["scp_sim_threshold"].as<float>();
		scp_corr_randomness = alg_cfg["scp_corr_randomness"].as<int>();
		scp_inlier_fraction = alg_cfg["scp_inlier_fraction"].as<float>();
		scp_number_of_samples = alg_cfg["scp_number_of_samples"].as<int>();

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

	bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud, const pcl::PointCloud<PointType> &tgt_cloud,
		pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f& transform) {

		typename pcl::PointCloud<PointType>::Ptr src_cloud_ptr = src_cloud.makeShared();
		typename pcl::PointCloud<PointType>::Ptr tgt_cloud_ptr = tgt_cloud.makeShared();


		// Estimate the normals and the FPFH features for the source cloud
		norm_est.setInputCloud(src_cloud_ptr);
		norm_est.compute(norm_est_src);

		fpfh_est.setInputCloud(src_cloud_ptr);
		fpfh_est.setInputNormals(norm_est_src.makeShared());
		fpfh_est.compute(features_src);

		// Estimate the normals and the FPFH features for the target cloud
		norm_est.setInputCloud(tgt_cloud_ptr);
		norm_est.compute(norm_est_tgt);

		fpfh_est.setInputCloud(tgt_cloud_ptr);
		fpfh_est.setInputNormals(norm_est_tgt.makeShared());
		//stop_watch.reset();
		fpfh_est.compute(features_tgt);
		//pcl::console::print_info("FPFH compute time: %.5f (s)\n", stop_watch.getTimeSeconds());

		scp_reg.setInputSource(src_cloud_ptr);
		scp_reg.setInputTarget(tgt_cloud_ptr);
		scp_reg.setSourceFeatures(features_src.makeShared());
		scp_reg.setTargetFeatures(features_tgt.makeShared());

		scp_reg.align(reg_cloud);

		transform = scp_reg.getFinalTransformation();

		return scp_reg.hasConverged();
	}

	double getFitness() {
		return scp_reg.getFitnessScore();
	}

private:
	std::string alg_name;

	pcl::SampleConsensusPrerejective<PointType, PointType, pcl::FPFHSignature33> scp_reg;
	pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	pcl::NormalEstimationOMP<PointType, pcl::Normal> norm_est;
	pcl::PointCloud<pcl::FPFHSignature33> features_src, features_tgt;
	pcl::PointCloud<pcl::Normal> norm_est_src, norm_est_tgt;
	pcl::PointCloud<PointType> src_cloud;
	pcl::PointCloud<PointType> tgt_cloud;
	pcl::search::KdTree<PointType> kd_tree;

	pcl::StopWatch stop_watch;
};
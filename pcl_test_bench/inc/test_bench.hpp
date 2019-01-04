#pragma once
#include <iostream>
#include <vector>
#include <type_traits>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>

template <typename PointType>
class TestBench{

public:
	/**
	* @brief
	*
	*/
	TestBench() {
		visualizer_ptr = new pcl::visualization::PCLVisualizer();
		visualizer_ptr->setWindowName("TestBench");
		visualizer_ptr->createViewPort(0.0, 0, 0.5, 1.0, view_port_1);
		visualizer_ptr->createViewPort(0.5, 0, 1.0, 1.0, view_port_2);
	}

	~TestBench() {
	}


	/**
	* @brief
	*
	*/
	void loadTestBenchConfig(std::string path) {
		cfg_file = YAML::LoadFile(path);
		grid_filter_leafsize = cfg_file["grid_filter_leafsize"].as<float>();
		pcd_file_paths = cfg_file["pcd_files_path"].as<std::vector<std::string>>();

		pcl::console::print_info("[TESTBENCH] grid_filter_leafsize = %f\n", grid_filter_leafsize);
		loadSrcFromPCDFile(pcd_file_paths.front());
		loadTgtFromPCDFile(std::vector<std::string>(pcd_file_paths.begin() + 1, pcd_file_paths.end()));
	}

	/**
	* @brief
	*
	*/
	void preprocessPointCloud() {
		pcl::VoxelGrid<PointType> grid;
		grid.setLeafSize(grid_filter_leafsize, grid_filter_leafsize, grid_filter_leafsize);

		grid.setInputCloud(cloud_src_ptr);
		grid.filter(cloud_src_filtered);
		cloud_src_filtered_ptr = cloud_src_filtered.makeShared();
		pcl::console::print_info("[TESTBENCH] Filtered source point cloud size: %d / %d\n", 
			cloud_src_filtered.size(), cloud_src.size());
		
		for (auto &cloud_ptr : clouds_tgt_ptr) {
			grid.setInputCloud(cloud_ptr);
			clouds_tgt_filtered.emplace_back();
			grid.filter(clouds_tgt_filtered.back());
			clouds_tgt_filtered_ptr.push_back(clouds_tgt_filtered.back().makeShared());
			pcl::console::print_info("[TESTBENCH] Filtered target point cloud size: %d / %d\n",
				clouds_tgt_filtered.back().size(), cloud_ptr->size());
		}
	}

	/**
	* @brief
	*
	*/
	void computeResult() {
		pcl::console::print_info("\n[TESTBENCH] Algorithm: %s \n", reg_algorithm_name.c_str());
		for (int i = 0; i < results.size(); ++i) {
			Result result = results[i];
			Eigen::Matrix4f T = result.final_transformation;
			pcl::console::print_info("\n==============TRANSFORMATION %d=================\n",i);
			pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", T(0, 0), T(0, 1), T(0, 2));
			pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", T(1, 0), T(1, 1), T(1, 2));
			pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", T(2, 0), T(2, 1), T(2, 2));
			pcl::console::print_info("t = [ %0.3f, %0.3f, %0.3f ]\n", T(0, 3), T(1, 3), T(2, 3));
			pcl::console::print_info("has converged: %s\n", result.is_converged?"true":"false");
			pcl::console::print_info("file: %s\n", result.pcd_file);
		}
	}

	/**
	* @brief
	*
	*/
	void vizSrcTgt() {

		if (std::is_same<PointType, pcl::PointXYZ>::value) {
			std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointType>> tgts_h;

			pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(cloud_src_ptr, 255, 0, 0);
			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp1_source", view_port_1);

			for (auto& pcd_ptr : clouds_tgt_ptr) {
				static int idx = 0;
				tgts_h.emplace_back(pcd_ptr, 0, 255, 0);
				visualizer_ptr->addPointCloud(pcd_ptr, tgts_h.back(), "vp1_target"+std::to_string(idx++), view_port_1);
			}
		}

		if (std::is_same<PointType, pcl::PointXYZRGB>::value) {
			std::vector<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> tgts_h;

			pcl::visualization::PointCloudColorHandlerRGBField<PointType> src_h(cloud_src_ptr);

			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp1_source", view_port_1);
			
			for (auto& pcd_ptr : clouds_tgt_ptr) {
				static int idx = 0;
				tgts_h.emplace_back(pcd_ptr);
				visualizer_ptr->addPointCloud(pcd_ptr, tgts_h.back(), "vp1_target"+std::to_string(idx++), view_port_1);
			}
		}

		visualizer_ptr->initCameraParameters();

		pcl::console::print_info("\n[TESTBENCH] Press q to begin the registration.\n");
		visualizer_ptr->spin();
	}

	/**
	* @brief
	*
	*/
	void vizResult() {

		if (std::is_same<PointType, pcl::PointXYZ>::value) {
			std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointType>> tgts_h;

			pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(cloud_src_ptr, 255, 0, 0);
			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);

			for (auto& pcd_ptr : clouds_reg_ptr) {
				static int idx = 0;
				tgts_h.emplace_back(pcd_ptr, 0, 255, 0);
				visualizer_ptr->addPointCloud(pcd_ptr, tgts_h.back(), "vp2_target" + std::to_string(idx++), view_port_2);
			}
		}

		if (std::is_same<PointType, pcl::PointXYZRGB>::value) {
			std::vector<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> tgts_h;

			pcl::visualization::PointCloudColorHandlerRGBField<PointType> src_h(cloud_src_ptr);

			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);

			for (auto& pcd_ptr : clouds_reg_ptr) {
				static int idx = 0;
				tgts_h.emplace_back(pcd_ptr);
				visualizer_ptr->addPointCloud(pcd_ptr, tgts_h.back(), "vp2_target" + std::to_string(idx++), view_port_2);
			}
		}

		pcl::console::print_info("\n[TESTBENCH] Press q to quit.\n");
		visualizer_ptr->spin();
	}

	/**
	* @brief
	*
	*/
	virtual bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud, const pcl::PointCloud<PointType> &tgt_cloud,
		                     pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f& transform) = 0;

	/**
	* @brief
	*
	*/
	void doAlignAll() {
		pcl::PointCloud<PointType> reg_cloud, reg_cloud_filtered;
		Eigen::Matrix4f final_transform;
		pcl::console::print_info("[TESTBENCH] Start registration...\n");
		for (int i = 0; i < clouds_tgt_filtered.size(); ++i) {
			bool is_converged = doAlignOnce(cloud_src_filtered, clouds_tgt_filtered[i], reg_cloud_filtered, final_transform);
			if (!is_converged) {
				pcl::console::print_warn("[TESTBENCH] Result not converge, target point cloud %d from %s\n", i, pcd_file_paths[i]);
			}
			pcl::transformPointCloud(clouds_tgt[i], reg_cloud, final_transform.inverse());
			clouds_reg.push_back(reg_cloud);
			clouds_reg_ptr.push_back(clouds_reg.back().makeShared());
			results[i].is_converged = is_converged;
			results[i].final_transformation = final_transform;
			results[i].cloud_reg_ptr = clouds_reg_ptr.back();
		}
	}

	/**
	* @brief
	*
	*/
	bool isVizStopped() {
		return visualizer_ptr->wasStopped();
	}

	/**
	* @brief
	*
	*/
	bool runTestBench() {
		preprocessPointCloud();
		vizSrcTgt();
		doAlignAll();
		computeResult();
		vizResult();
		return true;
	}


protected:
	std::string reg_algorithm_name;

	pcl::PointCloud<PointType> cloud_src, cloud_src_filtered;
	boost::shared_ptr<pcl::PointCloud<PointType>> cloud_src_ptr, cloud_src_filtered_ptr;

	std::vector<pcl::PointCloud<PointType>> clouds_tgt, clouds_tgt_filtered;
	std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> clouds_tgt_ptr, clouds_tgt_filtered_ptr;

	std::vector<pcl::PointCloud<PointType>> clouds_reg;
	std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> clouds_reg_ptr;

private:
	typedef struct {
		bool is_converged = false;
		std::string pcd_file;
		boost::shared_ptr<pcl::PointCloud<PointType>> cloud_reg_ptr;
		Eigen::Matrix4f ref_transformation;
		Eigen::Matrix4f final_transformation;
		Eigen::Vector3d error_euler;
		Eigen::Vector3d error_trans;
	} Result;

	YAML::Node cfg_file;
	std::vector<std::string> pcd_file_paths;
	double grid_filter_leafsize;
	pcl::visualization::PCLVisualizer *visualizer_ptr;
	int view_port_1, view_port_2;

	std::vector<Result> results;

	/**
	* @brief
	*
	*/
	void loadSrcFromPCDFile(const std::string &pcd_path) {
		pcl::PointCloud<PointType> pcd;
		pcl::io::loadPCDFile(pcd_path, pcd);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(pcd, pcd, indices);
		setSrcPointCloud(pcd);
		pcl::console::print_info("[TESTBENCH] Load source point cloud: %s, size: %d\n", pcd_path.c_str(), pcd.size());
	}

	/**
	* @brief
	*
	*/
	void loadTgtFromPCDFile(const std::vector<std::string> &pcd_paths) {
		for (auto &path : pcd_paths) {
			pcl::PointCloud<PointType> pcd;
			pcl::io::loadPCDFile(path, pcd);
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(pcd, pcd, indices);
			addTgtPointCloud(pcd);
			results.emplace_back();
			results.back().pcd_file = path;
			pcl::console::print_info("[TESTBENCH] Load target point cloud: %s, size: %d\n", path.c_str(), pcd.size());
		}
	}

	/**
	* @brief
	*
	*/
	void setSrcPointCloud(const pcl::PointCloud<PointType> &pcd) {
		cloud_src = pcd;
		cloud_src_ptr = cloud_src.makeShared();
	}

	/**
	* @brief 
	*
	*/
	void addTgtPointCloud(const pcl::PointCloud<PointType> &pcd) {
		clouds_tgt.push_back(pcd);
		clouds_tgt_ptr.push_back(clouds_tgt.back().makeShared());
	}
};


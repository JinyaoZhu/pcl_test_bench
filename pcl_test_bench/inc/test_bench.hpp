#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

template <typename PointType>
class TestBench{

public:
	/**
	* @brief
	*
	*/
	TestBench() {
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
		ref_trans_path = cfg_file["ref_trans_path"].as<std::string>();
		has_ref_transformation = cfg_file["has_ref_transformation"].as<bool>();
		pcl::console::print_info("[TESTBENCH] Load parameters: grid_filter_leafsize = %f\n", grid_filter_leafsize);
		loadSrcFromPCDFile(pcd_file_paths.front());
		loadTgtFromPCDFile(std::vector<std::string>(pcd_file_paths.begin() + 1, pcd_file_paths.end()));
		if(has_ref_transformation)
			loadRefTransformation(ref_trans_path);
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
	bool isVizStopped() {
		return visualizer_ptr->wasStopped();
	}

	/**
	* @brief
	*
	*/
	void runTestBench() {
		vizInit();
		preprocessPointCloud();
		vizSrcTgt<PointType>();
		doAlignAll();
		computeResult();
		pcl::console::print_info("\n[TESTBENCH] Press q to quit.\n");
		visualizer_ptr->spin();
		visualizer_ptr->close();
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
		Eigen::Vector3f error_euler;
		Eigen::Quaternionf error_quat;
		Eigen::Vector3f error_trans;
		double time_cost;
		int point_size_src;
		int point_size_tgt;
	} Result;

	YAML::Node cfg_file;
	std::vector<std::string> pcd_file_paths;
	std::string ref_trans_path;
	double grid_filter_leafsize;
	bool has_ref_transformation;

	pcl::visualization::PCLVisualizer *visualizer_ptr;
	int view_port_1, view_port_2;
	std::vector<vtkSmartPointer<vtkDataArray>> clouds_tgt_color;


	std::vector<Result> results;
	pcl::StopWatch stop_watch;

	/**
	* @brief
	*
	*/
	void loadSrcFromPCDFile(const std::string &pcd_path) {
		pcl::PointCloud<PointType> pcd;
		if (pcl::io::loadPCDFile(pcd_path, pcd) == -1) {
			pcl::console::print_error("[TESTBENCH] Error loading PCD file, please check config file.\n");
			stopProcess();
		}
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
			if (pcl::io::loadPCDFile(path, pcd) == -1) {
				pcl::console::print_error("[TESTBENCH] Error loading PCD file, please check config file.\n");
				stopProcess();
			}
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
	void loadRefTransformation(std::string path) {
		std::ifstream fin(path);
		YAML::Node doc = YAML::Load(fin);
		std::cout << "[TESTBENCH] Loading reference transformations ..." << "\n";
		if (doc.size() != clouds_tgt.size()) {
			pcl::console::print_error(
			"[TESTBENCH] reference transformations size not equal to target pointclouds size. (%d != %d)\n",
				doc.size(), clouds_tgt.size());
			stopProcess();
		}
		for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
			static int idx = 0;
			Eigen::Matrix4f ref_transformation = it->second.as<Eigen::Matrix4f>();
			results[idx].ref_transformation = ref_transformation;
			//std::cout << "[TESTBENCH] Reference transformation " << std::to_string(idx+1) << "\n";
			//std::cout << ref_transformation << "\n\n";
			idx++;
		}
		fin.close();
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
			Result &result = results[i];
			Eigen::Matrix4f final_trans = result.final_transformation;
			Eigen::Matrix4f ref_translation = result.ref_transformation;
			Eigen::Matrix4f error_transformation = ref_translation * final_trans.inverse();
			Eigen::Vector3f error_euler, error_trans;
			Eigen::Quaternionf error_q(error_transformation.block<3, 3>(0, 0));
			//error_euler = ref_translation.block<3, 3>(0, 0).eulerAngles(2, 1, 0) - final_trans.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
			//warpErrorEuler(error_euler);
			//error_euler *= (180 / M_PI);
			//results[i].error_euler = error_euler;
			result.error_quat = error_q;
			result.error_trans = error_trans = error_transformation.block<3, 1>(0, 3);

			Eigen::Matrix4f T = final_trans;
			pcl::console::print_info("\n=============ESTIMATED TRANSFORMATION %d=============\n", i + 1);
			pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", T(0, 0), T(0, 1), T(0, 2));
			pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", T(1, 0), T(1, 1), T(1, 2));
			pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", T(2, 0), T(2, 1), T(2, 2));
			pcl::console::print_info("t = [ %0.3f, %0.3f, %0.3f ]\n", T(0, 3), T(1, 3), T(2, 3));
			//pcl::console::print_info("Error euler: [ %6.3f, %6.3f, %6.3f ] (deg)\n", error_euler(0), error_euler(1), error_euler(2));
			if (has_ref_transformation) {
				if (abs(error_q.w()) > 0.99)
					pcl::console::print_info("Error quaternion: [ x: %4.3f, y: %4.3f, z: %4.3f, w: %4.3f ]\n", error_q.x(), error_q.y(), error_q.z(), error_q.w());
				else
					pcl::console::print_warn("Error quaternion: [ x: %4.3f, y: %4.3f, z: %4.3f, w: %4.3f ]\n", error_q.x(), error_q.y(), error_q.z(), error_q.w());

				if (error_trans.norm() < 0.1)
					pcl::console::print_info("Error translation: [ %6.3f, %6.3f, %6.3f ] (m)\n", error_trans(0), error_trans(1), error_trans(2));
				else
					pcl::console::print_warn("Error translation: [ %6.3f, %6.3f, %6.3f ] (m)\n", error_trans(0), error_trans(1), error_trans(2));
			}
			if (result.is_converged)
				pcl::console::print_info("Has converged: %s\n", "true");
			else
				pcl::console::print_warn("Has converged: %s\n", "false");

			pcl::console::print_info("Time cost: %.4f (s)\n", result.time_cost);

			pcl::console::print_info("Point cloud size: src=%d, tgt=%d\n", result.point_size_src, result.point_size_tgt);

			pcl::console::print_info("PCD file: %s\n", result.pcd_file.c_str());
		}
	}

	void warpErrorEuler(Eigen::Vector3f& euler) {
		for (int i = 0; i < 3; ++i) {
			while (euler(i) > M_PI)
				euler(i) -= 2 * M_PI;
			while (euler(i) < -M_PI)
				euler(i) += 2 * M_PI;
		}
	}

	template<typename T>
	void vizSrcTgt() {

	};

	template<typename T>
	void vizResult() {

	};

	template <typename T>
	void vizResultAddOne(boost::shared_ptr<pcl::PointCloud<PointType>> tgt_ptr, int tgt_idx) {
	};

	/**
	* @brief
	*
	*/
	template <>
	void vizSrcTgt<pcl::PointXYZ>() {
		pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(cloud_src_ptr, 255, 0, 0);
		visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp1_source", view_port_1);

		for (auto& pcd_ptr : clouds_tgt_ptr) {
			static int idx = 0;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> tgt_h(pcd_ptr, 255 * rand(), 255 * rand(), 255 * rand());
			visualizer_ptr->addPointCloud(pcd_ptr, tgt_h, "vp1_target" + std::to_string(++idx), view_port_1);
			clouds_tgt_color.emplace_back();
			tgt_h.getColor(clouds_tgt_color.back());
		}

		visualizer_ptr->initCameraParameters();

		pcl::console::print_info("\n[TESTBENCH] Press q to begin the registration.\n");
		vizResultAddOne<PointType>(nullptr, 0); // show source in right
		visualizer_ptr->spin();
	}

	template <>
	void vizSrcTgt<pcl::PointXYZRGB>() {

		pcl::visualization::PointCloudColorHandlerRGBField<PointType> src_h(cloud_src_ptr);
		visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp1_source", view_port_1);

		for (auto& pcd_ptr : clouds_tgt_ptr) {
			static int idx = 0;
			pcl::visualization::PointCloudColorHandlerRGBField<PointType> tgt_h(pcd_ptr);
			visualizer_ptr->addPointCloud(pcd_ptr, tgt_h, "vp1_target" + std::to_string(++idx), view_port_1);
		}

		visualizer_ptr->initCameraParameters();

		pcl::console::print_info("\n[TESTBENCH] Press q to begin the registration.\n");
		vizResultAddOne<PointType>(nullptr, 0); // show source in right
		visualizer_ptr->spin();
	}

	template <>
	void vizResultAddOne<pcl::PointXYZ>(boost::shared_ptr<pcl::PointCloud<PointType>> tgt_ptr, int tgt_idx) {

		static bool is_first_loop = true;

		if (is_first_loop) {
			pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(cloud_src_ptr, 255, 0, 0);
			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);
			is_first_loop = false;
		}

		if (tgt_ptr == nullptr) {
			visualizer_ptr->spinOnce();
			return;
		}

		double r = clouds_tgt_color[tgt_idx]->GetComponent(0, 0);
		double g = clouds_tgt_color[tgt_idx]->GetComponent(0, 1);
		double b = clouds_tgt_color[tgt_idx]->GetComponent(0, 2);

		pcl::visualization::PointCloudColorHandlerCustom<PointType> tgt_h(tgt_ptr, r, g, b);

		visualizer_ptr->addPointCloud(tgt_ptr, tgt_h, "vp2_target" + std::to_string(tgt_idx), view_port_2);

		visualizer_ptr->spinOnce();
	}

	template <>
	void vizResultAddOne<pcl::PointXYZRGB>(boost::shared_ptr<pcl::PointCloud<PointType>> tgt_ptr, int tgt_idx) {

		static bool is_first_loop = true;

		if (is_first_loop) {
			pcl::visualization::PointCloudColorHandlerRGBField<PointType> src_h(cloud_src_ptr);
			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);
			is_first_loop = false;
		}

		if (tgt_ptr == nullptr) {
			visualizer_ptr->spinOnce();
			return;
		}

		pcl::visualization::PointCloudColorHandlerRGBField<PointType> tgt_h(tgt_ptr);

		visualizer_ptr->addPointCloud(tgt_ptr, tgt_h, "vp2_target" + std::to_string(tgt_idx), view_port_2);

		visualizer_ptr->spinOnce();
	}


	//template <>
	//void vizResult<pcl::PointXYZ>() {

	//		std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointType>> tgts_h;

	//		pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(cloud_src_ptr, 255, 0, 0);
	//		visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);

	//		for (auto& pcd_ptr : clouds_reg_ptr) {
	//			static int idx = 0;
	//			tgts_h.emplace_back(pcd_ptr, 0, 255, 0);
	//			visualizer_ptr->addPointCloud(pcd_ptr, tgts_h.back(), "vp2_target" + std::to_string(++idx), view_port_2);
	//		}

	//	pcl::console::print_info("\n[TESTBENCH] Press q to quit.\n");
	//	visualizer_ptr->spin();
	//}

	//template <>
	//void vizResult<pcl::PointXYZRGB>() {

	//		std::vector<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> tgts_h;

	//		pcl::visualization::PointCloudColorHandlerRGBField<PointType> src_h(cloud_src_ptr);

	//		visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);

	//		for (auto& pcd_ptr : clouds_reg_ptr) {
	//			static int idx = 0;
	//			tgts_h.emplace_back(pcd_ptr);
	//			visualizer_ptr->addPointCloud(pcd_ptr, tgts_h.back(), "vp2_target" + std::to_string(++idx), view_port_2);
	//		}

	//	pcl::console::print_info("\n[TESTBENCH] Press q to quit.\n");
	//	visualizer_ptr->spin();
	//}

	void stopProcess() {
		pcl::console::print_error("Please close window.\n");
		for (;;);
	}


	void vizInit() {
		visualizer_ptr = new pcl::visualization::PCLVisualizer();
		visualizer_ptr->setWindowName("TestBench");
		visualizer_ptr->createViewPort(0.0, 0, 0.5, 1.0, view_port_1);
		visualizer_ptr->createViewPort(0.5, 0, 1.0, 1.0, view_port_2);
	}


	/**
* @brief
*
*/
	void doAlignAll() {
		pcl::PointCloud<PointType> reg_cloud, reg_cloud_filtered;
		Eigen::Matrix4f final_transform;
		pcl::console::print_info("[TESTBENCH] Start registration...\n");
		for (int i = 0; i < clouds_tgt_filtered.size(); ++i) {
			pcl::console::print_info("[TESTBENCH] Registration %d start...\n", i + 1);

			stop_watch.reset();
			bool is_converged = doAlignOnce(cloud_src_filtered, clouds_tgt_filtered[i], reg_cloud_filtered, final_transform);
			double time_cost = stop_watch.getTimeSeconds();

			if (!is_converged) {
				pcl::console::print_warn("[TESTBENCH] Result not converge, target point cloud %d from %s\n", i + 1, pcd_file_paths[i+1].c_str());
			}
			pcl::transformPointCloud(clouds_tgt[i], reg_cloud, final_transform.inverse());
			clouds_reg.push_back(reg_cloud);
			clouds_reg_ptr.push_back(clouds_reg.back().makeShared());
			results[i].is_converged = is_converged;
			results[i].final_transformation = final_transform;
			results[i].cloud_reg_ptr = clouds_reg_ptr.back();
			results[i].time_cost = time_cost;
			results[i].point_size_src = cloud_src_filtered.size();
			results[i].point_size_tgt = clouds_tgt_filtered[i].size();

			vizResultAddOne<PointType>(clouds_reg_ptr.back(), i);
			pcl::console::print_info("[TESTBENCH] Registration %d finished, time cost: %.4f[s]\n", i + 1, time_cost);
		}
	}
};


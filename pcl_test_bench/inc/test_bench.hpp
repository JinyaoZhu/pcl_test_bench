/**
 * Test Bench framework for PCL registration algorithm
 * Author: J.Zhu
 * Date: 2017.1.7
 */
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
#include <thread>
#include <chrono>

template <typename PointType>
class TestBench
{
public:
	typedef boost::shared_ptr<TestBench<PointType>> Ptr;

	/**
	* \brief constructor indicate new class
	*/
	TestBench()
	{
		pcl::console::print_info("[TESTBENCH] Create new Test Bench.\n");
	}

	/**
	 * \brief load all configuration for the test bench and the candidate/tester
	 */
	void loadTestBenchConfig(const std::string path)
	{
		cfg_file_path = path;
		cfg_file = YAML::LoadFile(path);
		grid_filter_leafsize = cfg_file["grid_filter_leafsize"].as<float>();
		pcd_file_paths = cfg_file["pcd_files_path"].as<std::vector<std::string>>();
		result_output_path_root = cfg_file["result_output_path_root"].as<std::string>();
		ref_trans_path = cfg_file["ref_trans_path"].as<std::string>();
		has_ref_transformation = cfg_file["has_ref_transformation"].as<bool>();
		pcl::console::print_info("[TESTBENCH] Load parameters: grid_filter_leafsize = %f\n", grid_filter_leafsize);
		loadSrcFromPCDFile(pcd_file_paths.front());
		loadTgtFromPCDFile(std::vector<std::string>(pcd_file_paths.begin() + 1, pcd_file_paths.end()));
		if (has_ref_transformation)
			loadRefTransformation(ref_trans_path);
		loadCandidateConfig(); // candidate name load here
	}

	/**
	 * \brief if viewer is stoppped
	 */
	bool isVizStopped()
	{
		return visualizer_ptr->wasStopped();
	}

	/**
	 * \brief run the test bench
	 */
	void runTestBench()
	{
		preprocessPointCloud();
		vizInit();
		vizSrcTgt<PointType>();
		doAlignAll();
		computeResult();
		if (has_ref_transformation)
			saveResult();
		pcl::console::print_info("\n[TESTBENCH] Press q to quit.\n");
		visualizer_ptr->spin();
		visualizer_ptr->close();
	}


protected:
	YAML::Node cfg_file;
	std::string reg_candidate_name;

	pcl::PointCloud<PointType> cloud_src, cloud_src_filtered;
	boost::shared_ptr<pcl::PointCloud<PointType>> cloud_src_ptr, cloud_src_filtered_ptr;

	std::vector<pcl::PointCloud<PointType>> clouds_tgt, clouds_tgt_filtered;
	std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> clouds_tgt_ptr, clouds_tgt_filtered_ptr;

	std::vector<pcl::PointCloud<PointType>> clouds_reg;
	std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> clouds_reg_ptr;

	/**
	 * \brief load config for candidate, must be define by user(registration algorithm)
	 */
	virtual void loadCandidateConfig() = 0;

	/**
	 * \brief do registration for A pair of point cloud, must be defined by user
	 * \input source and target point cloud
	 * \return aligned point cloud and estimated transformation
	 */
	virtual bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud, const pcl::PointCloud<PointType> &tgt_cloud,
		pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f &transform) = 0;

	/**
	* \brief return fitness score of the alignment, define by user
	*/
	virtual double getFitness() = 0;

private:
	typedef struct
	{
		bool is_converged = false;
		std::string pcd_file;
		boost::shared_ptr<pcl::PointCloud<PointType>> cloud_reg_ptr;
		Eigen::Matrix4f ref_transformation;
		Eigen::Matrix4f final_transformation;
		Eigen::Vector3f error_euler;
		Eigen::Quaternionf error_quat;
		Eigen::Vector3f error_trans;
		double time_cost;
		double fitness;
		int point_size_src;
		int point_size_tgt;
	} Result;

	std::string cfg_file_path;
	std::vector<std::string> pcd_file_paths;
	std::string result_output_path;
	std::string result_output_path_root;
	std::string ref_trans_path;
	double grid_filter_leafsize;
	bool has_ref_transformation;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_ptr;
	int view_port_1, view_port_2;
	std::vector<vtkSmartPointer<vtkDataArray>> clouds_tgt_color;
	bool is_viz_result_first_loop = true;

	std::vector<Result> results; // all registration result
	pcl::StopWatch stop_watch;

	/**
	* \brief load source cloud
	*/
	void loadSrcFromPCDFile(const std::string &pcd_path)
	{
		pcl::PointCloud<PointType> pcd;
		if (pcl::io::loadPCDFile(pcd_path, pcd) == -1)
		{
			pcl::console::print_error("[TESTBENCH] Error loading PCD file, please check config file.\n");
			stopProcess();
		}
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(pcd, pcd, indices);
		setSrcPointCloud(pcd);
		pcl::console::print_info("[TESTBENCH] Load source point cloud: %s, size: %d\n", pcd_path.c_str(), pcd.size());
	}

	/**
	 * \brief load target clouds
	 */
	void loadTgtFromPCDFile(const std::vector<std::string> &pcd_paths)
	{
		for (auto &path : pcd_paths)
		{
			pcl::PointCloud<PointType> pcd;
			if (pcl::io::loadPCDFile(path, pcd) == -1)
			{
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
	* \brief load reference transformations
	*/
	void loadRefTransformation(std::string path)
	{
		std::ifstream fin(path);
		YAML::Node doc = YAML::Load(fin);
		std::cout << "[TESTBENCH] Loading reference transformations ..."
			<< "\n";
		if (doc.size() != clouds_tgt.size())
		{
			pcl::console::print_error(
				"[TESTBENCH] reference transformations size not equal to target pointclouds size. (%d != %d)\n",
				doc.size(), clouds_tgt.size());
			stopProcess();
		}

		int idx = 0;
		for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it, ++idx)
		{
			Eigen::Matrix4f ref_transformation = it->second.as<Eigen::Matrix4f>();
			results[idx].ref_transformation = ref_transformation;
			//std::cout << "[TESTBENCH] Reference transformation " << std::to_string(idx+1) << "\n";
			//std::cout << ref_transformation << "\n\n";
		}
		fin.close();
	}

	void setSrcPointCloud(const pcl::PointCloud<PointType> &pcd)
	{
		cloud_src = pcd;
		cloud_src_ptr = cloud_src.makeShared();
	}

	void addTgtPointCloud(const pcl::PointCloud<PointType> &pcd)
	{
		clouds_tgt.push_back(pcd);
		clouds_tgt_ptr.push_back(clouds_tgt.back().makeShared());
	}

	/**
	* \brief preprocessing / down sampling
	*/
	void preprocessPointCloud()
	{
		pcl::VoxelGrid<PointType> grid;
		grid.setLeafSize(grid_filter_leafsize, grid_filter_leafsize, grid_filter_leafsize);

		grid.setInputCloud(cloud_src_ptr);
		grid.filter(cloud_src_filtered);
		cloud_src_filtered_ptr = cloud_src_filtered.makeShared();
		pcl::console::print_info("[TESTBENCH] Filtered source point cloud size: %d / %d\n",
			cloud_src_filtered.size(), cloud_src.size());
		int idx = 1;
		for (auto &cloud_ptr : clouds_tgt_ptr)
		{
			grid.setInputCloud(cloud_ptr);
			clouds_tgt_filtered.emplace_back();
			grid.filter(clouds_tgt_filtered.back());
			clouds_tgt_filtered_ptr.push_back(clouds_tgt_filtered.back().makeShared());
			pcl::console::print_info("[TESTBENCH] Filtered target point cloud %d size: %d / %d\n",
				idx++, clouds_tgt_filtered.back().size(), cloud_ptr->size());
		}
	}

	/**
	* \brief compute result of the registrations and show console
	*/
	void computeResult()
	{
		pcl::console::print_info("\n[TESTBENCH] Algorithm: %s \n", reg_candidate_name.c_str());
		for (int i = 0; i < results.size(); ++i)
		{
			Result &result = results[i];
			Eigen::Matrix4f final_transformation = result.final_transformation;
			Eigen::Matrix4f ref_transformation = result.ref_transformation;
			Eigen::Matrix4f error_transformation = ref_transformation * final_transformation.inverse();
			Eigen::Vector3f error_trans;
			Eigen::Vector3f error_euler = 180.0 / M_PI * rot2EulerZYX(error_transformation.block<3, 3>(0, 0)); // deg
			Eigen::Quaternionf error_q(error_transformation.block<3, 3>(0, 0));
			results[i].error_euler = error_euler; // deg
			result.error_quat = error_q;
			result.error_trans = error_trans = error_transformation.block<3, 1>(0, 3);

			Eigen::Matrix4f T = final_transformation;
			pcl::console::print_info("\n=============ESTIMATED TRANSFORMATION %d=============\n", i + 1);
			pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", T(0, 0), T(0, 1), T(0, 2));
			pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", T(1, 0), T(1, 1), T(1, 2));
			pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", T(2, 0), T(2, 1), T(2, 2));
			pcl::console::print_info("t = [ %0.3f, %0.3f, %0.3f ]\n", T(0, 3), T(1, 3), T(2, 3));

			if (has_ref_transformation)
			{
				if (result.is_converged)
					pcl::console::print_info("Has converged: %s\n", "true");
				else
					pcl::console::print_warn("Has converged: %s\n", "false");

				// rotation error
				if (abs(error_q.w()) > 0.99) {
					pcl::console::print_info("Error quaternion: [ x: %4.3f, y: %4.3f, z: %4.3f, w: %4.3f ]\n", error_q.x(), error_q.y(), error_q.z(), error_q.w());
					pcl::console::print_info("Error euler: [ %6.3f, %6.3f, %6.3f ] (deg)\n", error_euler(0), error_euler(1), error_euler(2));
				}
				else {
					pcl::console::print_warn("Error quaternion: [ x: %4.3f, y: %4.3f, z: %4.3f, w: %4.3f ]\n", error_q.x(), error_q.y(), error_q.z(), error_q.w());
					pcl::console::print_warn("Error euler: [ %6.3f, %6.3f, %6.3f ] (deg)\n", error_euler(0), error_euler(1), error_euler(2));
				}

				// translation error
				if (error_trans.norm() < 0.1)
					pcl::console::print_info("Error translation: [ %6.3f, %6.3f, %6.3f ] (m)\n", error_trans(0), error_trans(1), error_trans(2));
				else
					pcl::console::print_warn("Error translation: [ %6.3f, %6.3f, %6.3f ] (m)\n", error_trans(0), error_trans(1), error_trans(2));
			}

			pcl::console::print_info("Fitness score: %.6f (m)\n", result.fitness);

			pcl::console::print_info("Time cost: %.4f (s)\n", result.time_cost);

			pcl::console::print_info("Point cloud size: src=%d, tgt=%d\n", result.point_size_src, result.point_size_tgt);

			pcl::console::print_info("PCD file: %s\n", result.pcd_file.c_str());
		}
	}

	/**
	* \brief rotation matrix to yaw-pitch-roll euler angle
	*        signular by pitch = +-pi/2
	*/
	Eigen::Vector3f rot2EulerZYX(Eigen::Matrix3f rot)
	{
		Eigen::Vector3f euler;
		euler(0) = asin(rot(0, 2));
		euler(1) = asin(rot(1, 2) / cos(euler(1)));
		euler(2) = asin(rot(0, 1) / cos(euler(1)));
		return euler;
	}

	/**
	* \brief show point cloud in left view port, show source and all targets
	*/
	template <typename T>
	void vizSrcTgt() {
		pcl::visualization::PointCloudColorHandlerCustom<T> src_h(cloud_src_ptr, 255, 0, 0);
		visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp1_source", view_port_1);

		int idx = 0;
		for (auto &pcd_ptr : clouds_tgt_ptr)
		{
			pcl::visualization::PointCloudColorHandlerCustom<T> tgt_h(pcd_ptr,
				255 * (rand() / (RAND_MAX + 1.0)), 255 * (rand() / (RAND_MAX + 1.0)), 255 * (rand() / (RAND_MAX + 1.0)));
			visualizer_ptr->addPointCloud(pcd_ptr, tgt_h, "vp1_target" + std::to_string(++idx), view_port_1);
			clouds_tgt_color.emplace_back();
			tgt_h.getColor(clouds_tgt_color.back());
		}

		visualizer_ptr->initCameraParameters();

		pcl::console::print_info("\n[TESTBENCH] Press q to begin the registration.\n");
		vizResultAddOne<T>(nullptr, 0); // show source in right
		visualizer_ptr->spin();
	}

	/**
	* \brief here will have compile error at gcc.
	*/
	template <>
	void vizSrcTgt<pcl::PointXYZRGB>()
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_h(cloud_src_ptr);
		visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp1_source", view_port_1);

		int idx = 0;
		for (auto &pcd_ptr : clouds_tgt_ptr)
		{
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h(pcd_ptr);
			visualizer_ptr->addPointCloud(pcd_ptr, tgt_h, "vp1_target" + std::to_string(++idx), view_port_1);
		}

		visualizer_ptr->initCameraParameters();

		pcl::console::print_info("\n[TESTBENCH] Press q to begin the registration.\n");
		vizResultAddOne<pcl::PointXYZRGB>(nullptr, 0); // show source in right
		visualizer_ptr->spin();
	}

	/**
	* \brief add point cloud to right view port and show
	*/
	template <typename T>
	void vizResultAddOne(boost::shared_ptr<pcl::PointCloud<PointType>> tgt_ptr, int tgt_idx) {
		if (is_viz_result_first_loop)
		{
			pcl::visualization::PointCloudColorHandlerCustom<T> src_h(cloud_src_ptr, 255, 0, 0);
			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);
			is_viz_result_first_loop = false;
		}

		if (tgt_ptr == nullptr)
		{
			visualizer_ptr->spinOnce();
			return;
		}

		double r = clouds_tgt_color[tgt_idx]->GetComponent(0, 0);
		double g = clouds_tgt_color[tgt_idx]->GetComponent(0, 1);
		double b = clouds_tgt_color[tgt_idx]->GetComponent(0, 2);

		pcl::visualization::PointCloudColorHandlerCustom<T> tgt_h(tgt_ptr, r, g, b);

		visualizer_ptr->addPointCloud(tgt_ptr, tgt_h, "vp2_target" + std::to_string(tgt_idx), view_port_2);

		visualizer_ptr->spinOnce();
	};

	/**
	* \brief compile error by gcc...
	*/
	template <>
	void vizResultAddOne<pcl::PointXYZRGB>(boost::shared_ptr<pcl::PointCloud<PointType>> tgt_ptr, int tgt_idx)
	{
		if (is_viz_result_first_loop)
		{
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_h(cloud_src_ptr);
			visualizer_ptr->addPointCloud(cloud_src_ptr, src_h, "vp2_source", view_port_2);
			is_viz_result_first_loop = false;
		}

		if (tgt_ptr == nullptr)
		{
			visualizer_ptr->spinOnce();
			return;
		}

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h(tgt_ptr);

		visualizer_ptr->addPointCloud(tgt_ptr, tgt_h, "vp2_target" + std::to_string(tgt_idx), view_port_2);

		visualizer_ptr->spinOnce();
	}

	void stopProcess()
	{
		pcl::console::print_error("Please close window.\n");
		for (;;)
			;
	}
	/**
	* \brief create a visualizer
	*/
	void vizInit()
	{
		visualizer_ptr = boost::make_shared<pcl::visualization::PCLVisualizer>(*(new pcl::visualization::PCLVisualizer));
		visualizer_ptr->setWindowName("TestBench");
		visualizer_ptr->createViewPort(0.0, 0, 0.5, 1.0, view_port_1);
		visualizer_ptr->createViewPort(0.5, 0, 1.0, 1.0, view_port_2);
	}

	/**
	* \brief align all target point clouds to the source.
	*/
	void doAlignAll()
	{
		pcl::PointCloud<PointType> reg_cloud, reg_cloud_filtered;
		Eigen::Matrix4f final_transform;
		pcl::console::print_info("[TESTBENCH] Start registration...\n");
		for (int i = 0; i < clouds_tgt_filtered.size(); ++i) {
			pcl::console::print_info("[TESTBENCH] Registration %d start...\n", i + 1);

			stop_watch.reset();
			bool is_converged = doAlignOnce(cloud_src_filtered, clouds_tgt_filtered[i], reg_cloud_filtered, final_transform);
			double time_cost = stop_watch.getTimeSeconds();

			if (!is_converged)
			{
				pcl::console::print_warn("[TESTBENCH] Result not converge, target point cloud %d from %s\n", i + 1, pcd_file_paths[i + 1].c_str());
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
			results[i].fitness = getFitness();

			vizResultAddOne<PointType>(clouds_reg_ptr.back(), i);
			pcl::console::print_highlight("[TESTBENCH] Registration %d finished, time cost: %.4f[s]\n", i + 1, time_cost);
		}
	}

	/**
	* \brief save all test results in csv file.
	*/
	void saveErrorResultsCsv(std::string path)
	{
		pcl::console::print_info("[TESTBENCH] Save result csv...\n");
		std::ofstream result_file(path + "resultError.csv", std::ios::out);
		result_file.setf(std::ios::fixed, std::ios::floatfield);
		//result_file.precision(0);
		result_file << reg_candidate_name << std::endl
			<< "name" << ","
			<< "error_yaw" << ","
			<< "error_pitch" << ","
			<< "error_roll" << ","
			<< "error_x" << ","
			<< "error_y" << ","
			<< "error_z" << ","
			<< "converge" << ","
			<< "time_cost" << ","
			<< "src_points" << ","
			<< "tgt_points" << ","
			<< "fitness" << ","
			<< "file_path" << ","
			<< std::endl;
		int idx = 1;
		for (auto &result : results) {
			result_file << "transformation_" + std::to_string(idx++) << ",";
			result_file.precision(6);
			result_file
				<< result.error_euler(0) << ","
				<< result.error_euler(1) << ","
				<< result.error_euler(2) << ","
				<< result.error_trans(0) << ","
				<< result.error_trans(1) << ","
				<< result.error_trans(2) << ","
				<< (result.is_converged ? "true" : "false") << ","
				<< result.time_cost << ","
				<< result.point_size_src << ","
				<< result.point_size_tgt << ","
				<< result.fitness << ","
				<< result.pcd_file << ","
				<< std::endl;
		}
		result_file.close();
	}

	/**
	* \brief save aligned point clouds in pcd files
	*/
	void saveRegPCD(std::string path) {
		pcl::console::print_info("[TESTBENCH] Save result pcd...\n");
		pcl::io::savePCDFileBinary(path + "source.pcd", cloud_src);
		int idx = 1;
		for (auto &result : results) {
			pcl::io::savePCDFileBinary(path + "reg_" + std::to_string(idx) + ".pcd", *result.cloud_reg_ptr);
			idx++;
		}
	}

	/**
	* \brief save all.
	*/
	void saveResult() {
		result_output_path = result_output_path_root + reg_candidate_name + "/";
		boost::filesystem::create_directories(boost::filesystem::path(result_output_path));
		saveErrorResultsCsv(result_output_path);
		saveRegPCD(result_output_path);
	}
};



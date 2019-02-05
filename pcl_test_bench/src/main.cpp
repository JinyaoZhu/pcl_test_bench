#include <iostream>
#include <string>
#include <ref_data_generator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <scp_test.hpp>
#include <icp_test.hpp>
#include <icpnl_test.hpp>
#include <gicp_test.hpp>
#include <ndt_test.hpp>

typedef pcl::PointXYZ PointType;

// simple point cloud viewer
void cloudViewer() {
	std::string point_cloud_path;
	std::cout << "Point cloud path: ";
	std::cin >> point_cloud_path;
	pcl::PointCloud<PointType> cloud;
	if (point_cloud_path.find(".pcd") != std::string::npos) {
		if (pcl::io::loadPCDFile(point_cloud_path, cloud) < 0) {
			return;
		}
	}
	else if (point_cloud_path.find(".ply") != std::string::npos) {
		if (pcl::io::loadPLYFile(point_cloud_path, cloud) < 0) {
			return;
		}
	}

	pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("Cloud Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud.makeShared(), 0, 0, 0);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), single_color, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	pcl::console::print_info("Press q to quit.");
	while (!viewer.wasStopped())
	{	
		viewer.spinOnce(100);
	}
	viewer.close();
}

// generate point clouds with reference transformation and nosie
void refCloudGenerator() {
	std::string cloud_path;
	std::string output_path;
	int num_trans;

	std::cout << "Target .pcd/.ply file(eg: bun0.pcd bun0.ply): ";
	std::cin >> cloud_path;
	while (cloud_path.find(".pcd") == std::string::npos && cloud_path.find(".ply") == std::string::npos) {
		std::cout << "Invalid file type." << "\n";
		std::cout << "Target .pcd/.ply file(eg: bun0.pcd bun0.ply): ";
		std::cin >> cloud_path;
	}

	std::cout << "Output path(eg: ted/ref/): ";
	std::cin >> output_path;
	while (output_path.find_last_of("/") != (output_path.size() - 1)) {
		std::cout << "Invalid output path, output path must be a folder(eg: ted/ref/)." << "\n";
		std::cout << "Output path(eg: ted/ref/): ";
		std::cin >> output_path;
	}

	std::cout << "Number of transformations: ";
	std::cin >> num_trans;
	ReferenceDataGenerator<PointType> data_gen;
	data_gen.generatePointCloudWithRef(cloud_path, output_path, num_trans);
}


int main(int argc, char **argv)
{
	std::string user_input;
	bool is_finished = false;
	pcl::console::setVerbosityLevel(pcl::console::L_INFO); // show debug messages

	do {
		std::cout << "\n========================PCL Test Bench=========================\n";
		std::cout << "[0] Cloud Viewer.\n";
		std::cout << "[1] Generate point clouds with reference transformations.\n";
		std::cout << "[2] Run Test Bench: SCP.\n";
		std::cout << "[3] Run Test Bench: ICP.\n";
		std::cout << "[4] Run Test Bench: ICP NL.\n";
		std::cout << "[5] Run Test Bench: GICP.\n";
		std::cout << "[6] Run Test Bench: NDT.\n";
		std::cout << "[q] Quit.\n";
		std::cout << "Your input: ";
		std::cin >> user_input;

		if (user_input == "0") {
			cloudViewer();
		}
		else if (user_input == "1") {
			refCloudGenerator();
		}
		else if (user_input == "2") {
			ScpTest<PointType> test_bench = ScpTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
		}
		else if (user_input == "3") {
			IcpTest<PointType> test_bench = IcpTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
		}
		else if (user_input == "4") {
			ICPNLTest<PointType> test_bench = ICPNLTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
		}
		else if (user_input == "5") {
			GICPTest<PointType> test_bench = GICPTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
		}
		else if (user_input == "6") {
			NDTTest<PointType> test_bench = NDTTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
		}
		else if (user_input == "q") {
			is_finished = true;
		}
		else {
			std::cout << "Invalid command.\n";
		}
	} while (!is_finished);
	
	return 0;
}
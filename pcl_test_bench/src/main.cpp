#include <iostream>
#include <string>
#include <conio.h>
#include <ref_data_generator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <scp_test.hpp>
#include <icp_test.hpp>
#include <icpnl_test.hpp>
#include <gicp_test.hpp>

typedef pcl::PointXYZ PointType;


void cloudViewer() {
	std::string point_cloud_path;
	std::cout << "Point cloud path: ";
	std::cin >> point_cloud_path;
	pcl::PointCloud<pcl::PointXYZ> cloud;
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
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud.makeShared());
	pcl::console::print_info("Press q to quit.");
	while (!viewer.wasStopped())
	{
	}
}

void refCloudGenerator() {
	std::string src_cloud_path;
	std::string output_path;
	int num_trans;

	std::cout << "Source .pcd/.ply file(eg: bun0.pcd bun0.ply): ";
	std::cin >> src_cloud_path;
	while (src_cloud_path.find(".pcd") == std::string::npos && src_cloud_path.find(".ply") == std::string::npos) {
		std::cout << "Invalid source file type." << "\n";
		std::cout << "Source .pcd/.ply file(eg: bun0.pcd bun0.ply): ";
		std::cin >> src_cloud_path;
	}

	std::cout << "Output path(eg: ref/): ";
	std::cin >> output_path;
	while (output_path.find("/") != (output_path.size() - 1)) {
		std::cout << "Invalid output path, output path must be a folder(eg: ref/)." << "\n";
		std::cout << "Output path(eg: ref/): ";
		std::cin >> output_path;
	}

	std::cout << "Number of transformations: ";
	std::cin >> num_trans;
	ReferenceDataGenerator<PointType> data_gen;
	data_gen.generatePointCloudWithRef(src_cloud_path, output_path, num_trans);
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
		else if (user_input == "q") {
			is_finished = true;
		}
		else {
			std::cout << "Invalid command.\n";
		}
	} while (!is_finished);
	
	//std::cout << "Press any key to continue ...\n";
	//_getch();
	return 0;
}
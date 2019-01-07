#include <iostream>
#include <string>
#include <conio.h>
#include <ref_data_generator.hpp>

#include <scp_test.hpp>
#include <icp_test.hpp>

typedef pcl::PointXYZ PointType;

int main(int argc, char **argv)
{
	std::string user_input;
	bool is_finished = false;
	pcl::console::setVerbosityLevel(pcl::console::L_INFO); // show debug messages

	do {
		std::cout << "\n========================PCL Test Bench=========================\n";
		std::cout << "[1] Generate point clouds with reference transformations.\n";
		std::cout << "[2] Run Test Bench: SCP.\n";
		std::cout << "[3] Run Test Bench: ICP.\n";
		std::cout << "[q] Quit.\n";
		std::cout << "Your input: ";
		std::cin >> user_input;

		if (user_input == "1") {
			std::string src_cloud_path;
			std::string output_path;
			int num_trans;
			std::cout << "Source PCD file(eg: bun0.pcd): ";
			std::cin >> src_cloud_path;
			std::cout << "Output path(eg: ref/): ";
			std::cin >> output_path;
			while (output_path.find("/") != (output_path.size() - 1)) {
				std::cout << "\x1B[31m Invalid output path, output path must be a folder(eg: ref/). \x1B[0m" << "\n";
				std::cout << "Output path(eg: ref/): ";
				std::cin >> output_path;
			}
			std::cout << "Number of transformations: ";
			std::cin >> num_trans;
			ReferenceDataGenerator<PointType> data_gen;
			data_gen.generatePointCloudWithRef(src_cloud_path, output_path, num_trans);
		}
		else if (user_input == "2") {
			ScpTest<PointType> test_bench = ScpTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
			//is_finished = true;
		}
		else if (user_input == "3") {
			IcpTest<PointType> test_bench = IcpTest<PointType>();
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
			//is_finished = true;
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
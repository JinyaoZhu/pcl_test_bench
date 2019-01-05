#include <yaml-cpp/yaml.h>
#include <scp_test_bench.hpp>
#include <vector>
#include <ref_data_generator.hpp>
#include <conio.h>


typedef pcl::PointXYZ PointType;

int main(int argc, char **argv)
{
	std::string user_input;
	bool is_finished = false;
	pcl::console::setVerbosityLevel(pcl::console::L_INFO); // show debug messages

	
	do {
		std::cout << "[1] Generate point clouds with reference transformations.\n";
		std::cout << "[2] Run Test Bench.\n";
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
			if (output_path.find("/") != (output_path.size() - 1)) {
				std::cout << "Error output path.\n";
				continue;
			}
			std::cout << "Number of transformations: ";
			std::cin >> num_trans;
			ReferenceDataGenerator<PointType> data_gen;
			data_gen.generatePointCloudWithRef(src_cloud_path, output_path, num_trans);
			is_finished = true;
		}
		else if (user_input == "2") {
			ScpTestBench<PointType> test_bench;
			test_bench.loadScpConfig("cfg/algorithm_cfg.yaml");
			test_bench.loadTestBenchConfig("cfg/testbench_cfg.yaml");
			test_bench.runTestBench();
			is_finished = true;
		}
	} while (!is_finished);
	
	std::cout << "Press any key to continue ...\n";
	_getch();
	return 0;
}

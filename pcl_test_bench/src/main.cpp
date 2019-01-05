#include <yaml-cpp/yaml.h>
#include <scp_test_bench.hpp>
#include <vector>
#include <ref_data_generator.hpp>


typedef pcl::PointXYZ PointType;

int main(int argc, char **argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_INFO); // show debug messages

	ReferenceDataGenerator<PointType> data_gen;
	data_gen.generatePointCloudWithRef("pcd/coffee_mug_1_1_68.pcd", "ref_data/",5);

	ScpTestBench<PointType> test_bench;
	test_bench.loadTestBenchConfig("config/test_bench_cfg.yaml");
	test_bench.loadScpConfig("config/scp_cfg.yaml");
	test_bench.runTestBench();
	
	for (;!test_bench.isVizStopped(););
	
	return 0;
}

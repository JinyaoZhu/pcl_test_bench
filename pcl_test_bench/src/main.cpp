#include <limits>
#include <type_traits>
#include <yaml-cpp/yaml.h>
#include <scp_test_bench.hpp>
#include <vector>

void generatePointCloudWithRef(std::string pcd_path);

int main(int argc, char **argv)
{
	pcl::console::setVerbosityLevel(pcl::console::L_INFO); // show debug messages

	//ScpTestBench<pcl::PointXYZ> test_bench;
	//test_bench.loadTestBenchConfig("config/test_bench_cfg.yaml");
	//test_bench.loadScpConfig("config/scp_cfg.yaml");
	//test_bench.runTestBench();
	//
	//for (;!test_bench.isVizStopped(););
	
	generatePointCloudWithRef("pcd/coffee_mug_1_1_1.pcd");

	return 0;
}


namespace YAML {
	template<>
	struct convert<Eigen::Matrix4f> {
		static Node encode(const Eigen::Matrix4f& mat) {
			Node node;
			node.push_back(mat(0, 0)); node.push_back(mat(0, 1)); node.push_back(mat(0, 2)); node.push_back(mat(0, 3));
			node.push_back(mat(1, 0)); node.push_back(mat(1, 1)); node.push_back(mat(1, 2)); node.push_back(mat(1, 3));
			node.push_back(mat(2, 0)); node.push_back(mat(2, 1)); node.push_back(mat(2, 2)); node.push_back(mat(2, 3));
			node.push_back(mat(3, 0)); node.push_back(mat(3, 1)); node.push_back(mat(3, 2)); node.push_back(mat(3, 3));
			return node;
		}

		static bool decode(const Node& node, Eigen::Matrix4f& mat) {
			if (!node.IsSequence() || node.size() != 16) {
				return false;
			}

			mat(0, 0) = node[0].as<double>(); mat(0, 1) = node[1].as<double>(); mat(0, 2) = node[2].as<double>(); mat(0, 3) = node[3].as<double>();
			mat(1, 0) = node[4].as<double>(); mat(1, 1) = node[5].as<double>(); mat(1, 2) = node[6].as<double>(); mat(1, 3) = node[7].as<double>();
			mat(2, 0) = node[8].as<double>(); mat(2, 1) = node[9].as<double>(); mat(2, 2) = node[10].as<double>(); mat(2, 3) = node[11].as<double>();
			mat(3, 0) = node[12].as<double>(); mat(3, 1) = node[13].as<double>(); mat(3, 2) = node[14].as<double>(); mat(3, 3) = node[15].as<double>();
			return true;
		}
	};
}


void generatePointCloudWithRef(std::string pcd_path) {

	pcl::PointCloud<pcl::PointXYZ> cloud_source;

	pcl::io::loadPCDFile(pcd_path, cloud_source);

	std::ofstream fout("ref_trans.yaml");

	YAML::Node ref_trans = YAML::Load("");

	int N = 3;

	for (int i = 0; i < N; ++i) {
		// Transform the source cloud by a large amount
		Eigen::Vector3f t(0.05, 0.02, 0.1);
		float angle = 0.0*static_cast<float>(M_PI) / 4.0f;
		Eigen::Quaternionf q(cos(angle / 2), 0, 0, sin(angle / 2));
		pcl::PointCloud<pcl::PointXYZ> cloud_source_transformed;
		Eigen::Matrix4f transform;
		transform.block<3, 3>(0, 0) = q.toRotationMatrix();
		transform.block<3, 1>(0, 3) = t;
		transform(3, 3) = 1;
		pcl::transformPointCloud(cloud_source, cloud_source_transformed, transform);
		std::cout << "T"<< i << "=\n" << transform << std::endl;
	
		std::string node_name = "transformation_" + std::to_string(i);
		std::string pcd_name = "trans_" + std::to_string(i) + ".pcd";

		pcl::io::savePCDFileASCII(pcd_name, cloud_source_transformed);
		ref_trans[node_name.c_str()] = transform;
	}

	//fout << out.c_str();
	fout << ref_trans;
	fout.close();

	std::cout << "\n\n";

	std::ifstream fin("ref_trans.yaml");
	YAML::Node doc = YAML::Load(fin);


	//YAML::Node ref_trans_load = YAML::LoadFile("reference_transformations.yaml");
	//YAML::Node ref_trans = YAML::Load("{1B: Prince Fielder, 2B: Rickie Weeks, LF: Ryan Braun}");
	for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
		/*std::cout << "!!!" << "\n";*/
		Eigen::Matrix4f ref_transformation = it->second.as<Eigen::Matrix4f>();
		std::cout << ref_transformation << "\n\n";
	}
}
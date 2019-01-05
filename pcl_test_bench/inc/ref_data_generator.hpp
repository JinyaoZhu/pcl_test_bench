#pragma	once
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>

namespace YAML {
	template<>
	struct convert<Eigen::Matrix4f> {
		static Node encode(const Eigen::Matrix4f& mat) {
			Node node;
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j)
					node.push_back(mat(i, j));
			return node;
		}

		static bool decode(const Node& node, Eigen::Matrix4f& mat) {
			if (!node.IsSequence() || node.size() != 16) {
				return false;
			}
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j)
					mat(i, j) = node[i * 4 + j].as<float>();
			return true;
		}
	};
}

template <typename PointType>
class ReferenceDataGenerator {
public:
	ReferenceDataGenerator() {};
	~ReferenceDataGenerator() {};

	void generatePointCloudWithRef(std::string pcd_path, std::string output_path, int num_pcd) {

		srand(time(NULL));

		pcl::PointCloud<PointType> cloud_source;

		pcl::io::loadPCDFile(pcd_path, cloud_source);

		PointType centroid;
		pcl::computeCentroid(cloud_source, centroid);

		for (auto &point : cloud_source) {
			point.x -= centroid.x;
			point.y -= centroid.y;
			point.z -= centroid.z;
		}

		boost::filesystem::path dir(output_path);

		if (!(boost::filesystem::exists(dir))) {
			std::cout << output_path << " doesn't exists." << std::endl;

			if (boost::filesystem::create_directory(dir))
				std::cout << output_path << " successfully created." << std::endl;
		}

		std::ofstream fout(output_path + "ref_trans.yaml");

		YAML::Node ref_trans = YAML::Load("");

		double max_translation = 0.2; // [m]

		pcl::io::savePCDFileASCII(output_path+"trans_0.pcd", cloud_source);

		for (int i = 0; i < num_pcd; ++i) {
			Eigen::Vector3f t = max_translation * Eigen::Vector3f::Random();
			Eigen::Quaternionf q = Eigen::Quaternionf::UnitRandom();
			pcl::PointCloud<PointType> cloud_source_transformed;
			Eigen::Matrix4f transform;
			transform.block<3, 3>(0, 0) = q.toRotationMatrix();
			transform.block<3, 1>(0, 3) = t;
			transform(3, 3) = 1;
			pcl::transformPointCloud(cloud_source, cloud_source_transformed, transform);
			std::cout << "Generate " <<"transformation" << i+1 << "=\n" << transform << "\n\n";

			std::string node_name = "transformation_" + std::to_string(i + 1);
			std::string pcd_name = output_path + "trans_" + std::to_string(i + 1) + ".pcd";

			pcl::io::savePCDFileASCII(pcd_name, cloud_source_transformed);
			ref_trans[node_name.c_str()] = transform;
		}

		fout << ref_trans;
		fout.close();
	}
};
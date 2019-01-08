#pragma	once
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <random>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

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

	bool generatePointCloudWithRef(std::string pcd_path, std::string output_path, int num_pcd) {

		srand(time(NULL));

		pcl::PointCloud<PointType> cloud_source;

		if (pcl::io::loadPCDFile(pcd_path, cloud_source) == -1) {
			return false;
		}

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

		double max_translation; // [m]
		double max_angle; // [deg]
		std::string add_noise;

		std::cout << "Maximal translation(m): ";
		std::cin >> max_translation;

		std::cout << "Maximal rotation angle(deg): ";
		std::cin >> max_angle;

		pcl::io::savePCDFileASCII(output_path+"trans_0.pcd", cloud_source);

		std::cout << "Add noise to point clouds?(y/n):";
		std::cin >> add_noise;

		for (int i = 0; i < num_pcd; ++i) {
			Eigen::Vector3f t = max_translation * Eigen::Vector3f::Random();
			Eigen::Vector3f euler = (M_PI/180)*max_angle * Eigen::Vector3f::Random();
			Eigen::Quaternionf q(Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()));
			pcl::PointCloud<PointType> cloud_source_transformed;

			pcl::transformPointCloud(cloud_source, cloud_source_transformed, Eigen::Vector3f::Zero(), q);

			if (add_noise == "y") {
				std::cout << "Add noise ... \n";
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				pcl::ExtractIndices<PointType> extract;
				int quadrant = static_cast<int>(8.0*rand() / (RAND_MAX + 1.0)) + 1;
				for (int i = 0; i < cloud_source_transformed.size(); i++)
				{
					PointType& point = cloud_source_transformed.points[i];
					bool should_remove = (point.x > 0) && (point.y > 0) && (point.z > 0);

					switch (quadrant) {
					case 1: should_remove = (point.x > 0) && (point.y > 0) && (point.z > 0); break;
					case 2: should_remove = (point.x < 0) && (point.y > 0) && (point.z > 0); break;
					case 3: should_remove = (point.x < 0) && (point.y < 0) && (point.z > 0); break;
					case 4: should_remove = (point.x > 0) && (point.y < 0) && (point.z > 0); break;
					case 5: should_remove = (point.x > 0) && (point.y > 0) && (point.z < 0); break;
					case 6: should_remove = (point.x < 0) && (point.y > 0) && (point.z < 0); break;
					case 7: should_remove = (point.x < 0) && (point.y < 0) && (point.z < 0); break;
					case 8: should_remove = (point.x > 0) && (point.y < 0) && (point.z < 0); break;
					default:break;
					}

					if (should_remove)
						inliers->indices.push_back(i);
					else {
						std::default_random_engine generator;
						std::normal_distribution<double> distribution(0, 0.003);
						point.x += distribution(generator);
						point.y += distribution(generator);
						point.z += distribution(generator);
					}
				}
				extract.setInputCloud(cloud_source_transformed.makeShared());
				extract.setIndices(inliers);
				extract.setNegative(true);
				extract.filter(cloud_source_transformed);
			}

			pcl::transformPointCloud(cloud_source_transformed, cloud_source_transformed, t, Eigen::Quaternionf::Identity());

			//pcl::transformPointCloud(cloud_source, cloud_source_transformed, transform);
			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			transform.block<3, 3>(0, 0) = q.toRotationMatrix();
			transform.block<3, 1>(0, 3) = t;
			std::cout << "Generate " <<"transformation" << i+1 << "=\n" << transform << "\n\n";

			std::string node_name = "transformation_" + std::to_string(i + 1);
			std::string pcd_name = output_path + "trans_" + std::to_string(i + 1) + ".pcd";

			pcl::io::savePCDFileASCII(pcd_name, cloud_source_transformed);
			ref_trans[node_name.c_str()] = transform;
		}

		fout << ref_trans;
		fout.close();

		return true;
	}
};
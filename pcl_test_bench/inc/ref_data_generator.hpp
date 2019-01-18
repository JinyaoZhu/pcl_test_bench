/**
* Point cloud generator with reference transformations and artifical noise
*/
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
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>

namespace YAML {
	// explicit specialization for Eigen::Matrix4f
	// used when load reference transformations
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
		pcl::PointCloud<PointType> cloud_source;

		// update seed
		srand(time(NULL));

		if (pcd_path.find(".pcd") != std::string::npos) {
			if (pcl::io::loadPCDFile(pcd_path, cloud_source) < 0) {
				return false;
			}
		}
		else if (pcd_path.find(".ply") != std::string::npos) {
			if (pcl::io::loadPLYFile(pcd_path, cloud_source) < 0) {
				return false;
			}
		}

		PointType centroid;
		pcl::computeCentroid(cloud_source, centroid);
		// move the point cloud to the origin of the coordinate system
		for (auto &point : cloud_source) {
			point.x -= centroid.x;
			point.y -= centroid.y;
			point.z -= centroid.z;
		}

		boost::filesystem::path dir(output_path);
		boost::filesystem::create_directories(dir);

		std::ofstream ref_trans_fout(output_path + "ref_trans.yaml");
		std::ofstream parameter_fout(output_path + "info.yaml");

		YAML::Node ref_trans = YAML::Load("");
		YAML::Node parameter = YAML::Load("");

		double max_translation; // [m]
		double max_angle; // [deg]
		std::string add_noise;

		std::cout << "Maximal translation(m): ";
		std::cin >> max_translation;

		std::cout << "Maximal rotation angle(deg): ";
		std::cin >> max_angle;

		parameter["max_translation(m)"] = max_translation;
		parameter["max_rotation(deg)"] = max_angle;
		
		std::cout << "Add noise to point clouds?(y/n):";
		std::cin >> add_noise;

		double noise_std = 0;//[m]
		if (add_noise == "y") {
			std::cout << "noise std(m):";
			std::cin >> noise_std;
			parameter["noise_std(m)"] = noise_std;
		}

		pcl::io::savePCDFileBinary(output_path + "trans_0.pcd", cloud_source);
		parameter["output_file"].push_back(output_path + "trans_0.pcd");
		for (int i = 0; i < num_pcd; ++i) {
			Eigen::Vector3f t = max_translation * Eigen::Vector3f::Random();
			Eigen::Vector3f euler = (M_PI/180)*max_angle * Eigen::Vector3f::Random();
			Eigen::Quaternionf q(Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()));
			pcl::PointCloud<PointType> cloud_source_transformed;

			// rotate the source point cloud
			pcl::transformPointCloud(cloud_source, cloud_source_transformed, Eigen::Vector3f::Zero(), q);

			if (add_noise == "y") {
				std::cout << "Add noise ... \n";
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				pcl::ExtractIndices<PointType> extract;
				// randomly sample a quadrant
				int quadrant = static_cast<int>(8.0*rand() / (RAND_MAX + 1.0)) + 1;
				for (int i = 0; i < cloud_source_transformed.size(); i++)
				{
					PointType& point = cloud_source_transformed.points[i];
					bool should_remove = (point.x > 0) && (point.y > 0) && (point.z > 0);

					// randomly mask a quadrant
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
						// add gaussian noise
						std::default_random_engine generator;
						std::normal_distribution<double> distribution(0, noise_std);
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

			// move the source point cloud
			pcl::transformPointCloud(cloud_source_transformed, cloud_source_transformed, t, Eigen::Quaternionf::Identity());

			//pcl::transformPointCloud(cloud_source, cloud_source_transformed, transform);
			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			transform.block<3, 3>(0, 0) = q.toRotationMatrix();
			transform.block<3, 1>(0, 3) = t;
			std::cout << "Generate " <<"transformation" << i+1 << "=\n" << transform << "\n\n";

			std::string node_name = "transformation_" + std::to_string(i + 1);
			std::string pcd_name = output_path + "trans_" + std::to_string(i + 1) + ".pcd";

			pcl::io::savePCDFileBinary(pcd_name, cloud_source_transformed);
			ref_trans[node_name.c_str()] = transform;
			parameter["output_file"].push_back(pcd_name);
		}
		// save reference transformations in yaml
		ref_trans_fout << ref_trans;
		ref_trans_fout.close();
		// save info.yaml
		parameter_fout << parameter;
		parameter_fout.close();
		return true;
	}
};
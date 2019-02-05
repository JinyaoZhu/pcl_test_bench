/**
* Point cloud generator with reference transformations and artifical noise and mask
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
		pcl::PointCloud<PointType> cloud_target;

		// update seed
		srand(time(NULL));

		if (pcd_path.find(".pcd") != std::string::npos) {
			if (pcl::io::loadPCDFile(pcd_path, cloud_target) < 0) {
				return false;
			}
		}
		else if (pcd_path.find(".ply") != std::string::npos) {
			if (pcl::io::loadPLYFile(pcd_path, cloud_target) < 0) {
				return false;
			}
		}

		PointType centroid;
		pcl::computeCentroid(cloud_target, centroid);
		// move the point cloud to the origin of the coordinate system
		for (auto &point : cloud_target) {
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

		parameter["input file"] = pcd_path;


		double max_translation_x; // [m]
		double max_translation_y; // [m]
		double max_translation_z; // [m]

		double max_angle_yaw; // [deg]
		double max_angle_pitch; // [deg]
		double max_angle_roll; // [deg]
		std::string add_noise;

		std::cout << "Maximal translation x(m): ";
		std::cin >> max_translation_x;
		std::cout << "Maximal translation y(m): ";
		std::cin >> max_translation_y;
		std::cout << "Maximal translation z(m): ";
		std::cin >> max_translation_z;

		std::cout << "Maximal rotation angle yaw(deg): ";
		std::cin >> max_angle_yaw;
		std::cout << "Maximal rotation angle pitch(deg): ";
		std::cin >> max_angle_pitch;
		std::cout << "Maximal rotation angle roll(deg): ";
		std::cin >> max_angle_roll;

		parameter["max_translation_x(m)"] = max_translation_x;
		parameter["max_translation_y(m)"] = max_translation_y;
		parameter["max_translation_z(m)"] = max_translation_z;
		parameter["max_rotation_yaw(deg)"] = max_angle_yaw;
		parameter["max_rotation_pitch(deg)"] = max_angle_pitch;
		parameter["max_rotation_roll(deg)"] = max_angle_roll;
		
		std::cout << "Add noise to point clouds?(y/n):";
		std::cin >> add_noise;

		double noise_std = 0;//[m]
		if (add_noise == "y") {
			std::cout << "noise std(m):";
			std::cin >> noise_std;
			parameter["Add noise"] = "true";
			parameter["noise_std(m)"] = noise_std;
		}
		else {
			parameter["Add noise"] = "false";
		}

		pcl::io::savePCDFileBinary(output_path + "trans_0.pcd", cloud_target);
		parameter["output_file"].push_back(output_path + "trans_0.pcd");
		for (int i = 0; i < num_pcd; ++i) {
			Eigen::Vector3f t = Eigen::Vector3f(max_translation_x, max_translation_y, max_translation_z).cwiseProduct(Eigen::Vector3f::Random());
			Eigen::Vector3f euler = (M_PI/180)*
			  Eigen::Vector3f(max_angle_yaw, max_angle_pitch, max_angle_roll).cwiseProduct(Eigen::Vector3f::Random());
			Eigen::Quaternionf q(Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()));
			pcl::PointCloud<PointType> cloud_target_transformed;

			// rotate the target point cloud
			pcl::transformPointCloud(cloud_target, cloud_target_transformed, Eigen::Vector3f::Zero(), q);

			if (add_noise == "y") {
				std::cout << "Add noise ... \n";
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				pcl::ExtractIndices<PointType> extract;
				// randomly sample a quadrant
				int quadrant = static_cast<int>(8.0*rand() / (RAND_MAX + 1.0)) + 1;
				for (int i = 0; i < cloud_target_transformed.size(); i++)
				{
					PointType& point = cloud_target_transformed.points[i];
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
				extract.setInputCloud(cloud_target_transformed.makeShared());
				extract.setIndices(inliers);
				extract.setNegative(true);
				extract.filter(cloud_target_transformed);
			}

			// move the source point cloud
			pcl::transformPointCloud(cloud_target_transformed, cloud_target_transformed, t, Eigen::Quaternionf::Identity());

			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			transform.block<3, 3>(0, 0) = q.toRotationMatrix();
			transform.block<3, 1>(0, 3) = t;
			// p_target = T*p_source
			transform = transform.inverse();
			std::cout << "Generate " <<"transformation" << i+1 << "=\n" << transform << "\n\n";

			std::string node_name = "transformation_" + std::to_string(i + 1);
			std::string pcd_name = output_path + "trans_" + std::to_string(i + 1) + ".pcd";

			pcl::io::savePCDFileBinary(pcd_name, cloud_target_transformed);
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
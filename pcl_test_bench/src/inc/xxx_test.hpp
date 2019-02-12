/**
 * A template for Tester
 * 1. change the class name (line 19)
 * 2. change alg_name (line 25)
 * 3. change config file path's name (line 36)
 * 4. fill the fucntions (line 39, 45, 51)
 * 5. initialize the class in main.cpp and run testbench
 */
#pragma once
#include <iostream>
#include <string>
#include <test_bench.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <typename PointType>
class TemplateTest : public TestBench<PointType>
{
  public:
    /**
     * \brief fill your algorithm name behind alg_name
     */
    TemplateTest() : alg_name("Template algorithm")
    {
        /*************start your code below*************/
    }

    void loadCandidateConfig()
    {
        YAML::Node alg_cfg;
        TestBench<PointType>::reg_candidate_name = alg_name;
        /* change the name of the parameter for the config file path, 
        don't forget add path of the config in testbench_cfg.yaml */
        std::string path = TestBench<PointType>::cfg_file["xxx_cfg_path"].as<std::string>();
        alg_cfg = YAML::LoadFile(path);

        /*************start your code below*************/
    }

    bool doAlignOnce(const pcl::PointCloud<PointType> &src_cloud, const pcl::PointCloud<PointType> &tgt_cloud,
                     pcl::PointCloud<PointType> &reg_cloud, Eigen::Matrix4f &transform)
    {
        /*************start your code below*************/

        return true; // only return true if the registration is converge
    }

    double getFitness() {
        /*************start your code below*************/
        // return fitness score of this registration
		return 0.0;
	}

  private:
    std::string alg_name; // algorithm name, this will pass to the test bench
};
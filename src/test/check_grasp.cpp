#include "o2ac_pose_distribution_updater/planner.hpp"
#include "o2ac_pose_distribution_updater/read_stl.hpp"
#include <matheval.h>
#include <random>
#include <yaml-cpp/yaml.h>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

void load_grasp_points(const std::string &yaml_file_path,
                       std::vector<Eigen::Isometry3d> &grasp_points,
                       std::vector<std::string> &names) {
  char *variable_names[] = {(char *)"pi"};
  double variable_values[] = {acos(-1)};

  YAML::Node config = YAML::LoadFile(yaml_file_path);
  const YAML::Node &grasp_points_data = config["grasp_points"];
  grasp_points.resize(grasp_points_data.size());
  names.resize(grasp_points_data.size());
  for (int i = 0; i < grasp_points.size(); i++) {
    std::string grasp_name =
        grasp_points_data[i]["grasp_name"].as<std::string>();
    names[i] = grasp_name;
    const YAML::Node &grasp_point_pose = grasp_points_data[i]["pose_xyzrpy"];
    Particle particle;
    for (int j = 0; j < 6; j++) {
      std::string expression = grasp_point_pose[j].as<std::string>();
      void *evaluator = evaluator_create((char *)expression.c_str());
      particle[j] =
          evaluator_evaluate(evaluator, 1, variable_names, variable_values);
    }
    grasp_points[i] = particle_to_eigen_transform(particle);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "planner_test");
  ros::NodeHandle nd;

  int argi = 1;
  std::string stl_file_path(argv[argi++]), metadata_file_path(argv[argi++]);

  std::shared_ptr<mesh_object> gripped_geometry(new mesh_object);
  read_stl_from_file_path("/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/"
                          "config/wrs_assembly_2020/meshes/" +
                              std::string(stl_file_path),
                          gripped_geometry->vertices,
                          gripped_geometry->triangles);
  for (auto &vertex : gripped_geometry->vertices) {
    vertex /= 1000.0; // milimeter -> meter
  }
  std::shared_ptr<std::vector<Eigen::Isometry3d>> grasp_points(
      new std::vector<Eigen::Isometry3d>);
  std::vector<std::string> names;
  load_grasp_points("/root/o2ac-ur/catkin_ws/src/o2ac_assembly_database/config/"
                    "wrs_assembly_2020/object_metadata/" +
                        std::string(metadata_file_path),
                    *grasp_points, names);
  // create planner and set parameters
  Planner planner;
  planner.load_config_file(
      "/root/o2ac-ur/catkin_ws/src/o2ac_pose_distribution_updater/launch/"
      "estimator_config.yaml");

  double support_surface = 0.0;
  planner.set_geometry(gripped_geometry, grasp_points, support_surface);

  // set covariance
  CovarianceMatrix pre_initial_covariance = CovarianceMatrix::Zero(),
                   initial_covariance;
  double variance = atof(argv[argi++]);
  for (int i = 0; i < 6; i++) {
    pre_initial_covariance(i, i) = variance;
  }

  for (int i = 0; i < grasp_points->size(); i++) {
    printf("%s: ", names[i].c_str());
    Eigen::Isometry3d initial_mean,
        initial_gripper_pose(
            Eigen::AngleAxisd(asin(1.0), Eigen::Vector3d::UnitY())),
        pre_initial_mean = (*grasp_points)[i].inverse();
    planner.grasp_step_with_Lie_distribution(
        gripped_geometry->vertices, gripped_geometry->triangles,
        initial_gripper_pose, pre_initial_mean, pre_initial_covariance,
        initial_mean, initial_covariance, false);
    Eigen::Isometry3d pose_difference =
        initial_mean * pre_initial_mean.inverse();
    if (pose_difference.translation().norm() < 1e-6 &&
        Eigen::AngleAxisd(pose_difference.rotation()).angle() < 1e-6) {
      puts("OK");
    } else {
      printf("NG %lf %lf\n", pose_difference.translation().norm(),
             Eigen::AngleAxisd(pose_difference.rotation()).angle());
    }
  }

  return 0;
}

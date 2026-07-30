// Copyright 2026 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "mg400_common/kinematics.hpp"

namespace
{

enum class Mode
{
  FORWARD,
  INVERSE
};

struct Config
{
  Mode mode;
  std::array<double, 4> values{};
  bool angles_in_radians = false;
  bool positions_in_meters = false;
};

std::string mode_name(Mode mode)
{
  return mode == Mode::FORWARD ? "fk" : "ik";
}

void print_usage(std::ostream & stream)
{
  stream
    << "Usage:\n"
    << "  mg400_kinematics_cli fk <j1> <j2> <j3> <j4> [--radians]\n"
    << "  mg400_kinematics_cli ik <x> <y> <z> <yaw> [--meters] [--radians]\n"
    << "\n"
    << "Default input units:\n"
    << "  fk: joint angles in degrees\n"
    << "  ik: position in millimeters, yaw in degrees\n"
    << "\n"
    << "Options:\n"
    << "  --radians     Interpret angle inputs as radians.\n"
    << "  --degrees     Interpret angle inputs as degrees.\n"
    << "  --meters      Interpret IK position inputs as meters.\n"
    << "  --millimeters Interpret IK position inputs as millimeters.\n"
    << "  -h, --help    Show this help.\n"
    << "\n"
    << "Examples:\n"
    << "  ros2 run mg400_tools mg400_kinematics_cli fk 0 10 20 30\n"
    << "  ros2 run mg400_tools mg400_kinematics_cli ik 250 0 150 90\n"
    << "  ros2 run mg400_tools mg400_kinematics_cli ik 0.25 0.0 0.15 1.5708 --meters --radians\n";
}

double parse_number(const std::string & value)
{
  size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument("Invalid numeric value: " + value);
  }
  return parsed;
}

Config parse_config(const std::vector<std::string> & args)
{
  if (args.empty()) {
    throw std::invalid_argument("Missing mode. Use --help for usage.");
  }

  for (const auto & arg : args) {
    if (arg == "--help" || arg == "-h") {
      print_usage(std::cout);
      std::exit(0);
    }
  }

  Config config;
  if (args.front() == "fk") {
    config.mode = Mode::FORWARD;
  } else if (args.front() == "ik") {
    config.mode = Mode::INVERSE;
  } else {
    throw std::invalid_argument("Unknown mode: " + args.front());
  }

  std::vector<double> numeric_values;
  for (size_t index = 1; index < args.size(); ++index) {
    const std::string & arg = args[index];
    if (arg == "--radians") {
      config.angles_in_radians = true;
    } else if (arg == "--degrees") {
      config.angles_in_radians = false;
    } else if (arg == "--meters") {
      if (config.mode != Mode::INVERSE) {
        throw std::invalid_argument("--meters is only supported in ik mode.");
      }
      config.positions_in_meters = true;
    } else if (arg == "--millimeters") {
      if (config.mode != Mode::INVERSE) {
        throw std::invalid_argument("--millimeters is only supported in ik mode.");
      }
      config.positions_in_meters = false;
    } else {
      numeric_values.push_back(parse_number(arg));
    }
  }

  if (numeric_values.size() != 4) {
    throw std::invalid_argument(
            "Expected 4 numeric values for " + mode_name(config.mode) + " mode.");
  }

  for (size_t index = 0; index < numeric_values.size(); ++index) {
    config.values[index] = numeric_values[index];
  }

  return config;
}

std::string format_vector(
  const std::array<double, 4> & values,
  int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision)
         << "[" << values[0]
         << ", " << values[1]
         << ", " << values[2]
         << ", " << values[3]
         << "]";
  return stream.str();
}

std::string format_vector(const Eigen::Vector4d & values, int precision)
{
  std::array<double, 4> data = {
    values(0), values(1), values(2), values(3)
  };
  return format_vector(data, precision);
}

std::string format_constraint_state(bool valid)
{
  return valid ? "ok" : "ng";
}

void print_constraint_result(const mg400_common::kinematics::ConstraintResult & result)
{
  std::cout
    << "Constraint check: " << (result.allValid() ? "valid" : "invalid") << "\n"
    << "  j1=" << format_constraint_state(result.j1_valid)
    << " j2=" << format_constraint_state(result.j2_valid)
    << " j3=" << format_constraint_state(result.j3_valid)
    << " j4=" << format_constraint_state(result.j4_valid)
    << " j3-j2=" << format_constraint_state(result.j3_1_valid)
    << "\n";
}

Eigen::Vector4d to_joint_vector_rad(const Config & config)
{
  Eigen::Vector4d joints;
  joints << config.values[0], config.values[1], config.values[2], config.values[3];

  if (!config.angles_in_radians) {
    for (int index = 0; index < joints.size(); ++index) {
      joints(index) = mg400_common::kinematics::deg2rad(joints(index));
    }
  }

  return joints;
}

Eigen::Vector4d to_pose_vector_si(const Config & config)
{
  Eigen::Vector4d pose;
  pose << config.values[0], config.values[1], config.values[2], config.values[3];

  if (!config.positions_in_meters) {
    pose.head<3>() /= 1000.0;
  }
  if (!config.angles_in_radians) {
    pose(3) = mg400_common::kinematics::deg2rad(pose(3));
  }

  return pose;
}

std::array<double, 4> to_joint_vector_deg(const Eigen::Vector4d & joints_rad)
{
  return {
    mg400_common::kinematics::rad2deg(joints_rad(0)),
    mg400_common::kinematics::rad2deg(joints_rad(1)),
    mg400_common::kinematics::rad2deg(joints_rad(2)),
    mg400_common::kinematics::rad2deg(joints_rad(3))
  };
}

std::array<double, 4> to_pose_vector_mm_deg(const Eigen::Vector4d & pose_si)
{
  return {
    pose_si(0) * 1000.0,
    pose_si(1) * 1000.0,
    pose_si(2) * 1000.0,
    mg400_common::kinematics::rad2deg(pose_si(3))
  };
}

int run_fk(const Config & config)
{
  const Eigen::Vector4d joints_rad = to_joint_vector_rad(config);
  const auto constraint_result = mg400_common::kinematics::check_constraints(joints_rad);
  const Eigen::Vector4d pose_si = mg400_common::kinematics::fk_for_flange_from_origin(joints_rad);

  std::cout << "Mode: FK\n";
  std::cout << "Input joints\n";
  std::cout << "  deg: " << format_vector(to_joint_vector_deg(joints_rad), 3) << "\n";
  std::cout << "  rad: " << format_vector(joints_rad, 6) << "\n";
  print_constraint_result(constraint_result);
  std::cout << "Output pose (flange from origin)\n";
  std::cout << "  mm/deg: " << format_vector(to_pose_vector_mm_deg(pose_si), 3) << "\n";
  std::cout << "  m/rad : " << format_vector(pose_si, 6) << "\n";

  return constraint_result.allValid() ? 0 : 2;
}

int run_ik(const Config & config)
{
  const Eigen::Vector4d pose_si = to_pose_vector_si(config);
  const Eigen::Vector4d joints_rad = mg400_common::kinematics::ik_for_flange_from_origin(pose_si);
  const auto constraint_result = mg400_common::kinematics::check_constraints(joints_rad);

  std::cout << "Mode: IK\n";
  std::cout << "Input pose (flange from origin)\n";
  std::cout << "  mm/deg: " << format_vector(to_pose_vector_mm_deg(pose_si), 3) << "\n";
  std::cout << "  m/rad : " << format_vector(pose_si, 6) << "\n";
  std::cout << "Output joints\n";
  std::cout << "  deg: " << format_vector(to_joint_vector_deg(joints_rad), 3) << "\n";
  std::cout << "  rad: " << format_vector(joints_rad, 6) << "\n";
  print_constraint_result(constraint_result);

  return constraint_result.allValid() ? 0 : 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    const std::vector<std::string> program_args(args.begin() + 1, args.end());
    const Config config = parse_config(program_args);

    const int exit_code = config.mode == Mode::FORWARD ? run_fk(config) : run_ik(config);
    rclcpp::shutdown();
    return exit_code;
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << "\n";
    print_usage(std::cerr);
    rclcpp::shutdown();
    return 1;
  }
}

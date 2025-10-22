#pragma once

#include <array>
#include <Eigen/Dense>

// approximately equally distributed directions
// generated using blender
// (cube -> subdivision surface (Catmull-Clark), level 2,
// removed half the points, exported as obj, formatted for c++)

inline array<Eigen::Vector3d, 49> probeDirections{
	Eigen::Vector3d{-0.577350, -0.577350, 0.577350},
	Eigen::Vector3d{-0.577350, 0.577350, 0.577350},
	Eigen::Vector3d{-0.577350, -0.577350, -0.577350},
	Eigen::Vector3d{-0.577350, 0.577350, -0.577350},
	Eigen::Vector3d{-0.664003, -0.664003, -0.343804},
	Eigen::Vector3d{-0.707107, -0.707107, -0.000000},
	Eigen::Vector3d{-0.664003, -0.664003, 0.343804},
	Eigen::Vector3d{-0.664003, -0.343804, 0.664003},
	Eigen::Vector3d{-0.707107, -0.000000, 0.707107},
	Eigen::Vector3d{-0.664003, 0.343804, 0.664003},
	Eigen::Vector3d{-0.664003, 0.664003, 0.343804},
	Eigen::Vector3d{-0.707107, 0.707107, 0.000000},
	Eigen::Vector3d{-0.664003, 0.664003, -0.343804},
	Eigen::Vector3d{-0.664003, 0.343804, -0.664003},
	Eigen::Vector3d{-0.707107, -0.000000, -0.707107},
	Eigen::Vector3d{-0.664003, -0.343804, -0.664003},
	Eigen::Vector3d{0.000000, -0.707107, -0.707107},
	Eigen::Vector3d{-0.343804, -0.664003, -0.664003},
	Eigen::Vector3d{-0.343804, 0.664003, -0.664003},
	Eigen::Vector3d{-0.000000, 0.707107, -0.707107},
	Eigen::Vector3d{-0.343804, -0.664003, 0.664003},
	Eigen::Vector3d{-0.343804, 0.664003, 0.664003},
	Eigen::Vector3d{-0.852438, -0.369696, 0.369696},
	Eigen::Vector3d{-0.919965, -0.000000, 0.392001},
	Eigen::Vector3d{-0.852438, 0.369696, 0.369695},
	Eigen::Vector3d{-0.919965, -0.392001, -0.000000},
	Eigen::Vector3d{-1.000000, 0.000000, 0.000000},
	Eigen::Vector3d{-0.919965, 0.392001, 0.000000},
	Eigen::Vector3d{-0.852438, -0.369696, -0.369696},
	Eigen::Vector3d{-0.919965, 0.000000, -0.392001},
	Eigen::Vector3d{-0.852438, 0.369696, -0.369696},
	Eigen::Vector3d{-0.369696, -0.369696, -0.852438},
	Eigen::Vector3d{-0.392001, -0.000000, -0.919965},
	Eigen::Vector3d{-0.369695, 0.369696, -0.852438},
	Eigen::Vector3d{0.000000, -0.392001, -0.919965},
	Eigen::Vector3d{0.000000, 0.000000, -1.000000},
	Eigen::Vector3d{-0.000000, 0.392001, -0.919965},
	Eigen::Vector3d{-0.369696, -0.369696, 0.852438},
	Eigen::Vector3d{-0.392001, 0.000000, 0.919965},
	Eigen::Vector3d{-0.369696, 0.369696, 0.852438},
	Eigen::Vector3d{-0.369696, -0.852438, -0.369696},
	Eigen::Vector3d{-0.000000, -0.919965, -0.392001},
	Eigen::Vector3d{-0.392001, -0.919965, 0.000000},
	Eigen::Vector3d{-0.369695, -0.852438, 0.369696},
	Eigen::Vector3d{0.000000, 0.919965, -0.392001},
	Eigen::Vector3d{-0.369696, 0.852438, -0.369695},
	Eigen::Vector3d{-0.000000, 1.000000, 0.000000},
	Eigen::Vector3d{-0.392001, 0.919965, -0.000000},
	Eigen::Vector3d{-0.369696, 0.852438, 0.369696}
};
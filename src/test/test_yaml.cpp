
#include "stl.h"
#include "str.h"
#include "yaml-cpp/yaml.h"
#include "test_main.h"

TEST(test_yaml, file) {
	YAML::Node config = YAML::LoadFile("../../../models/ur5e/ur5e.yaml");
	const std::string has_gripper = config["has_gripper"].as<std::string>();
	std::cout << has_gripper << "\n";
}

TEST(test_yaml, string) {
	YAML::Node config = YAML::Load("has_gripper: false");
	const std::string has_gripper = config["has_gripper"].as<std::string>();
	std::cout << has_gripper << "\n";
}

TEST(test_yaml, parse_error) {
	bool caught = false;
	try {
		YAML::Node config = YAML::Load("has_gripper=false");
		const std::string has_gripper = config["has_gripper"].as<std::string>();
		std::cout << has_gripper << "\n";
	}
	catch (std::runtime_error& e) {
		caught = true;
		std::cout << e.what() << "\n";
	}
	EXPECT_TRUE(caught);
}

TEST(test_yaml, missing_error) {
	bool caught = false;
	try {
		YAML::Node config = YAML::Load("has_gripper: false");
		const std::string has_gripper = config["has_another_gripper"].as<std::string>();
		std::cout << has_gripper << "\n";
	}
	catch (std::runtime_error& e) {
		caught = true;
		std::cout << e.what() << "\n";
	}
	EXPECT_TRUE(caught);
}

TEST(test_yaml, sequence) {
	YAML::Node config = YAML::Load("[[1,2],[3,4]]");
	std::cout << config[0].size() << "\n";
}
TEST_MAIN

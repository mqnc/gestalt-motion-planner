
#include "stl.h"
#include "str.h"
#include <Eigen/Dense>
#include "encapsulator/encapsulate.h"
#include "test_main.h"
#include "render/html/renderer.html.h"

using namespace Eigen;

TEST(test_encapsulate, double_sphere) {

	// support points and radius of our expected capsule
	//*
	Vector3d s1{ 1, -2, 3 };
	Vector3d s2{ -4, 5, -6 };
	double r = 7;
	/*/
	Vector3d s1{ 0, 0, 0 };
	Vector3d s2{ 3, 0, 0 };
	double r = 1;
	/**/

	// generate spheres of random points around support points
	std::vector<std::valarray<double>> points;
	std::vector<Vector3d> eigPoints;

	for (size_t i = 0; i < 50; i++) {
		Vector3d p = Vector3d::Random();
		if (p.norm() > 1) {
			p = p.normalized();
		}
		Vector3d p1 = s1 + p * r;
		Vector3d p2 = s2 + p * r;
		points.push_back({ p1[0], p1[1], p1[2] });
		points.push_back({ p2[0], p2[1], p2[2] });
		eigPoints.push_back(p1);
		eigPoints.push_back(p2);
	}

	Capsule cap = encapsulatePoints(points);

	std::stringstream ss;
	ss << "convexhull([";
	for (const auto& p : points) {
		ss << "v3(" << p[0] << "," << p[1] << "," << p[2] << "),";
	}
	ss << "]);\n";

	ss << "capsule(";
	ss << "v3(" << cap.p0[0] << "," << cap.p0[1] << "," << cap.p0[2] << "),";
	ss << "v3(" << cap.p1[0] << "," << cap.p1[1] << "," << cap.p1[2] << "),";
	ss << cap.radius << ");\n";

	/*
	ss << "sphere(" << cap.radius << ",["
		<< cap.p0[0] << "," << cap.p0[1] << "," << cap.p0[2] << ",0,0,0,1]);\n";
	ss << "sphere(" << cap.radius << ",["
		<< cap.p1[0] << "," << cap.p1[1] << "," << cap.p1[2] << ",0,0,0,1]);\n";
	*/

	str::save("encapsulation.html", str::replace(rendererHtml, "/*OBJECTS*/", ss.str()));
}

TEST_MAIN


#include "stl.h"
#include "str.h"
#include "test_main.h"
#include <cstdlib>

#include "quickhull/QuickHull.cpp"

using namespace quickhull;

TEST(test_quickhull, complete) {
	QuickHull<double> qh;
	std::vector<Vector3<double>> pointCloud;

	for(size_t i=0; i<1e6; i++){
		pointCloud.push_back(Vector3<double>(
			1.0*rand()/RAND_MAX,
			1.0*rand()/RAND_MAX,
			1.0*rand()/RAND_MAX
		));
	}

	auto hull = qh.getConvexHull(pointCloud, true, false);
	const auto& indexBuffer = hull.getIndexBuffer();
	const auto& vertexBuffer = hull.getVertexBuffer();
}

TEST_MAIN

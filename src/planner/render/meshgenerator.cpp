
#include "meshgenerator.h"
#include "spherepoints.h"

Mesh UNIT_SPHERE = convexHull({
	UNIT_SPHERE_POINTS,
	{}
	});

MeshGenerator::MeshGenerator(
	Shape* shape,
	btTransform trafo,
	size_t indexOffset
) :
	m_trafo{ trafo },
	m_indexOffset{ indexOffset }{

	// the compound shape already applies transformation and offset
	// during visit and then resets them
	shape->accept(*this);

	// for all other shapes we apply transformation and offset now
	if (not (m_trafo == btTransform::getIdentity())) {
		for (auto& v : m_result.vertices) {
			btVector3 tv = this->m_trafo * btVector3(v[0], v[1], v[2]);
			v = { tv.x(), tv.y(), tv.z() };
		}
	}

	if (m_indexOffset != 0) {
		for (auto& f : m_result.faces) {
			for (auto& c : f) {
				c += m_indexOffset;
			}
		}
	}
}

void MeshGenerator::visit(CompoundShape* shape) {
	m_result = {};

	size_t offset = m_indexOffset;

	for (size_t i = 0; i < shape->getNumShapes(); i++) {

		auto childTrafo = shape->getShapeTrafo(i);
		auto child = shape->getShape(i);

		auto childMesh = MeshGenerator(
			child,
			m_trafo * childTrafo,
			offset
		).get();

		std::copy(
			childMesh.vertices.begin(),
			childMesh.vertices.end(),
			std::back_inserter(m_result.vertices)
		);
		std::copy(
			childMesh.faces.begin(),
			childMesh.faces.end(),
			std::back_inserter(m_result.faces)
		);

		offset += childMesh.vertices.size();
	}

	// this has already been applied to all children,
	// so we don't have to do it in the constructor again after the visit
	m_trafo = btTransform::getIdentity();
	m_indexOffset = 0;
}


void MeshGenerator::visit(BoxShape* shape) {
	btVector3 whd = shape->getSize();
	double margin = shape->getSafetyMargin();

	Mesh cuboid;
	cuboid.vertices.reserve(8);
	for (int i = -1; i <= 1; i += 2) {
		for (int j = -1; j <= 1; j += 2) {
			for (int k = -1; k <= 1; k += 2) {
				cuboid.vertices.push_back({
					i * whd[0] / 2,
					j * whd[1] / 2,
					k * whd[2] / 2 });
			}
		}
	}

	if (margin == 0) {
		m_result = convexHull(cuboid);
	}
	else {
		m_result = minkowskiSum(cuboid, UNIT_SPHERE, { margin });
#ifndef COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
		// draw another box around
		Mesh cuboid2;
		cuboid2.vertices.reserve(8);
		for (int i = -1; i <= 1; i += 2) {
			for (int j = -1; j <= 1; j += 2) {
				for (int k = -1; k <= 1; k += 2) {
					cuboid2.vertices.push_back({
						i * (whd[0] / 2 + margin),
						j * (whd[1] / 2 + margin),
						k * (whd[2] / 2 + margin) });
				}
			}
		}
		m_result = combine(m_result, convexHull(cuboid2));
#endif
	}
}

void MeshGenerator::visit(CylinderZShape* shape) {
	double radius = shape->getRadius();
	double height = shape->getHeight();
	double margin = shape->getSafetyMargin();

	const int n = 36;

	Mesh cylinder;
	cylinder.vertices.reserve(n);
	for (int i = 0; i < n; i++) {
		double phi = 2 * M_PI * i / n;
		for (int j = -1; j <= 1; j++) {
			cylinder.vertices.push_back({
				cos(phi) * radius,
				sin(phi) * radius,
				j * height / 2 });
		}
	}

	if (margin == 0) {
		m_result = convexHull(cylinder);
	}
	else {
		m_result = minkowskiSum(cylinder, UNIT_SPHERE, { margin });
	}
}

void MeshGenerator::visit(SphereShape* shape) {
	auto r = shape->getRadius() + shape->getSafetyMargin();

	m_result;
	m_result.vertices.reserve(UNIT_SPHERE.vertices.size());
	for (const auto& v : UNIT_SPHERE.vertices) {
		m_result.vertices.push_back({ r * v[0], r * v[1], r * v[2] });
	}
	std::copy(std::begin(UNIT_SPHERE.faces), std::end(UNIT_SPHERE.faces),
		std::back_inserter(m_result.faces));
}

void MeshGenerator::visit(CapsuleZShape* shape) {
	double r = shape->getRadius() + shape->getSafetyMargin();
	double h = shape->getHeight();

	m_result = minkowskiSum(
		Mesh{ {{0, 0, -h / 2}, {0, 0, h / 2}, {}} },
		UNIT_SPHERE,
		{ r }
	);
}

void MeshGenerator::visit(ConvexHullShape* shape) {
	double margin = shape->getSafetyMargin();

	Mesh hull;
	hull.vertices.reserve(shape->getNumPoints());
	for (size_t i = 0; i < shape->getNumPoints(); i++) {
		btVector3 v = shape->getPoint(i);
		hull.vertices.push_back({ v[0], v[1], v[2] });
	}

	if (margin == 0) {
		m_result = convexHull(hull);
	}
	else {
		m_result = minkowskiSum(hull, UNIT_SPHERE, { margin });
	}
}

void MeshGenerator::visit(ConcaveTriangleMeshShape* shape){
	double margin = shape->getSafetyMargin();
	auto triangles = shape->getTriangles();
	size_t i = 0;
	for (const auto& triangle : triangles) {
		btVector3 normal;
		triangle.calcNormal(normal);
		for (double direction : {-1, 1}) {
			for (size_t j = 0; j < 3; j++) {
				btVector3 v = triangle.m_vertices1[j];
				v += normal * direction * margin;
				m_result.vertices.push_back({ v[0], v[1], v[2] });
			}
			m_result.faces.push_back({ i * 3, i * 3 + 1, i * 3 + 2 });
			i++;
		}

	}
}

void MeshGenerator::visit(MultiSphereShape* shape) {
	double margin = shape->getSafetyMargin();

	Mesh multi;
	multi.vertices.reserve(shape->getNumSpheres());

	valarray<double> radii(shape->getNumSpheres());
	for (size_t i = 0; i < shape->getNumSpheres(); i++) {
		btVector3 p = shape->getSpherePosition(i);
		multi.vertices.push_back({ p[0], p[1], p[2] });
		radii[i] = shape->getSphereRadius(i) + margin;
	}
	m_result = minkowskiSum(multi, UNIT_SPHERE, radii);
}

void MeshGenerator::visit(VisualMeshShape* shape) {
	if (shape->getSafetyMargin() != 0) {
		throw runtime_error("concave meshes cannot be inflated");
	}

	m_result;

	const auto& vs = shape->getVertices();
	m_result.vertices.reserve(vs.size());
	for (const auto& v : vs) {
		m_result.vertices.push_back({ v[0], v[1], v[2] });
	}

	const auto& fs = shape->getFaces();
	m_result.faces.reserve(fs.size());
	for (const auto& f : fs) {
		m_result.faces.push_back({ f[0], f[1], f[2] });
	}
}

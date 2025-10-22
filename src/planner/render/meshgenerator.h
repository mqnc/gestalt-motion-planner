
#pragma once

#include "common.h"
#include "zip.h"

#include "btBulletCollisionCommon.h"

#include "collision/shapes.h"
#include "collision/collisionrobot.h"
#include "collision/collisionchecker.h"

#include "quickhull/QuickHull.hpp"

struct Mesh {
	vector<array<double, 3>> vertices; // must be contiguous
	vector<array<uint64_t, 3>> faces;
	string hash;
};

inline string hash(Mesh mesh) {
	const double phi = (sqrt(5) + 1) / 2;
	double phi_n = 0;
	double sum = 0;

	for (const auto& v : mesh.vertices) {
		for (const auto& vi : v) {
			sum = sum + phi_n * vi;
			phi_n += phi;
			phi_n -= floor(phi_n);
		}
	}

	for (const auto& f : mesh.faces) {
		for (const auto& fi : f) {
			sum = sum + phi_n * fi;
			phi_n += phi;
			phi_n -= floor(phi_n);
		}
	}

	return string("#") + str::replace(to_string(sum), ".", "");
}

inline Mesh combine(
	const Mesh& mesh1,
	const Mesh& mesh2
) {
	Mesh buffer;

	buffer.vertices = mesh1.vertices;
	buffer.vertices.reserve(mesh1.vertices.size() + mesh2.vertices.size());
	for (const auto& v : mesh2.vertices) {
		buffer.vertices.push_back(v);
	}

	buffer.faces = mesh1.faces;
	buffer.faces.reserve(mesh1.faces.size() + mesh2.faces.size());
	for (const auto& f : mesh2.faces) {
		buffer.faces.push_back({
			f[0] + mesh1.vertices.size(),
			f[1] + mesh1.vertices.size(),
			f[2] + mesh1.vertices.size()
			});
	}

	return buffer;
}

inline Mesh convexHull(const Mesh& mesh) {
	quickhull::QuickHull<double> qh;

	quickhull::ConvexHull hull = qh.getConvexHull(
		&(mesh.vertices[0][0]), mesh.vertices.size(), false, false);

	Mesh result;

	const quickhull::VertexDataSource<double>& vertexBuffer = hull.getVertexBuffer();
	result.vertices.resize(vertexBuffer.size());

	for (auto&& [i, v] : enumerate(vertexBuffer)) {
		result.vertices[i] = { v.x, v.y, v.z };
	}

	const std::vector<size_t>& indexBuffer = hull.getIndexBuffer();
	result.faces.resize(indexBuffer.size() / 3);
	for (size_t i = 0; i < indexBuffer.size() / 3; i++) {
		result.faces[i] = {
			indexBuffer[i * 3], indexBuffer[i * 3 + 1], indexBuffer[i * 3 + 2] };
	}

	return result;
}

inline Mesh minkowskiSum(
	const Mesh& mesh1,
	const Mesh& mesh2,
	valarray<double> scale2 = { 1 }) {

	Mesh buffer;
	buffer.vertices.reserve(mesh1.vertices.size() * mesh2.vertices.size());
	if (scale2.size() == 1) {
		const auto& s = scale2[0];
		for (const auto& v1 : mesh1.vertices) {
			for (const auto& v2 : mesh2.vertices) {
				buffer.vertices.push_back({
					v1[0] + s * v2[0],
					v1[1] + s * v2[1],
					v1[2] + s * v2[2] });
			}
		}
	}
	else {
		for (size_t i = 0; i < mesh1.vertices.size(); i++) {
			const auto& v1 = mesh1.vertices[i];
			const auto& s = scale2[i];
			for (const auto& v2 : mesh2.vertices) {
				buffer.vertices.push_back({
					v1[0] + s * v2[0],
					v1[1] + s * v2[1],
					v1[2] + s * v2[2] });
			}
		}
	}

	return convexHull(buffer);
}

class MeshGenerator :public ShapeVisitor {
	Mesh m_result;
	btTransform m_trafo;
	size_t m_indexOffset;

public:
	MeshGenerator(
		Shape* shape,
		btTransform trafo = btTransform::getIdentity(),
		size_t indexOffset = 0
	);
	virtual void visit(CompoundShape* shape);
	virtual void visit(BoxShape* shape);
	virtual void visit(CylinderZShape* shape);
	virtual void visit(SphereShape* shape);
	virtual void visit(CapsuleZShape* shape);
	virtual void visit(ConvexHullShape* shape);
	virtual void visit(ConcaveTriangleMeshShape* shape);
	virtual void visit(MultiSphereShape* shape);
	virtual void visit(VisualMeshShape* shape);

	Mesh get() {
		m_result.hash = hash(m_result);
		return m_result;
	}
};

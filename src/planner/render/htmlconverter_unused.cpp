
#include "htmlconverter.h"
#include "html/renderer.html.h"

string vectorToHtml(double x, double y, double z) {
	stringstream ss;
	ss << "v3(" << x << "," << y << "," << z << ")";
	return ss.str();
}

string vectorToHtml(const btVector3& vec) {
	return vectorToHtml(vec[0], vec[1], vec[2]);
}

string trafoToHtml(const btTransform& trafo) {
	stringstream ss;
	auto p = trafo.getOrigin();
	auto q = trafo.getRotation();
	ss << "[" << p.getX() << "," << p.getY() << "," << p.getZ() << ","
		<< q.getX() << "," << q.getY() << "," << q.getZ() << "," << q.getW() << "]";
	return ss.str();
}

void HtmlConverter::visit(CompoundShape* shape) {
	for (size_t i = 0; i < shape->getNumShapes(); i++) {
		auto childTrafo = shape->getShapeTrafo(i);
		auto child = shape->getShape(i);
		HtmlConverter childVisitor(trafo * childTrafo, material);
		child->accept(childVisitor);
		result << childVisitor.getResult();
	}
}
void HtmlConverter::visit(BoxShape* shape) {
	btVector3 whd = shape->getSize();
	double margin = shape->getSafetyMargin();
	result << "box("
		<< whd[0] << "," << whd[1] << "," << whd[2]
		<< "," << trafoToHtml(trafo)
		<< "," << material
		<< "," << margin << ");\n";
}
void HtmlConverter::visit(CylinderZShape* shape) {
	double r = shape->getRadius();
	double h = shape->getHeight();
	double margin = shape->getSafetyMargin();
	result << "cylinderz("
		<< r << "," << h
		<< "," << trafoToHtml(trafo)
		<< "," << material
		<< "," << margin << ");\n";
}
void HtmlConverter::visit(SphereShape* shape) {
	auto r = shape->getRadius();
	double margin = shape->getSafetyMargin();
	result << "sphere(" << r
		<< "," << trafoToHtml(trafo)
		<< "," << material
		<< "," << margin << ");\n";
}
void HtmlConverter::visit(CapsuleZShape* shape) {
	double margin = shape->getSafetyMargin();
	double r = shape->getRadius();
	double h = shape->getHeight();
	result << "orangenet([" << vectorToHtml(0, 0, -h / 2.0 + r) << ","
		<< vectorToHtml(0, 0, h / 2.0 - r) << "],["
		<< r << "," << r << "],"
		<< trafoToHtml(trafo)
		<< "," << material
		<< "," << margin << ");\n";
}
void HtmlConverter::visit(ConvexHullShape* shape) {
	// orangenet(centers, radii, trafo, margin = 0, draw = OBJECT_AND_HULL)
	double margin = shape->getSafetyMargin();
	result << "orangenet([";
	for (size_t i = 0; i < shape->getNumPoints(); i++) {
		btVector3 v = shape->getPoint(i);
		result << "v3(" << v[0] << "," << v[1] << "," << v[2] << "),";
	}
	result << "],0,"
		<< trafoToHtml(trafo)
		<< "," << material
		<< "," << margin << ");\n";
}
void HtmlConverter::visit(ConcaveTriangleMeshShape* shape) {
	cout << "TODO\n";
}
void HtmlConverter::visit(MultiSphereShape* shape) {
	double margin = shape->getSafetyMargin();
	result << "orangenet([";
	for (size_t i = 0; i < shape->getNumSpheres(); i++) {
		btVector3 v = shape->getSpherePosition(i);
		result << "v3(" << v[0] << "," << v[1] << "," << v[2] << "),";
	}
	result << "],[";
	for (size_t i = 0; i < shape->getNumSpheres(); i++) {
		result << shape->getSphereRadius(i) << ",";
	}
	result << "],"
		<< trafoToHtml(trafo)
		<< "," << material
		<< "," << margin << ");\n";
}
void HtmlConverter::visit(VisualMeshShape* shape) {
	result << "mesh([";
	for (const auto& v : shape->getVertices()) {
		result << v[0] << "," << v[1] << "," << v[2] << ",\n";
	}
	result << "],[\n";
	for (const auto& f : shape->getFaces()) {
		result << f[0] << "," << f[1] << "," << f[2] << ",\n";
	}
	result << "],"
		<< trafoToHtml(trafo)
		<< "," << material << ");\n";
}


string sceneToHtml(
	const dict<CollisionRobot>& robots,
	const CollisionChecker::CollisionReport& collisions
) {
	stringstream ss;

	for (const auto& [_, robot] : robots) {
		for (const auto& part : robot.getParts()) {
			auto trafo = part->bulletObject.getWorldTransform();

			HtmlConverter htmlConvCol(trafo, "forceFieldMaterial");
			part->collisionShape->accept(htmlConvCol);
			ss << htmlConvCol.getResult();

			HtmlConverter htmlConvVis(trafo, "objectMaterial");
			part->visualShape->accept(htmlConvVis);
			ss << htmlConvVis.getResult();
		}
	}

	for (const auto& col : collisions.collisions.value()) {
		ss << "collision(" << vectorToHtml(col.pointOnB) << ")\n;"
			<< "collision(" << vectorToHtml(col.pointOnB) << ")\n;";
	}

	return ss.str();
}

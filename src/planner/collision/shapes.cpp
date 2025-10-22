
#include "shapes.h"

Shape* Shape::fromBullet(const btCollisionShape* const btShape) {
	auto shape = static_cast<Shape*>(btShape->getUserPointer());
	assert(shape->getBulletShape() == btShape);
	return shape;
}

template<typename D, typename Bt>
void ShapeTemplate<D, Bt>::accept(ShapeVisitor& v) {
	v.visit(static_cast<D*>(this));
}

// "true" argument enables dynamicAABBtree for children
CompoundShape::CompoundShape()
	:ShapeTemplate<CompoundShape, btCompoundShape>{ true }
{
	setSafetyMargin(0);
}

// copy constructor
CompoundShape::CompoundShape(const CompoundShape& other)
	:CompoundShape() {

	children.reserve(other.children.size());
	for (size_t i = 0; i < other.children.size(); i++) {
		children.push_back(other.children[i]->clone());
		bulletShape.addChildShape(
			other.bulletShape.getChildTransform(i),
			children[i]->getBulletShape()
		);
	}
}

// move constructor
CompoundShape::CompoundShape(CompoundShape&& other) noexcept
	:CompoundShape() {

	children.reserve(other.children.size());
	for (size_t i = 0; i < other.children.size(); i++) {
		children.push_back(move(other.children[i]));
		bulletShape.addChildShape(
			other.bulletShape.getChildTransform(i),
			children[i]->getBulletShape()
		);
	}
}

// destructor
CompoundShape::~CompoundShape() {}

void CompoundShape::addShape(
	const btTransform& localTransform,
	Holder<Shape> shape
) {
	bulletShape.addChildShape(localTransform, shape->getBulletShape());
	children.push_back(move(shape));
}

size_t CompoundShape::getNumShapes() {
	return children.size();
}

Shape* CompoundShape::getShape(size_t index) {
	return children[index].get();
}

btTransform CompoundShape::getShapeTrafo(size_t index){
	return getBulletShape()->getChildTransform(index);
}

void CompoundShape::setSafetyMargin(double margin) {
	for (size_t i = 0; i < getNumShapes(); i++) {
		getShape(i)->setSafetyMargin(margin);
	}
	update();
}

double CompoundShape::getSafetyMargin() {
	if (getNumShapes() == 0) {
		return NaN;
	}
	double margin = getShape(0)->getSafetyMargin();
	for (size_t i = 0; i < getNumShapes(); i++) {
		if (getShape(i)->getSafetyMargin() != margin) {
			return NaN;
		}
	}
	return margin;
}

void CompoundShape::update() {
	for (size_t i = 0; i < getNumShapes(); i++) {
		// we don't actually update the trafo, we call this to update the dynamic aabb tree
		// which is not done by recalculateLocalAabb() :/
		bulletShape.updateChildTransform(i, bulletShape.getChildTransform(i), false);
	}
	bulletShape.recalculateLocalAabb();
}


BoxShape::BoxShape(double w, double h, double d) :
	// bullet uses half extents
	ShapeTemplate<BoxShape, btBoxShape>(btVector3(w / 2.0, h / 2.0, d / 2.0)) {

	bulletShape.setMargin(0);
	setSafetyMargin(0);
}

btVector3 BoxShape::getSize() {
	return bulletShape.getHalfExtentsWithoutMargin() * 2.0;
}

void BoxShape::setSafetyMargin(double margin) {
	bulletShape.btConvexInternalShape::setMargin(margin);
}

double BoxShape::getSafetyMargin() {
	return bulletShape.btConvexInternalShape::getMargin();
}


CylinderZShape::CylinderZShape(double r, double h) :
	ShapeTemplate<CylinderZShape, btCylinderShapeZ>(btVector3(r, r, h / 2.0)) {

	bulletShape.setMargin(0);
	setSafetyMargin(0);
}

double CylinderZShape::getRadius() {
	int radiusAxis = (bulletShape.getUpAxis() + 2) % 3;
	return bulletShape.getHalfExtentsWithoutMargin()[radiusAxis];
}

double CylinderZShape::getHeight() {
	return bulletShape.getHalfExtentsWithoutMargin() // ...
		[bulletShape.getUpAxis()] * 2.0;
}

void CylinderZShape::setSafetyMargin(double margin) {
	bulletShape.btConvexInternalShape::setMargin(margin);
}

double CylinderZShape::getSafetyMargin() {
	return bulletShape.btConvexInternalShape::getMargin();
}


SphereShape::SphereShape(double radius) :
	ShapeTemplate<SphereShape, btSphereShape>{ radius },
	radiusWithoutMargin{ radius }{

	setSafetyMargin(0);
}

double SphereShape::getRadius() {
	return radiusWithoutMargin;
}

void SphereShape::setSafetyMargin(double margin) {
	bulletShape.setUnscaledRadius(radiusWithoutMargin + margin);
}

double SphereShape::getSafetyMargin() {
	return bulletShape.getRadius() - radiusWithoutMargin;
}


CapsuleZShape::CapsuleZShape(double radius, double height) :
	ShapeTemplate<CapsuleZShape, btCapsuleShapeZ>{ radius, height },
	dimsWithoutMargin{ bulletShape.getImplicitShapeDimensions() }{

	setSafetyMargin(0);
}

double CapsuleZShape::getRadius() {
	int radiusAxis = (bulletShape.getUpAxis() + 2) % 3;
	return dimsWithoutMargin[radiusAxis];
}

double CapsuleZShape::getHeight() {
	return dimsWithoutMargin[bulletShape.getUpAxis()] * 2.0;
}

void CapsuleZShape::setSafetyMargin(double margin) {
	bulletShape.setImplicitShapeDimensions(
		dimsWithoutMargin + btVector3(margin, margin, margin));
	int radiusAxis = (bulletShape.getUpAxis() + 2) % 3;
	bulletShape.btConvexInternalShape::setMargin(
		dimsWithoutMargin[radiusAxis] + margin);
}

double CapsuleZShape::getSafetyMargin() {
	return bulletShape.getRadius() - getRadius();
}


ConvexHullShape::ConvexHullShape() {
	setSafetyMargin(0);
}

void ConvexHullShape::addPoint(double x, double y, double z) {
	bulletShape.addPoint(btVector3(x, y, z));
}

void ConvexHullShape::optimize() {
	bulletShape.optimizeConvexHull();
}

size_t ConvexHullShape::getNumPoints() {
	return bulletShape.getNumPoints();
}

btVector3 ConvexHullShape::getPoint(size_t index) {
	return bulletShape.getUnscaledPoints()[index];
}

void ConvexHullShape::setSafetyMargin(double margin) {
	bulletShape.setMargin(margin);
}

double ConvexHullShape::getSafetyMargin() {
	return bulletShape.getMargin();
}


ConcaveTriangleMeshShape::ConcaveTriangleMeshShape(shared_ptr<btTriangleMesh> mesh):
	ShapeTemplate<ConcaveTriangleMeshShape, btGImpactMeshShape>{ mesh.get() },
	// might be multi-thread unsafe as the shared pointer is not copied yet
	mesh{ mesh }
{
    bulletShape.updateBound();
	setSafetyMargin(0);
}

ConcaveTriangleMeshShape::ConcaveTriangleMeshShape(const ConcaveTriangleMeshShape& other):
	ShapeTemplate<ConcaveTriangleMeshShape, btGImpactMeshShape>{ other.mesh.get() },
	// might be multi-thread unsafe as the shared pointer is not copied yet
	mesh{ other.mesh }
{
	bulletShape.updateBound();
	setSafetyMargin(0);
}

ConcaveTriangleMeshShape::ConcaveTriangleMeshShape(ConcaveTriangleMeshShape&& other) noexcept:
	ShapeTemplate<ConcaveTriangleMeshShape, btGImpactMeshShape>{ other.mesh.get() },
	// might be multi-thread unsafe as the shared pointer is not copied yet
	mesh{ move(other.mesh) }
{
	bulletShape.updateBound();
	setSafetyMargin(0);
}

size_t ConcaveTriangleMeshShape::getNumTriangles(){
	return mesh->getNumTriangles();
}

std::vector<btTriangleShapeEx> ConcaveTriangleMeshShape::getTriangles(){
	std::vector<btTriangleShapeEx> result;
	result.reserve(getNumTriangles());
	bulletShape.getMeshPart(0)->lockChildShapes();
	for (size_t i=0; i<getNumTriangles(); i++){
		btTriangleShapeEx triangle;
		bulletShape.getMeshPart(0)->getBulletTriangle(i, triangle);
		result.push_back(triangle);
	}
	bulletShape.getMeshPart(0)->unlockChildShapes();
	return result;
}

void ConcaveTriangleMeshShape::setSafetyMargin(double margin){
	bulletShape.setMargin(margin);
    bulletShape.updateBound();
}

double ConcaveTriangleMeshShape::getSafetyMargin(){
	return bulletShape.getMargin();
}


size_t MultiSphereShape::getNumSpheres() {
	return bulletShape.getSphereCount();
}

btVector3 MultiSphereShape::getSpherePosition(size_t index) {
	return bulletShape.getSpherePosition(index);
}

double MultiSphereShape::getSphereRadius(size_t index) {
	return bulletShape.getSphereRadius(index);
}

void MultiSphereShape::setSafetyMargin(double margin) {
	bulletShape.setMargin(margin);
	bulletShape.recalcLocalAabb();
}

double MultiSphereShape::getSafetyMargin() {
	return bulletShape.getMargin();
}


void MultiSphereShapeBuilder::addSphere(double x, double y, double z, double r) {
	positions.emplace_back(x, y, z);
	radii.push_back(r);
}

MultiSphereShape MultiSphereShapeBuilder::build() {
	return MultiSphereShape(positions.data(), radii.data(),
		static_cast<int>(positions.size()));
}


void VisualMeshShape::setSafetyMargin(double margin) {
	throw runtime_error("VisualMeshShape can't have safety margin");
}

double VisualMeshShape::getSafetyMargin() {
	return 0;
}

void VisualMeshShape::addVertex(btVector3 v) {
	vertices.emplace_back(v);
}

void VisualMeshShape::addFace(const array<size_t, 3>& f) {
	faces.emplace_back(f);
}

const vector<btVector3>& VisualMeshShape::getVertices() const {
	return vertices;
}

const vector<array<size_t, 3>>& VisualMeshShape::getFaces() const {
	return faces;
}



#pragma once

#include "common.h"
#include "holder.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

/*
Bullet physics uses a little margin around objects to make dynamic
collision handling faster and more stable. We abuse this for
creating a safety margin around objects so the trajectory planner
plans with some wiggle room.
However, bullet does not implement the margin in the same way for
all shapes (see here https://www.youtube.com/watch?v=BGAwRKPlpCw)
so we wrap them in a more consistent interface.
Note that the box-box collision detection doesn't consider rounded
edges (see video) and I have decided that we live with this.
Shapes can be copied and moved around like integers, compound shapes
own their children, copies are deep copies.
*/

class ShapeVisitor;

class Shape {
public:
	virtual btCollisionShape* getBulletShape() = 0;
	static Shape* fromBullet(const btCollisionShape* const btShape);
	virtual void setSafetyMargin(double margin) = 0;
	virtual double getSafetyMargin() = 0;
	virtual void accept(ShapeVisitor& v) = 0;
	virtual Holder<Shape> clone() = 0;
	virtual ~Shape() = default;
};

template<typename D, typename Bt>
class ShapeTemplate :public Shape {
	// the D parameter is so that a ShapeTemplate knows the type of its derivats
	// so accept and clone don't have to be repeated for all shape types

protected:
	Bt bulletShape;
public:
	template<typename... Args>
	ShapeTemplate(Args&& ... args)
		: bulletShape{ std::forward<Args>(args)... } {

		bulletShape.setUserPointer(static_cast<Shape*>(this));
	}

	virtual ~ShapeTemplate() = default;

	ShapeTemplate(const ShapeTemplate& other) // copy constructor
		: bulletShape(other.bulletShape) {
		bulletShape.setUserPointer(static_cast<Shape*>(this));
	}

	ShapeTemplate(ShapeTemplate&& other) noexcept // move constructor
		: bulletShape(other.bulletShape) {
		bulletShape.setUserPointer(static_cast<Shape*>(this));
	}
	
	ShapeTemplate& operator=(const ShapeTemplate& other) = delete; // copy assignment
	ShapeTemplate& operator=(ShapeTemplate&& other) noexcept = delete; // move assignment
	
	Bt* getBulletShape() noexcept override {
		return &bulletShape;
	}

	// needs to be defined after ShapeVisitor
	void accept(ShapeVisitor& v) override;

	virtual Holder<Shape> clone() {
		return make_unique<D>(*static_cast<D*>(this));
	}

	friend class ShapeVisitor;
};

// bullet has some internal optimization going on which caches the aabb of the child shapes
// so if you mess around with child shapes, make sure to call update() afterwards
class CompoundShape :public ShapeTemplate<CompoundShape, btCompoundShape> {
	vector<Holder<Shape>> children;
public:
	CompoundShape();
	CompoundShape(const CompoundShape& other); // copy constructor
	CompoundShape(CompoundShape&& other) noexcept; // move constructor
	~CompoundShape();

	void addShape(
		const btTransform& localTransform,
		Holder<Shape> shape
	);
	size_t getNumShapes();
	Shape* getShape(size_t index);
	btTransform getShapeTrafo(size_t index);
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
	void update();
};

class BoxShape :public ShapeTemplate<BoxShape, btBoxShape> {
public:
	BoxShape(double w, double h, double d);
	btVector3 getSize();
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
};

class CylinderZShape :public ShapeTemplate<CylinderZShape, btCylinderShapeZ> {
public:
	CylinderZShape(double r, double h);
	double getRadius();
	double getHeight();
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
};

class SphereShape :public ShapeTemplate<SphereShape, btSphereShape> {
	double radiusWithoutMargin = 0;
public:
	SphereShape(double radius);
	double getRadius();
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
};

class CapsuleZShape :public ShapeTemplate<CapsuleZShape, btCapsuleShapeZ> {
	btVector3 dimsWithoutMargin;
public:
	CapsuleZShape(double radius, double height);
	double getRadius();
	double getHeight();
	void setSafetyMargin(double margin) override;
	double getSafetyMargin();
};

class ConvexHullShape :public ShapeTemplate<ConvexHullShape, btConvexHullShape> {
public:
	ConvexHullShape();
	void addPoint(double x, double y, double z);
	void optimize();
	size_t getNumPoints();
	btVector3 getPoint(size_t index);
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
};

class ConcaveTriangleMeshShape :public ShapeTemplate<ConcaveTriangleMeshShape, btGImpactMeshShape> {
	shared_ptr<btTriangleMesh> mesh;
public:
	ConcaveTriangleMeshShape(shared_ptr<btTriangleMesh> mesh);
	ConcaveTriangleMeshShape(const ConcaveTriangleMeshShape& other); // copy constructor
	ConcaveTriangleMeshShape(ConcaveTriangleMeshShape&& other) noexcept; // move constructor
	size_t getNumTriangles();
	std::vector<btTriangleShapeEx> getTriangles();
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
};

// prevent bullet from subtracting the margin by briefly setting it 0
class btMultiSphereShapeWithMargin :public btMultiSphereShape {
	using btMultiSphereShape::btMultiSphereShape;

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const {
		double margin = getMargin();
		const_cast<btMultiSphereShapeWithMargin*>(this)->setMargin(0);
		auto result = btMultiSphereShape::localGetSupportingVertexWithoutMargin(vec);
		const_cast<btMultiSphereShapeWithMargin*>(this)->setMargin(margin);
		return result;
	};

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors)const {
		double margin = getMargin();
		const_cast<btMultiSphereShapeWithMargin*>(this)->setMargin(0);
		btMultiSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(
			vectors, supportVerticesOut, numVectors);
		const_cast<btMultiSphereShapeWithMargin*>(this)->setMargin(margin);
	}
};

class MultiSphereShape :public ShapeTemplate<MultiSphereShape, btMultiSphereShapeWithMargin> {
public:
	template<typename... Args>
	MultiSphereShape(const btVector3* positions, const btScalar* radi, int numSpheres)
		: ShapeTemplate<MultiSphereShape, btMultiSphereShapeWithMargin>{
			positions, radi, numSpheres
	} {
		setSafetyMargin(0);
	}
	size_t getNumSpheres();
	btVector3 getSpherePosition(size_t index);
	double getSphereRadius(size_t index);
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() override;
};

class MultiSphereShapeBuilder {
	vector<btVector3> positions;
	vector<double> radii;
public:
	MultiSphereShapeBuilder() {}

	void addSphere(double x, double y, double z, double r);
	MultiSphereShape build();
};


class VisualMeshShape :public ShapeTemplate<VisualMeshShape, btEmptyShape> {
	vector<btVector3> vertices;
	vector<array<size_t, 3>> faces;

public:
	VisualMeshShape() :
		ShapeTemplate<VisualMeshShape, btEmptyShape>{} {}

	VisualMeshShape(
		vector<btVector3> vertices,
		vector<array<size_t, 3>> faces
	) :
		ShapeTemplate<VisualMeshShape, btEmptyShape>{},
		vertices{ vertices },
		faces{ faces }
	{}

	void setSafetyMargin(double margin);
	double getSafetyMargin() override;
	void addVertex(btVector3 v);
	void addFace(const array<size_t, 3>& f);
	const vector<btVector3>& getVertices()const;
	const vector<array<size_t, 3>>& getFaces()const;
};

class ShapeVisitor {
public:
	virtual void visit(CompoundShape* shape) = 0;
	virtual void visit(BoxShape* shape) = 0;
	virtual void visit(CylinderZShape* shape) = 0;
	virtual void visit(SphereShape* shape) = 0;
	virtual void visit(CapsuleZShape* shape) = 0;
	virtual void visit(ConvexHullShape* shape) = 0;
	virtual void visit(ConcaveTriangleMeshShape* shape) = 0;
	virtual void visit(MultiSphereShape* shape) = 0;
	virtual void visit(VisualMeshShape* shape) = 0;
	virtual ~ShapeVisitor() = default;
};
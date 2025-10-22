
#pragma once

#include "common.h"
#include "bullet.hpp"
#include "collision/shapes.h"


inline void to_json(json& j, const Shape& s);

class JsonConverter :public ShapeVisitor {
	json result;
public:
	json getResult() {
		return result;
	}

	virtual void visit(CompoundShape* shape) {
		result = {
			{"type", "Compound"},
			{"children", {}}
		};

		for (size_t i = 0; i < shape->getNumShapes(); i++) {
			result["children"].push_back({
				{"shape", *(shape->getShape(i))},
				{"trafo", shape->getShapeTrafo(i)},
				});
		}
	};

	virtual void visit(BoxShape* shape) {
		result = {
			{"type", "Box"},
			{"size", shape->getSize()},
			{"margin", shape->getSafetyMargin()}
		};
	}

	virtual void visit(CylinderZShape* shape) {
		result = {
			{"type", "CylinderZ"},
			{"radius", shape->getRadius()},
			{"height", shape->getHeight()},
			{"margin", shape->getSafetyMargin()}
		};
	}

	virtual void visit(SphereShape* shape) {
		result = {
			{"type", "Sphere"},
			{"radius", shape->getRadius()},
			{"margin", shape->getSafetyMargin()}
		};
	}

	virtual void visit(CapsuleZShape* shape) {
		result = {
			{"type", "CapsuleZ"},
			{"radius", shape->getRadius()},
			{"height", shape->getHeight()},
			{"margin", shape->getSafetyMargin()}
		};
	};

	virtual void visit(ConvexHullShape* shape) {
		result = {
			{"type", "ConvexHull"},
			{"margin", shape->getSafetyMargin()}
		};
		for (size_t i = 0; i < shape->getNumPoints(); i++) {
			result["points"].push_back(shape->getPoint(i));
		}
	};

	virtual void visit(MultiSphereShape* shape) {
		result = {
			{"type", "MultiSphere"},
			{"margin", shape->getSafetyMargin()}
		};
		for (size_t i = 0; i < shape->getNumSpheres(); i++) {
			result["spheres"].push_back(
				std::make_pair(
					shape->getSpherePosition(i),
					shape->getSphereRadius(i)
				)
			);
		}
	};

	virtual void visit(VisualMeshShape* shape) {
		result = {
			{"type", "VisualMesh"},
			{"vertices", shape->getVertices()},
			{"faces", shape->getFaces()}
		};
	};
};

inline void to_json(json& j, const Shape& s) {
	JsonConverter jc;
	const_cast<Shape&>(s).accept(jc);
	j = jc.getResult();
}



inline void from_json(const json& j, Holder<Shape>& hs) {
	string type = j.at("type");

	if (type == "Compound") {
		hs = hold<CompoundShape>();
		auto hcs = dynamic_cast<CompoundShape*>(hs.get());
		for (const auto& shape : j.at("children")) {
			hcs->addShape(shape.at("trafo"), shape.at("shape"));
		}
	}
	else if (type == "Box") {
		auto size = j.at("size");
		hs = hold<BoxShape>(size[0], size[1], size[2]);
		hs->setSafetyMargin(j.at("margin"));
	}
	else if (type == "CylinderZ") {
		hs = hold<CylinderZShape>(j.at("radius"), j.at("height"));
		hs->setSafetyMargin(j.at("margin"));
	}
	else if (type == "Sphere") {
		hs = hold<SphereShape>(j.at("radius"));
		hs->setSafetyMargin(j.at("margin"));
	}
	else if (type == "CapsuleZ") {
		hs = hold<CylinderZShape>(j.at("radius"), j.at("height"));
		hs->setSafetyMargin(j.at("margin"));
	}
	else if (type == "ConvexHull") {
		hs = hold<ConvexHullShape>();
		auto hch = dynamic_cast<ConvexHullShape*>(hs.get());
		for (const auto& point : j.at("points")) {
			hch->addPoint(point[0], point[1], point[2]);
		}
		// hch->optimize(); // this changes already optimized meshes and makes export import comparisons fail
		hch->setSafetyMargin(j.at("margin"));
	}
	else if (type == "MultiSphere") {
		MultiSphereShapeBuilder builder;
		for (const auto& sphere : j.at("spheres")) {
			builder.addSphere(sphere[0][0], sphere[0][1], sphere[0][2], sphere[1]);
		}
		hs = hold<MultiSphereShape>(builder.build());
		hs->setSafetyMargin(j.at("margin"));
	}
	else if (type == "VisualMesh") {
		hs = hold<VisualMeshShape>(j.at("vertices"), j.at("faces"));
	}
	else {
		throw runtime_error(string() + "unknown shape type: " + type);
	}
}


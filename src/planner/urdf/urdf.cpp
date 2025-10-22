
/* todo:
- wrap tinyxml2
- sanitize vector input
- check for duplicates, unresolved references and cycles
*/

#include "urdf.h"

using namespace tinyxml2;

string getAttribute(
	XMLElement* xml,
	const char* name,
	optional<string> defaultValue
) {
	const char* attribute = xml->Attribute(name);
	if (attribute) {
		return attribute;
	}
	else if (defaultValue) {
		return *defaultValue;
	}
	throw runtime_error(string()
		+ "<" + xml->Name() + "> (line "
		+ to_string(xml->GetLineNum()) + ") "
		+ "misses required attribute '" + name + "'");
}

XMLElement* getRequiredChild(
	XMLDocument* xml,
	const char* name
) {
	XMLElement* child = xml->FirstChildElement(name);
	if (child) {
		return child;
	}
	throw runtime_error(string()
		+ "document misses required child '" + name + "'");
}

XMLElement* getRequiredChild(
	XMLElement* xml,
	const char* name
) {
	XMLElement* child = xml->FirstChildElement(name);
	if (child) {
		return child;
	}
	throw runtime_error(string()
		+ "<" + xml->Name() + "> (line "
		+ to_string(xml->GetLineNum()) + ") "
		+ "misses required child '" + name + "'");
}

valarray<double> valarray3FromString(string s) {
	stringstream ss(s);
	double x, y, z;
	ss >> x >> y >> z;
	return valarray<double>{ x, y, z };
}

UrdfTrafo originFromXml(XMLElement* xml) {
	if (xml) {
		auto xyz = valarray3FromString(getAttribute(xml, "xyz", "0 0 0"));
		auto rpy = valarray3FromString(getAttribute(xml, "rpy", "0 0 0"));
		return UrdfTrafo{ xyz, rpy };
	}
	else {
		return UrdfTrafo{ {0, 0, 0}, {0, 0, 0} };
	}
}

UrdfRobot interpretXml(XMLDocument& xml) {

	auto robotXml = getRequiredChild(&xml, "robot");

	UrdfRobot robot{getAttribute(robotXml, "name")};

	for (
		auto link = robotXml->FirstChildElement("link");
		link != nullptr;
		link = link->NextSiblingElement("link")
		) {
		auto name = getAttribute(link, "name");
		vector<UrdfCollisionGeometry> collisionGeometries;

		for (
			auto collision = link->FirstChildElement("collision");
			collision != nullptr;
			collision = collision->NextSiblingElement("collision")
			) {
			auto originXml = collision->FirstChildElement("origin");
			auto origin = originFromXml(originXml);
			auto geometry = getRequiredChild(collision, "geometry");

			string type = "";
			valarray<double> size{ NaN, NaN, NaN };
			double radius = NaN;
			double length = NaN;
			string filename = "";
			valarray<double> meshScale{ 1, 1, 1 };

			if (auto box = geometry->FirstChildElement("box")) {
				type = "box";
				size = valarray3FromString(getAttribute(box, "size"));
			}
			else if (auto cylinder = geometry->FirstChildElement("cylinder")) {
				type = "cylinder";
				radius = std::stod(getAttribute(cylinder, "radius"));
				length = std::stod(getAttribute(cylinder, "length"));
			}
			else if (auto sphere = geometry->FirstChildElement("sphere")) {
				type = "sphere";
				radius = std::stod(getAttribute(sphere, "radius"));
			}
			else if (auto mesh = geometry->FirstChildElement("mesh")) {
				type = "mesh";
				filename = getAttribute(mesh, "filename");
				meshScale = valarray3FromString(getAttribute(mesh, "scale", "1 1 1"));
			}

			collisionGeometries.push_back(UrdfCollisionGeometry{
				type, origin, size, meshScale, radius, length, filename });
		}

		robot.links.insert(UrdfLink{ name, collisionGeometries });
	}

	for (
		auto joint = robotXml->FirstChildElement("joint");
		joint != nullptr;
		joint = joint->NextSiblingElement("joint")
		) {
		auto name = getAttribute(joint, "name");
		auto type = getAttribute(joint, "type");
		if (type == "revolute" || type =="continuous" || type == "prismatic"){
			robot.defaultJointSelection.push_back(name);
		}
		auto parent = getAttribute(getRequiredChild(joint, "parent"), "link");
		auto child = getAttribute(getRequiredChild(joint, "child"), "link");
		auto originXml = joint->FirstChildElement("origin");
		auto origin = originFromXml(originXml);

		auto axisXml = joint->FirstChildElement("axis");
		auto axis = axisXml ?
			valarray3FromString(getAttribute(axisXml, "xyz"))
			: valarray<double>{ 1, 0, 0 };

		auto limit = [&]() {
			if ("revolute" == type || "prismatic" == type) {
				auto lower = getAttribute(getRequiredChild(joint, "limit"), "lower", "0");
				auto upper = getAttribute(getRequiredChild(joint, "limit"), "upper", "0");
				return array<double, 2>{std::stod(lower), std::stod(upper)};
			}
			else {
				return array<double, 2>{NaN, NaN};
			}
		}();

		robot.joints.insert(UrdfJoint{
			name, type, parent, child, origin, axis, limit });
	}

	return robot;
}

UrdfRobot parseUrdf(const string& urdfSource) {
	XMLDocument xml;
	auto result = xml.Parse(urdfSource.c_str(), urdfSource.size());
	if (result != XML_SUCCESS) {
		throw runtime_error("could not interpret urdf source\n");
	}
	return interpretXml(xml);
}

UrdfRobot loadUrdfFile(const string& filename) {
	XMLDocument xml;
	auto result = xml.LoadFile(filename.c_str());
	if (result != XML_SUCCESS) {
		throw runtime_error(string()
			+ "could not read urdf file" + filename + '\n');
	}
	return interpretXml(xml);
}

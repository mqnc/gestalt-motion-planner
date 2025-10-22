
/* todo:
- wrap tinyxml2
- sanitize vector input
- check for duplicates, unresolved references and cycles
*/

#pragma once

#include "common.h"

#include "tinyxml2/tinyxml2.h"
#include "urdftypes.h"

string getAttribute(
	tinyxml2::XMLElement* xml,
	const char* name,
	optional<string> defaultValue = {}
);

tinyxml2::XMLElement* getRequiredChild(
	tinyxml2::XMLDocument* xml,
	const char* name
);

tinyxml2::XMLElement* getRequiredChild(
	tinyxml2::XMLElement* xml,
	const char* name
);

valarray<double> valarray3FromString(string s);

UrdfTrafo originFromXml(tinyxml2::XMLElement* xml);

UrdfRobot interpretXml(tinyxml2::XMLDocument& xml);

UrdfRobot parseUrdf(const string& urdfSource);

UrdfRobot loadUrdfFile(const string& filename);
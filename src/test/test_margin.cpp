
#include "stl.h"
#include "str.h"
#include "test_main.h"

#include "btTools.h"
#include "btBulletCollisionCommon.h"
#include "api.h"
#include "main/planner_headers.h"

TEST(test_margin, sphere) {
	{
		GestaltPlanner planner;
		planner.spawn("myObject", "../../../src/test/urdf/sphere.urdf");
		planner.render_scene("sphere test", "", "html", "sphere.html", true);
		planner.set_safety_margin("*", 0.3);
		planner.render_scene("sphere test", "", "html", "sphere_03.html", true);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_margin, complete) {
	{
		GestaltPlanner planner;
		//planner.spawn("myObject", "../../../src/test/urdf/ico.urdf");
		planner.spawn("myObject", "../../../src/test/urdf/shapes.urdf");
		planner.render_scene("margin test", "__world__", "html", "margins.html", true);
		planner.set_safety_margin("*", 0.1);
		planner.render_scene("margin test", "__world__", "html", "margins_01.html", true);
		planner.set_safety_margin("*", 0.2);
		planner.render_scene("margin test", "__world__", "html", "margins_02.html", true);
		planner.set_safety_margin("*", 0.3);
		planner.render_scene("margin test", "__world__", "html", "margins_03.html", true);
		planner.set_safety_margin("*", 0.4);
		planner.render_scene("margin test", "__world__", "html", "margins_04.html", true);
		planner.set_safety_margin("*", 0.5);
		planner.render_scene("margin test", "__world__", "html", "margins_05.html", true);
		planner.set_safety_margin("*", 0.6);
		planner.render_scene("margin test", "__world__", "html", "margins_06.html", true);
		planner.set_safety_margin("*", 1.0);
		planner.render_scene("margin test", "__world__", "html", "margins_10.html", true);

		planner.set_safety_margin("myObject.box", 0.1);
		planner.set_safety_margin("myObject.sphere", 0.2);
		planner.set_safety_margin("myObject.cylinder", 0.3);
		planner.set_safety_margin("myObject.mesh", 0.4);
		planner.render_scene("margin test", "__world__", "html", "margins_mixed.html", true);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}


TEST_MAIN


#include "stl.h"
#include "str.h"
#include "test_main.h"

#include "api.h"
#include "btBulletCollisionCommon.h"
#include "btTools.h"
#include "main/planner_headers.h"

TEST(test_compound, margin_change)
{
    {
        GestaltPlanner planner;
        planner.spawn("b1", "../../../src/test/urdf/box.urdf");
        planner.spawn("b2", "../../../src/test/urdf/box.urdf", "", Pose { 1.1 });

        planner.render_scene("margin_change test", "", "html", "boxes.html", true);
        EXPECT_TRUE(planner.check_clearance());

        planner.set_safety_margin("*", 0.3);

        planner.render_scene("margin_change test", "", "html", "boxes2.html", true);
        EXPECT_FALSE(planner.check_clearance());
    }
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
    btDumpMemoryLeaks();
#endif
}

TEST_MAIN

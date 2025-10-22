import os
import sys

import PyGestaltPlanner as gp

print("                    --------")
print("                    Planners")
print("                    --------")
print()

mock = gp.GestaltPlanner(command_log_file="", show_logo=False)
planners = mock.get_planner_info()
for name in planners:
    planner = planners[name]
    mt = "multithreaded" if planner.multithreaded else "singlethreaded"
    d = "directed" if planner.directed else "undirected"
    print(name + " (" + mt + ", " + d + ")")
    for p in planner.params:
        param = planner.params[p]
        print("\t" + p + " (default = " + param.defaultValue, end="")
        if param.rangeSuggestion != "":
            print(", suggested range = " + param.rangeSuggestion, end="")
        print(")")
    print()

print("                    ------------")
print("                    Complete API")
print("                    ------------")

for thing in dir(gp):
    if thing[0] != "_":
        print(thing + ":\n")
        cls = getattr(gp, thing)
        members = dir(cls)
        for member in members:
            if member == "__init__" or member[0] != "_":
                type_name = type(getattr(cls, member)).__name__
                if type_name == "instancemethod":
                    print("\t" + getattr(cls, member).__doc__.replace("(", "(\n\t\t").replace(", ", ",\n\t\t").replace(
                        ")", "\n\t)"))
                else:
                    print("\t" + member + " (" + type_name + ")\n")

print("                    ----")
print("                    Demo")
print("                    ----")

sys.stdout.flush()

planner = gp.GestaltPlanner(command_log_file="my.example.log.cpp", show_logo=True)

planner.cache_file(file_name="floor.urdf",
                   raw_content="""<?xml version="1.0"?><robot name="floor"><link name="floor">
	<collision><origin xyz="0 0 -0.02" rpy="0 0 0" />
	<geometry><box size="1 1 0.04" /></geometry>
	</collision></link></robot>""")

planner.cache_file(file_name="sword.urdf",
                   raw_content="""<?xml version="1.0"?><robot name="sword"><link name="sword">
	<collision><origin xyz="0.02 0 -0.28" rpy="0 0 0" />
	<geometry><cylinder radius="0.02" length="0.6" /></geometry>
	</collision></link></robot>""")

planner.cache_file(file_name="pillar.urdf",
                   raw_content="""<?xml version="1.0"?><robot name="pillar"><link name="pillar">
	<collision><origin xyz="0 0 0.5" rpy="0 0 0" />
	<geometry><cylinder radius="0.05" length="1" /></geometry>
	</collision></link></robot>""")

planner.spawn(
    object_id="floor",
    description_file="floor.urdf",
)

planner.spawn(object_id="robot", description_file="models/ur5e/ur5e.urdf", config_file="models/ur5e/ur5e.yaml")

# the robot has two base links :/
planner.create_collision_ignore_group(name="floor_base", members=["robot.base_link", "floor.floor"])

planner.spawn(object_id="sword", description_file="sword.urdf", parent_object_id="robot", parent_link="ee_link")

planner.create_collision_ignore_group(name="hand_sword", members=["robot.wrist_2_link", "robot.wrist_3_link", "sword"])

r = 0.4

planner.spawn(object_id="pillar1", description_file="pillar.urdf", pose=gp.Pose(x=r, y=r))

planner.spawn(object_id="pillar2", description_file="pillar.urdf", pose=gp.Pose(x=-r, y=r))

planner.spawn(object_id="pillar3", description_file="pillar.urdf", pose=gp.Pose(x=r, y=-r))

planner.spawn(object_id="pillar4", description_file="pillar.urdf", pose=gp.Pose(x=-r, y=-r))

planner.set_safety_margin(object_id="*", margin=0.012)

pi = 3
start = [0, -1.2, 2, -0.5, -1.5 * pi, -6.2]
target = [pi, 1.2 - pi, -2, 0.5 - pi, 1.5 * pi, 6.2]

path = planner.plan_path(object_id="robot",
                         target_joint_positions=[target],
                         start_joint_positions=start,
                         maxChecks=50000,
                         planner="RRTConnect",
                         planner_params={"range": "1.5"})

dt = 0.04

interp = planner.interpolate(
    waypoints=path,
    dt=dt,
    max_velocity=[2] * 6,
    max_acceleration=[10] * 6,
    max_jerk=[10] * 6,
)

reverse_path = path[::-1]
rev_smooth = planner.smoothen_path(
    object_id="robot",
    waypoints=reverse_path,
    dt=dt,
    max_velocity=[2] * 6,
    max_acceleration=[10] * 6,
    max_jerk=[10] * 6,
)

output_file = "demo.log.html"

planner.render_animation(title="Demo Path",
                         active_object="robot",
                         trajectory=interp + rev_smooth,
                         dt=dt,
                         format="html",
                         output_file=output_file)

print("\nCheck out " + output_file + "!\n")


# Gestalt Motion Planner

The Gestalt Motion Planner is a collision-free trajectory planner for robotic manipulators. Setup a scene, run path planning jobs, optimize the resulting trajectories and feed them to a robot.

## Project Status

The planner is used in industrial applications at Gestalt Automation GmbH. The author is no longer working there and maintaining this planner as a hobby project (with permission).

### Features

* C++ and optional Python bindings, JSON-RPC wrap of the complete API for easy interface creation
* support of URDF with STL models
* path finding using [OMPL](https://ompl.kavrakilab.org) including all its planners
* collision avoidance using [bullet3](https://github.com/bulletphysics/bullet3) for collision detection, supports all URDF primitives (spheres, cylinders and boxes) and arbitrary triangle meshes, including concave ones
* path smoothing using [ruckig](https://github.com/pantor/ruckig) for jerk-limited trajectory generation
* checking of robot trajectories for collisions and kinematic feasibility (range, speed, acceleration and jerk limits of joints)
* configurable safety margin per robot or obstacle
* reasonably deterministic: given the same build, hardware, selection of a deterministic planner and no timeouts, the results are reproducible
* joint constraints which can be used to constrain the end-effector so it does not spill liquids while allowing for very efficient path planning
* collision ignore groups for flexible collision filtering
* comprehensive html debug visualizations for states and trajectories

### Missing

* strict input validation: many common errors are caught, unforeseen edge cases can cause a crash
* inverse kinematics: the planner operates entirely in joint space, ik for target poses has to be implemented by the user
* usable debug logs: right now, the planner creates a C++ file with the history of function calls which can be compiled into the planner itself and debugged in an IDE which is arguably somewhat unelegant

## License

This project is released under the [PolyForm Noncommercial License 1.0.0](https://polyformproject.org/licenses/noncommercial/1.0.0/). For a commercial license contact [Gestalt Automation GmbH](https://www.gestalt-automation.com/en/contact).

## Quickstart

### Setup

If you run some recent version of Ubuntu and have docker installed, you can just execute `sh docker_build_and_test.sh` and it will spit out static libs for C++ and python modules into a build folder. If not, you can still explore the Dockerfile for installation instructions.

### API

For a complete **C++** API reference, refer to `src/planner/api.h` and `src/planner/plannermethods.inc`. For usage demonstrations, refer to `src/test/test_api.cpp.`

For a complete **Python** API reference, run `src/bindings/python/demo.py` and check the output. For a basic usage demonstration, check its source.

## How to Read this Documentation

It is probably the best idea to read the whole thing from top to bottom. Many quirks are only mentioned once where they are most relevant, although they are also a bit relevant for other aspects.

## Concepts

### Robot Hierarchy

To the planner, everything is a robot. Rigid objects are robots with a single link and no joints. The scene is represented using the [Scene Graph](https://en.wikipedia.org/wiki/Scene_graph#Scene_graphs_in_games_and_3D_applications) pattern. A node represents a robot link and an edge represents a joint (or an attachment, which is seen as a fixed joint). The root node represents the world coordinate system, every other node (link) must have one parent and can have an arbitrary amount of children. Cycles like objects that have two gripper links as parents are not possible, the object has to be attached to only one of the gripper links in that case. A link's pose is relative to its parent and the link will move with the parent. Collisions between parent and child are automatically ignored.

### Robot Description

Robots are described using the [URDF file format.](http://wiki.ros.org/urdf/XML) in combination with an additional yaml file for further specifications like the order of the actuated joints (which tuple value maps to which joint), collision ignore groups etc. See `models/ur5e/ur5e.yaml` for an example.

### Safety Margin

Every robot can have an individual safety margin, inflating it artificially in order to account for inaccuracies of the robot mount and also to prevent missed collisions due to motion sampling. A safety margin of 5 cm means that a gap of 5 cm between the robot and another object (i.e. that object's safety margin) is seen as a collision.

### Collision Ignore Groups

Objects that should not be checked for collision among each other can be grouped in collision ignore groups. Each object can be in multiple groups. Collisions between parent and child are automatically ignored. Any object that is currently not actuated will not be checked for collisions with other passive objects. All those objects are automatically members of the `__passive__` collision ignore group.

### Planning Domain

All planning is done in joint space. This avoids the effort of having to compute the inverse kinematics for every step and it also avoids any trouble with singularities. Since the planner does not need inverse kinematics computation at all, it is robot-agnostic. For kinematic constraints, like holding the end-effector upright for handling liquids, check out [constraints](#constraints).

When you actually need linear motion in Cartesian space, you probably do not need path planning anyway because you likely want to move along a predefined line. In this case, you can use the interpolator to generate your trajectory in Cartesian space, compute the inverse kinematics manually at each sample and just check the result for collisions using the planner.

### Planning Workflow

Trajectory planning is done in the following steps:

* **Exploration:** This searches for a path from start to target. The path can (and probably will) be very unoptimal and zigzag. It is going to be improved in the next steps, the purpose of this step is to find any valid solution. If found, it consists of a list of arbitrarily far apart waypoints. Point to point motion (straight lines in joint space) is collision-free.
* **Simplifying:** This step checks if waypoints can be skipped. It checks if a direct motion (in joint space) from start to end point of a trajectory is possible. If not, it recursively checks the first and then the second half of the trajectory.
* **Tightening:** This step iterates through the found waypoints and tries to bring each waypoint closer to the average of its neighbors, while continuously making sure that no collisions are introduced. This can be pictured as threading a rubber band through all waypoints and tightening it, so that the path gets shorter while tightly curving around collisions.
* **Smooth Interpolation:** Here a heuristic (similar to [Akima splines](https://en.wikipedia.org/wiki/Akima_spline)) is employed to generate desired velocity vectors for the waypoints. Smooth spline curves are then created to connect the waypoints, considering the velocity vectors. If new collisions are introduced, the velocities are reduced until they are zero, which might result in straight lines again in an edge case.

### Constraints

In order to hold something upright while moving, constraints were introduced. Constraints keep the dot product of the joint angles and arbitrary vectors constant. Consider the following configuration:

![](docs/constraints.jpg)

If you keep both `q5` as well as the sum `q2 + q3 + q4` constant, the flask can only stay perfectly level. This can be specified via the following list of constraints: `[[0, 0, 0, 0, 1, 0], [0, 1, 1, 1, 0, 0]]`, meaning

```
0*q1 + 0*q2 + 0*q3 + 0*q4 + 1*q5 + 0*q6 = const.
0*q1 + 1*q2 + 1*q3 + 1*q4 + 0*q5 + 0*q6 = const.
```
Additionally, you can also define `q1 - q6` to remain constant (`[1, 0, 0, 0, 0, 0, -1]`), which would also fix the rotation of the flask about the vertical axis and thus completely fix its orientation in space during motion.

Pay attention to the signs! For instance, if `q5` is rotated by 180 ° compared to the sketch (which would hold the flask upside down), `q1` and `q6` would no longer be antiparallel but parallel and you would have to define `[1, 0, 0, 0, 0, 0, 1]` as a constraint in order to prevent rotation about the up axis.

Pay another attention to the fact that both start and target of your planning requests must fulfill your constraints in the same way. For instants, if you specify the aforementioned constraint `[0, 1, 1, 1, 0, 0]`, you can not move from `q = [0°, 0°, 0°, -90°, 0°, 0°]` to `q = [0°, 0°, 0°, 270°, 0°, 0°]`, although both configurations have the same end-effector orientation. The robot would have to do turn the wrist1 joint by 360°, which would violate the constraint (and spill the liquid). This might seem obvious in this obvious example, but there were lots of *"Hey, can we loosen the constraints? My thing doesn't work."* requests. Luckily, the planner will issue a very descriptive error message in this case.

You can also specify arbitrary constraints like `[1, -2, 3, -4, 5, -6.7]` but there is little point to that.

This mechanism of forcing linear combinations of joint angles to be constant in order to fix certain Cartesian properties of the end-effector exploits the joint configuration that most industrial 6-dof robots fortunately have. It would for instance not work with Kinova's [JACO](https://www.kinovarobotics.com/product/jaco) arm, due to its unconventional wrist design.

Another condition for the constraints concept to work is that the robot is mounted horizontally (possibly upside-down). It can also work in a 90° wall mount case but that puts some restrictions on the gripper posture and is less flexible. It does not work with arbitrarily tilted robot base orientations and it also does not allow arbitrary end-effector orientations.

Although all these conditions seem pretty limiting, they should be fulfilled in most real-world applications. And on the flip side, introducing more constraints actually makes every step of the planning phase more performant, since each constraint reduces the dimensionality of the problem by one. The implementation also makes use of this fact. However, the search space is reduced by cutting off massive parts of it, which can make finding a solution less likely.9

### NaN for Unchanged Values

If you want to change only some values in a pose or a joint vector, set the other ones to NaN. For example, setting the base pose of a robot to `[x=1, y=2, z=NaN, qx=NaN, qy=NaN, qz=NaN, qw=NaN]` will only change the x and y components of its pose. Similarly, setting the joint values to `q = [NaN, NaN, NaN, 1, 2, 3]` will only change the last three joints. This probably changes to `std::optional` in the future though as it is somewhat semantically unintuitive which is never good. Note that this convention is only used for poses and joint values, not for arbitrary parameters. Note also that when changing poses, you can only set either all values of the quaternion part or none of them.

### Logging

If you specify a name for the `command_log_file` when constructing the planner, a cpp will be created which logs any method call that was invoked on the planner. This can be compiled with the planner library and then debugged.

### Rendering

The planner lets you render situations and trajectories as html files. Passive objects will be teal with a plum collision margin, active objects are yellow with a blue collision margin. Only collision hulls are rendered, not the visual representations of links that are stored in the URDF files.

## API in Sensical Order

Note that the API is evolving and this documentation might be outdated. `src/planner/plannermethods.inc` and the output of demo.py are reliable references, generated from the code. In python, keyword arguments (kwargs) have to be used. This is so that the API can be extended later without breaking your code. If you are working in C++20 (which you should be, it's awesome), you can also kind of use [named parameters](https://pdimov.github.io/blog/2020/09/07/named-parameters-in-c20/), the wrapper functions taking parameter structs are automatically generated for each method.

### Instantiating the Planner

The planner is an object with state and you need to construct one to hold a planning scene.

### Spawning / Removing Things

Call `spawn` for creating robots and static objects (which are also considered robots) in the scene. Note that the planner uses a file cache and loading the same file multiple times is cheap. However, if the file changes, the planner will not notice and spawn from the cache. Files can be cached manually using `cache_file`, for instance if the planner runs on another machine and does not actually have the file in its file system. Using `remove` you can remove objects again.

### Positioning Things

Call `set_base_pose` for changing the base pose of a robot. It can also attach the robot to a different parent. If it is called with all pose values set to NaN, it can change the parent without changing the pose in world coordinates. This is useful for dynamic attachments, like something picking something up. Collisions between parent and child are automatically ignored. Use `set_joint_positions` to change the joint values and thus the posture of the robot. Note that neither `set_base_pose` nor `set_joint_positions` perform any kind of planning or collision detection, they will both happily move robots into each other. Note that you can use `NaN` for individual joint values to leave them unchanged.

### Collision Configuration

Use `set_safety_margin` for setting the safety margin around a robot. A safety margin of 5 cm means that a gap of 5 cm between the robot and another object (i.e. that object's safety margin) is seen as a collision. For ignoring collisions between links, you can put them into collision ignore groups using `create_collision_ignore_group`. Groups can be deleted using `delete_collision_ignore_group`. All existing collision ignore groups can be listed using `get_collision_ignore_groups`.

### Path Planning

Finally! The `plan_path` method explores the configuration space of a robot, trying to find a collision-free connection from start to target. This uses the [Open Motion Planning Library](https://ompl.kavrakilab.org/index.html). The result is a list of waypoints between which a linear motion in joint space is collision-free, or an empty list if no path was found. Be aware that planning a motion restores the start pose when it is done. So if you actually perform the motion on the real robot, you need to update the scene using `set_joint_positions` additionally.

OMPL supports [lots of different planning algorithms](https://ompl.kavrakilab.org/planners.html) with lots of parameters. You can use `get_planner_info` to get a list of all supported planners and their parameters. This list is also printed when you run demo.py. In practice, RRT connect outperformed all other planners by a large margin in all our scenarios.

### Path Tools

The method `interpolate` can be used to do linear interpolation of arbitrary parameters. It converts a set of waypoints into a trajectory with limited velocity, acceleration and jerk, stopping at each waypoint. Since `plan_path` finds the waypoints in a way so that linear joint space motion between them is collision free, `interpolate` does not require knowledge of the scene, for interpolating in a collision-free way. Furthermore, you can use `interpolate` to generate a linear trajectory in Cartesian space, compute the inverse kinematics manually at each sample and check the result for collisions (see Path Validation).

The method `simplify_path` checks if a direct linear motion (in joint space) from the start to the end is possible. If so, it discards all intermediate waypoints. If not, it recursively calls itself on the first half and the last half of the waypoint list. The always true base case is if the to-be-checked waypoints are neighbors anyway.
This scheme was chosen as a compromise between coverage and performance, as checking for shortcuts between each waypoint pair scales badly with the number of waypoints.

The method `tighten_path` iterates through a list of waypoints and tries to bring each waypoint closer to the average of its neighbors, while continuously making sure that no collisions are introduced. This can be pictured as threading a rubber band through all waypoints and tightening it, so that the path gets shorter while tightly curving around collisions. Since `plan_path` usually results in very zigzagish paths, this method is already built in if you set the `tighten` parameter in `plan_path` to `true`.

Finally, you can lay a smooth curve through a list of waypoints, using `smoothen_path`. It creates a spline through the waypoints while making sure that no new collisions are introduced.

If you sampled your own trajectory (e.g. by generating each point via inverse kinematics), you can use `time_parameterize_path` to make sure that all kinematic limits (velocity, acceleration and jerk) in joint space are obeyed. The function will measure the maximum overshoot and resample accordingly.

### Path Validation

The `check_kinematic_feasibility` method can be used to see if maximum velocity, acceleration and jerk (all in joint space) are not overshot within the trajectory. For collision checking, there are `check_clearance` which checks all waypoints in random order and returns `false` when it encounters the first collision, and `find_collisions` which checks the complete trajectory and reports every collision, so it is much slower but yields more detailed information.

### Debugging

There are two main mechanisms for debugging: logging and rendering. When instantiating the planner, you can pass it a filename, in which every method call will be logged. This log file is also valid C++ code that can be compiled with the planner and then you can for instance debug your colleague's python session in C++ with a proper IDE.

The function `get_version` returns a string containing the planner version, including some commit information. Its main purpose is to see if the version is matching the version of the log you are running, if you are running a log. But you can also check if you are using the same version as someone else who tries to help you debugging but somehow different things seem to happen on their computer.

As the log gets quite big quite fast, you can call `command_log_comment` in between, which will be printed into the log file and then you can search for that string there.

Use `render_scene` and `render_animation` to inspect the scene you have created and the trajectories you have generated. Passive objects will be teal with a plum collision margin, active objects are yellow with a blue collision margin. Only collision hulls are rendered, not the visual representations of links that are stored in the URDF files. Collisions are also visualized and the colliding links are printed in the browser console. Use the mouse for navigating the scene. If you are viewing an animation, you can use space to start/stop the animation, left and right arrow keys to step one frame back or forth in time and up and down arrow keys to go to the beginning or the end of the animation. Note that the safety margin of convex triangle meshes have lots of gaps. This is due to the fact that for rendering, the triangles are just shifted outwards but the gaps are not filled for performance reasons. However, collision checks work in a different way and the meshes are inflated properly without gaps.

### Miscellaneous Stuff

Using `reset`, you can delete the complete scene. It is the same as creating a new planner instance. It also clears the file cache.

If you want to actuate a different set of joints than what is specified in the yaml config for a robot, you can use `select_joints` for selecting them.

If you have to create a new interface (http/ROS/ZeroMQ/...), the quickest way to do it would be to wrap the `json_call` method. It implements the [JSON-RPC 2.0 Specification](https://www.jsonrpc.org/specification) and can call any other planner method.

## API in Alphabetical Order

Note that the API is evolving and this documentation might be outdated. `src/planner/plannermethods.inc` and the output of demo.py are reliable references, generated from the code. In python, keyword arguments (kwargs) have to be used. This is so that the API can be extended later without breaking your code. If you are working in C++20, you can also kind of use [named parameters](https://pdimov.github.io/blog/2020/09/07/named-parameters-in-c20/), the wrapper functions taking parameter structs are automatically generated for each method. Note that parameters are not explained per function but [here](#parameters).

```py
GestaltPlanner:
        __init__(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                command_log_file: str = 'last_run.log.cpp',
                show_logo: bool = True
        ) -> None
        cache_file(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                file_name: str,
                raw_content: str,
                is_base64: bool = False
        ) -> None
        check_clearance(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str = '__world__',
                trajectory: List[List[float]] = []
        ) -> bool
        check_kinematic_feasibility(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                trajectory: List[List[float]],
                dt: float,
                max_velocity: List[float],
                max_acceleration: List[float],
                max_jerk: List[float]
        ) -> bool
        command_log_comment(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                comment: str
        ) -> None
        create_collision_ignore_group(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                name: str,
                members: List[str]
        ) -> None
        delete_collision_ignore_group(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                name: str
        ) -> None
        export_state(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str = '',
                indent: bool = True
        ) -> str
        find_collisions(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str = '__world__',
                trajectory: List[List[float]] = []
        ) -> List[PyGestaltPlanner.Collision]
        get_collision_ignore_groups(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                active_object: str = ''
        ) -> Dict[str,
                Set[str]]
        get_joint_selection(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str
        ) -> List[str]
        get_planner_info(
                self: PyGestaltPlanner.GestaltPlanner
        ) -> Dict[str,
                PyGestaltPlanner.PlannerInfo]
        get_version(
                self: PyGestaltPlanner.GestaltPlanner
        ) -> str
        interpolate(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                waypoints: List[List[float]],
                dt: float,
                max_velocity: List[float],
                max_acceleration: List[float] = [],
                max_jerk: List[float] = [],
                safety_factor: float = 1.0
        ) -> List[List[float]]
        json_call(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                data: str
        ) -> str
        plan_multistep_path(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                sets_of_target_joint_positions: List[List[List[float]]] = [],
                start_joint_positions: List[float] = [],
                waypoint_suggestions: List[List[float]] = [],
                jiggle: float = 1e-06,
                constraints: List[List[float]] = [],
                constraint_tolerance: float = 1e-06,
                planner: str = 'RRTConnect',
                planner_params: Dict[str,
                str] = {},
                max_checks_per_step: int = 5000,
                timeout_per_step: float = -1.0,
                random_seed: int = 0,
                tighten: bool = True
        ) -> List[List[List[float]]]
        plan_path(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                target_joint_positions: List[List[float]] = [],
                start_joint_positions: List[float] = [],
                waypoint_suggestions: List[List[float]] = [],
                jiggle: float = 1e-06,
                constraints: List[List[float]] = [],
                constraint_tolerance: float = 1e-06,
                planner: str = 'RRTConnect',
                planner_params: Dict[str,
                str] = {},
                maxChecks: int = 5000,
                timeout: float = -1.0,
                random_seed: int = 0,
                simplify: bool = True,
                tighten: bool = True
        ) -> List[List[float]]
        remove(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str
        ) -> None
        render_animation(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                title: str = 'debug',
                active_object: str = '__world__',
                trajectory: List[List[float]] = [],
                dt: float = 1.0,
                format: str = 'json',
                output_file: str = ''
        ) -> str
        render_scene(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                title: str = 'debug',
                active_object: str = '__world__',
                format: str = 'json',
                output_file: str = '',
                probe_margins: bool = False
        ) -> str
        reset(
                self: PyGestaltPlanner.GestaltPlanner
        ) -> None
        select_joints(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                joints: List[str] = []
        ) -> None
        set_base_pose(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                pose: PyGestaltPlanner.Pose = <PyGestaltPlanner.PoseUpdate object at 0x7f861f34aa70>,
                parent_object_id: str = '',
                parent_link: str = '__root__'
        ) -> None
        set_joint_positions(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                joint_positions: List[float] = []
        ) -> None
        set_safety_margin(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str = '*',
                margin: float = 0.0
        ) -> None
        simplify_path(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                waypoints: List[List[float]],
                constraints: List[List[float]] = []
        ) -> List[List[float]]
        smoothen_path(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                waypoints: List[List[float]],
                dt: float,
                max_velocity: List[float],
                max_acceleration: List[float],
                max_jerk: List[float],
                constraints: List[List[float]] = [],
                quick: bool = True
        ) -> List[List[float]]
        spawn(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                description_file: str,
                config_file: str = '',
                pose: PyGestaltPlanner.Pose = <PyGestaltPlanner.Pose object at 0x7f861f34b270>,
                pose_type: str = 'static',
                joint_positions: List[float] = [],
                parent_object_id: str = '__world__',
                parent_link: str = '__root__',
                encapsulate_meshes: bool = False
        ) -> None
        tighten_path(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                object_id: str,
                waypoints: List[List[float]],
                constraints: List[List[float]] = []
        ) -> List[List[float]]
        time_parameterize_path(
                self: PyGestaltPlanner.GestaltPlanner,
                *,
                trajectory: List[List[float]],
                dt: float,
                max_velocity: List[float],
                max_acceleration: List[float],
                max_jerk: List[float]
        ) -> List[List[float]]
```

### Parameters

|  |  |  |
| --- | --- | --- |
| `active_object` | — | When rendering, the active object is highlighted in yellow and only collisions that involve the active object are shown. When listing collision ignore groups, all inactive objects are put in the `__passive__` group, as they are during collision checks. |
| `command_log_file` | — | Every method call on the planner will be logged in this file so the session can be debugged in C++ later. |
| `config_file` | — | the yaml config file for the robot |
| `constraint_tolerance` | — | Allow the dot product of a constraint and the joint values to be off by this much without rejecting; useful if you start in a position that is a measurement from the real robot |
| `constraints` | — | see [constraints](#constraints) |
| `data` | — | string containing a [JSON-RPC 2.0](https://www.jsonrpc.org/specification) request |
| `description_file` | — | the urdf file for the robot |
| `dt` | — | the sampling time |
| `encapsulate_meshes` | — | This was intended for encapsulating meshes in much simpler hulls for speeding up the collision detection but it inflated the meshes too much and other issues had higher priority. It's totally gonna happen one day tho! |
| `format` | — | "json" or "html" |
| `jiggle` | — | If the path planning starts inside a collision (for example if a robot is picking something up that is standing on a table), each joint will be jiggled by this much in both directions to see if the contact can be broken and the planning can start from there. |
| `joint_positions` | — | initial joint positions for the robot; the order of the joints is specified in the yaml config file' |
| `joints` | — | must contain strings that refer to joint ids as defined inside the URDF file |
| `max_acceleration` | — | Acceleration limit for each joint |
| `max_jerk` | — | Jerk limit for each joint |
| `max_velocity` | — | Velocity limit for each joint |
| `maxChecks` | — | maximum number of collision checks for the planning attempt (ball park) |
| `members` | — | List of links among which collisions should be ignored; Each item can be a specific link of a specific robot in the form of `"myRobot.myLink"`, the `object_id` of a robot to add all links of that robot, or the name of another collision ignore group to add all links in that group to the new group (permanently, even if the old group is deleted). |
| `object_id` | — | When spawning, a unique id must be given to each robot for referring to her later. In the other functions, this is how you select that robot. |
| `parent_link` | — | id of the link (in the parent) that this robot is attached to; set to `"__root__"` to attach it to the parent's root link |
| `parent_object_id` | — | id of the parent object that a robot is attached to; leave empty to not change the parent; set to `"__world__"` if the robot should not have a parent (meaning the world is its parent (which is very poetical)) |
| `planner_params` | — | the parameters for configuring the used planning algorithm, call `get_planner_info` for a list of all possible parameters for each planner or check the output of `demo.py`. |
| `planner` | — | the path finding algorithm that OMPL uses, call `get_planner_info` for a list of all planners or check the output of `demo.py`. |
| `pose_type` | — | This is only for semantic consistency with the Gestalt Simulator (different project); can be `"static"`, `"virtual"` or `"hypothesis"`. Basically, robots with virtual poses are ignored, anything else is treated as normal robots. |
| `pose` | — | pose of the robot base in relation to its parent; an object of the `Pose` class, which defaults to `Pose(x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1)` or an object of the `PoseUpdate` class, which defaults to `PoseUpdate(x=NaN, y=NaN, z=NaN, qx=NaN, qy=NaN, qz=NaN, qw=NaN)` (see [here](#nan)). So `Pose(x=1)` means 1 m next to the origin in x direction with default orientation, `PoseUpdate(x=1)` means 1 m away from the current pose in x direction, keeping the current orientation. |
| `probe_margins` | — | deprecated |
| `random_seed` | — | initialize the random sampler with this seed; Same seeds will produce the same trajectory under the same circumstances (deterministic planning), except if a multithreaded planner is used. `get_planner_info` tells you which planners are multithreaded. |
| `safety_factor` | — | `max_velocity`, `max_acceleration` and `max_jerk` will be scaled by this factor. |
| `show_logo` | — | Display a little ascii logo for the planner in the console log. |
| `tighten` | — | automatically call `tighten_path` after `plan_path`, highly recommended |
| `title` | — | title for the browser window of the export, was originally intended to distinguish different animations in the same html file but so far only one animation can be shown per file and screw [YAGNI](https://en.wikipedia.org/wiki/You_aren%27t_gonna_need_it). |
| `trajectory` | — | A list of waypoints in joint space |
| `waypoints_suggestions` | — | When specified, the random sampler that is used by the state space explorer will output this list of waypoints first and also every now and then during random sampling, use this to guide the path. |

### Major Dependencies

[OMPL](https://ompl.kavrakilab.org/) is an abstract pathfinding library. You give it a set of start values and a set (or multiple sets) of target values and it tries to connect the two, exploring the parameter space and asking a callback if certain positions and transitions between positions are ok or not. It does not care about the scene or what the parameters do.

[BulletPhysics](https://github.com/bulletphysics/bullet3) requires a description of the scene in terms of collision primitives and can detect collisions between them.

[Ruckig](https://github.com/pantor/ruckig) is invoked in order to interpolate the waypoints of the found path and turn them into a smooth curve.

## Working on the Planner

I will just write in I-form now, as it's just you and me here. I originally wrote this section so my colleagues could work on the planner while I was in parental leave. I will just leave it in, it serves as a nice entry point to the code.

I tried to comply to the rules of [Clean Architecture](https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html) but it has proven to be very difficult as the planner has grown with the project requirements and needed big refactories every once in a while. Old concepts became irrelevant, leaving their traces, and new concepts were woven in, despite the planner not wasing designed for them. All under time pressure. You know what it's like. I hope you will find your way through the code.

### Navigating the Source Code

I recommend VSCode with C++ Extensions and the CMake Tools extension for that. Your entry point is `src/planner/api.h`, from there you can (and absolutely should) use the "Go to Definition" [F12] and "Go to Implementations" [Ctrl+F12] functions to navigate everywhere and dig through the code. I wanted to comment everything with lots of in-depth explanations but then I saw [this](https://youtu.be/2a_ytyt9sf8?t=603) and who am I to argue with Uncle Bob. Also, my code totally documents itself 😉

### Code Generation

The planner has lots of interfaces: traditional C++ methods, methods for C++20 parameter structs, a JSON-RPC interface and python bindings. Normally, one would have to keep them all in sync manually whenever the API changes a bit. This proved to be incredibly inconvenient. Hence, I wrote crawlers that scan through the code, collect interface information and generate everything automatically. These generators are located in `src/codingassistants`. They scan the files in `src/planner/main` and produce `src/planner/plannermethods.inc`, `src/planner/main/jsondispatch.inc` and `src/bindings/bindings.cpp`. CMake will call the generators automatically but sometimes it does not notice a change and you need to delete the generated files and they get regenerated.

Since the generators are hand-crafted parsers that only know as much C++ syntax as necessary, it might happen that you change an API method in a way that the parser doesn't understand. It will then give you the very wholistic error message "something's wrong with ...". Try to see if your syntax is somehow noticably different from mine. If you really need something that the parser cannot understand, your best bet is probably to deactivate the automatic code generation: In `src/planner/CMakeLists.txt`, commentate the `extract_members` call. In `src/bindings/python/CMakeLists.txt`, commentate the `generate_python_bindings` call. Trying to tweak the extractors instead is probably wasted time unless you are already familiar with PEG parsing and want to dig into my uncommented parser spaghetti code.

### Extending the Functionality

If you just want to create some new API methods, that shouldn't be too difficult. Copy one of the files in `src/planner/main` (`select_joints.cpp` is a good candidate), rename it to `my_awesome_function.cpp`, edit it to your liking, add the file name to the list `set(PLANNER_SOURCES ...` inside `src/planner/CMakeLists.txt` and you're done. Function headers and all bindings are generated automagically.

**Happy Planning!**

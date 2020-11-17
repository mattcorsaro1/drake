In this example, a [Robotiq 3-Finger Adaptive Gripper](https://robotiq.com/products/3-finger-adaptive-robot-gripper) grasps a box or cylinder using one of the gripper's five main grasp types. Instead of modeling the underactuated fingers using springs, this example uses the hybrid model defined in [Technical Report: Use of Hybrid Systems to model the RobotiQ Adaptive Gripper](https://www.researchgate.net/profile/Giulia_Franchi2/publication/278158284_Use_of_Hybrid_Systems_to_model_the_RobotiQ_Adaptive_Gripper/links/557ca3a708aec87640db4f0d/Use-of-Hybrid-Systems-to-model-the-RobotiQ-Adaptive-Gripper.pdf) by Giulia Franchi and Kris Hauser. A single binary closing command can be sent to the gripper. A sequence of several joint goals are sent to the hand, and these goals are modified if contact is detected between a finger link and the graspable object. When a link contacts the object, the corresponding joint and all more proximal joints are given final position goals slightly beyond their current positions.

# Usage
To begin the simulation:
`bazel-bin/examples/robotiq_3f/joint_control/robotiq_3f_single_object_simulation --obj <object> --grasp_type <grasp type>`

Where `<object>` is `cylinder` or `box` and `<grasp type>` is `wide_power`, `wide_precision`, `basic_power`, `basic_precision`, or `pincher`. Scissor grasps are not currently supported.

To execute a grasp:
`bazel-bin/examples/robotiq_3f/joint_control/grasp_commander`

The specified object spawns on a plane, and the gripper's position is hard-coded based on the specified grasp type. The gripper's pose can alternately be set by specifying, e.g., `--gripper_x` or `--gripper_qw`, though changing any position or orientation will cause all elements from the hard-coded poses to be ignored.

The gripper is mounted to a `slider`, a robot with no visual or collision elements that contains one prismatic joint. After executing a grasp, the slider slides the hand upward parallel to the positive z-axis.

`grasp_commander` sends a binary closing command to the gripper controller and subscribes to the object's pose.

# TODO

- Replace joint state hybrid model with springs to emulate underactuation
- Use one URDF where the joints that spread the fingers apart are actuated
- Modify gains to replicate real gripper behavior
- Clean up and docuent code according to style guides
- Write tests

# Contribution Summary

## manipulation/models/robotiq_3f_description

The Robotiq 3-Finger Adaptive Gripper has three main operating modes: wide, basic, and pincher. The gripper switches between modes by spreading the two fingers on the same side of the gripper apart or bringing them closer together. The [official Robotiq ROS package](https://github.com/ros-industrial/robotiq) supports three discrete operating positions to avoid self-collision. Instead of actuating the joint that spreads these two fingers apart, this example currently uses three separate URDFs with fixed joint values. Additionally, as each finger's second joint remains fixed in pincher mode, these joints (between links 1 and 2) are fixed in the corresponding `robotiq-3f-gripper_articulated_pincher.urdf`.

The link masses used were reported by Robotiq: the palm is 1725.68g, each finger's links 0 and 1 combined are 126.22g, the fingers' second links are 60.1g, and the third links are 24g. Assuming links 1 and 2 have the same density, using the respective volumes of the visual meshes, link 1's mass is 104.89g. Each link's inertia was estimated using these masses and inertias computed from the visual meshes in MeshLab.

The collision models used are small boxes with surfaces at the original collision mesh's grasp surfaces. There are sizable gaps between the links to avoid self-collision.

## lcmtypes

`lcmt_obj_status` is used to publish messages about the graspable object's pose. The array `obj_state` of length `num_state_vals` (13 in this example), contains the object's orientation, position, and velocity.

`lcmt_robotiq_3f_binary_command` has a boolean `close_hand`, set true to request the controller to execute a grasp.

`lcmt_robotiq_3f_status`, like `lcmt_allegro_status`, contains arrays with joint position, velocity, commanded position, and commanded torques. It also contains an array `child_link_in_contact` to indicate whether each link is in contact with the graspable object used in the hybrid controller. The `contact_with_environment` flag is set when the gripper is in contact with the table.

`lcmt_slider_command` is used to set the desired position of the slider's prismatic joint. `lcmt_slider_status` reports `slider_state`, which contains the joint's position and velocity.

## examples/robotiq_3f

### robotiq_3f_common

Based on `examples/allegro_hand/allegro_common`. PID position controller gains, the joint mapping, and selector matrices are set here.

### robotiq_3f_lcm

The main grasping hybrid controller logic is located in `robotiq_3f_lcm`'s `Robotiq3fCommandReceiver::DoCalcDiscreteVariableUpdates`. From `robotiq_3f_lcm.cc`: Each of the gripper's 3-jointed fingers is actuated by one motor and wraps around objects it comes into contact with. These fingers can be modeled as hybrid systems by sending a short series of joint commands, and modifying the goals depending on when a link comes into contact with an obstacle. Currently, our LCM node receives a binary request to close the gripper. We use joint angles and contact information to send and modify joint angle goals. If contact is made and a joint in the physical underactuated hand would stop, we stop the joint by setting a final goal some small value above the current joint angle.

### Object and Slider LCM

LCM classes to send actuation commands to the slider, listen to its status, and listen to the object's pose are located in `slider_lcm` and `obj_status_lcm`.

### joint_control

`cylinder.sdf` and `box.sdf` are the two graspable object models. `floor.urdf` is the static surface the graspable objects rest on, and `slider.urdf` is the robot with a prismatic joint.

`robotiq_3f_single_object_simulation.cc` loads the slider and mounted gripper at the specified initial position. The floor and object are also loaded. LCM systems are connected and the simulation starts.

`grasp_commander.cc` sends a binary command to close the gripper, waits for the grasp to be executed, and listens to the object position.

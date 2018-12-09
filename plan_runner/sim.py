# import numpy as np
import time

# from pydrake.common import FindResourceOrThrow

# from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
# from plan_runner.manipulation_station_plan_runner import *
# from plan_runner.open_left_door import (GenerateOpenLeftDoorPlansByTrajectory,
#                                         GenerateOpenLeftDoorPlansByImpedanceOrPosition,)

# from pydrake.examples.manipulation_station import (ManipulationStation,
#                                     ManipulationStationHardwareInterface)
# from pydrake.geometry import SceneGraph
# from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
# from pydrake.systems.framework import DiagramBuilder
# from pydrake.systems.analysis import Simulator
# from pydrake.common.eigen_geometry import Isometry3
# from pydrake.systems.primitives import Demultiplexer, LogOutput
# from underactuated.meshcat_visualizer import MeshcatVisualizer

# from plan_runner.manipulation_station_plan_runner import ManipStationPlanRunner
# from plan_runner.manipulation_station_plan_runner_diagram import CreateManipStationPlanRunnerDiagram
# from plan_runner.plan_utils import *

# # Construct simulator system.
# object_file_path = FindResourceOrThrow(
#     "drake/examples/manipulation_station/models/061_foam_brick.sdf")

# manip_station_sim = ManipulationStationSimulator(
#     time_step=2e-3,
#     object_file_path=object_file_path,
#     object_base_link_name="base_link",)

# # Generate plans.
# plan_list = None
# gripper_setpoint_list = None
# plan_list, gripper_setpoint_list = GenerateOpenLeftDoorPlansByTrajectory()

# plan_list2 = list(plan_list)
# gripper_setpoint_list2 = list(gripper_setpoint_list)
# q0_kuka = [0, 0, 0, -1.75, 0, 1.0, 0]
# # iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
# #     state_log = manip_station_sim.RunSimulation(
# #         plan_list, gripper_setpoint_list, extra_time=2.0, real_time_rate=0.0, q0_kuka=q0,
# #         is_visualizing=True,
# #         is_plan_runner_diagram=True)
# # PlotExternalTorqueLog(iiwa_external_torque_log)
# # PlotIiwaPositionLog(iiwa_position_command_log, iiwa_position_measured_log)

# extra_time = 2.0
# real_time_rate= 0
# is_visualizing = True
# sim_duration = None

# builder = DiagramBuilder()
# builder.AddSystem(manip_station_sim.station)

# # Add plan runner.
# plan_runner, plan_scheduler = CreateManipStationPlanRunnerDiagram(
#     station=manip_station_sim.station,
#     kuka_plans=plan_list,
#     gripper_setpoint_list=gripper_setpoint_list)
# manip_station_sim.plan_runner = plan_runner

# builder.AddSystem(plan_runner)
# builder.Connect(plan_runner.GetOutputPort("gripper_setpoint"),
#                 manip_station_sim.station.GetInputPort("wsg_position"))
# builder.Connect(plan_runner.GetOutputPort("force_limit"),
#                 manip_station_sim.station.GetInputPort("wsg_force_limit"))


# demux = builder.AddSystem(Demultiplexer(14, 7))
# builder.Connect(
#     plan_runner.GetOutputPort("iiwa_position_and_torque_command"),
#     demux.get_input_port(0))
# builder.Connect(demux.get_output_port(0),
#                 manip_station_sim.station.GetInputPort("iiwa_position"))
# builder.Connect(demux.get_output_port(1),
#                 manip_station_sim.station.GetInputPort("iiwa_feedforward_torque"))
# builder.Connect(manip_station_sim.station.GetOutputPort("iiwa_position_measured"),
#                 plan_runner.GetInputPort("iiwa_position"))
# builder.Connect(manip_station_sim.station.GetOutputPort("iiwa_velocity_estimated"),
#                 plan_runner.GetInputPort("iiwa_velocity"))

# # Add meshcat visualizer
# if is_visualizing:
#     scene_graph = manip_station_sim.station.get_mutable_scene_graph()
#     viz = MeshcatVisualizer(scene_graph,
#                             is_drawing_contact_force = True,
#                             plant = manip_station_sim.plant)
#     builder.AddSystem(viz)
#     builder.Connect(manip_station_sim.station.GetOutputPort("pose_bundle"),
#                     viz.GetInputPort("lcm_visualization"))
#     builder.Connect(manip_station_sim.station.GetOutputPort("contact_results"),
#                     viz.GetInputPort("contact_results"))

# # Add logger
# iiwa_position_command_log = LogOutput(demux.get_output_port(0), builder)
# iiwa_position_command_log._DeclarePeriodicPublish(0.005)

# iiwa_external_torque_log = LogOutput(
#     manip_station_sim.station.GetOutputPort("iiwa_torque_external"), builder)
# iiwa_external_torque_log._DeclarePeriodicPublish(0.005)

# iiwa_position_measured_log = LogOutput(
#     manip_station_sim.station.GetOutputPort("iiwa_position_measured"), builder)
# iiwa_position_measured_log._DeclarePeriodicPublish(0.005)

# plant_state_log = LogOutput(
#     manip_station_sim.station.GetOutputPort("plant_continuous_state"), builder)
# plant_state_log._DeclarePeriodicPublish(0.005)

# # build diagram
# diagram = builder.Build()
# print(diagram)
# if is_visualizing:
#     viz.load()
#     time.sleep(2.0)
#     # RenderSystemWithGraphviz(diagram)

# # construct simulator
# simulator = Simulator(diagram)
# manip_station_sim.simulator = simulator

# context = diagram.GetMutableSubsystemContext(
#     manip_station_sim.station, simulator.get_mutable_context())

# # set initial state of the robot
# manip_station_sim.station.SetIiwaPosition(q0_kuka, context)
# manip_station_sim.station.SetIiwaVelocity(np.zeros(7), context)
# manip_station_sim.station.SetWsgPosition(0.05, context)
# manip_station_sim.station.SetWsgVelocity(0, context)

# # set initial hinge angles of the cupboard.
# # setting hinge angle to exactly 0 or 90 degrees will result in intermittent contact
# # with small contact forces between the door and the cupboard body.
# left_hinge_joint = manip_station_sim.plant.GetJointByName("left_door_hinge")
# left_hinge_joint.set_angle(
#     context=manip_station_sim.station.GetMutableSubsystemContext(manip_station_sim.plant, context), angle=-0.001)

# right_hinge_joint = manip_station_sim.plant.GetJointByName("right_door_hinge")
# right_hinge_joint.set_angle(
#     context=manip_station_sim.station.GetMutableSubsystemContext(manip_station_sim.plant, context), angle=0.001)

# # set initial pose of the object
# if manip_station_sim.object_base_link_name is not None:
#     manip_station_sim.plant.tree().SetFreeBodyPoseOrThrow(
#        manip_station_sim.plant.GetBodyByName(manip_station_sim.object_base_link_name, manip_station_sim.object),
#         manip_station_sim.X_WObject, manip_station_sim.station.GetMutableSubsystemContext(manip_station_sim.plant, context))

# simulator.set_publish_every_time_step(False)
# simulator.set_target_realtime_rate(real_time_rate)

# # calculate starting time for all plans.
# t_plan = GetPlanStartingTimes(plan_list)
# if sim_duration is None:
#     sim_duration = t_plan[-1] + extra_time
# print "simulation duration", sim_duration
# simulator.Initialize()
# simulator.StepTo(sim_duration)
# print("Done with first sim")


# manip_station_sim.station.SetIiwaPosition(q0_kuka, context)
# manip_station_sim.station.SetIiwaVelocity(np.zeros(7), context)
# manip_station_sim.station.SetWsgPosition(0.05, context)
# manip_station_sim.station.SetWsgVelocity(0, context)

# # set initial hinge angles of the cupboard.
# # setting hinge angle to exactly 0 or 90 degrees will result in intermittent contact
# # with small contact forces between the door and the cupboard body.
# left_hinge_joint = manip_station_sim.plant.GetJointByName("left_door_hinge")
# left_hinge_joint.set_angle(
#     context=manip_station_sim.station.GetMutableSubsystemContext(manip_station_sim.plant, context), angle=-0.001)

# right_hinge_joint = manip_station_sim.plant.GetJointByName("right_door_hinge")
# right_hinge_joint.set_angle(
#     context=manip_station_sim.station.GetMutableSubsystemContext(manip_station_sim.plant, context), angle=0.001)

# # set initial pose of the object
# if manip_station_sim.object_base_link_name is not None:
#     manip_station_sim.plant.tree().SetFreeBodyPoseOrThrow(
#        manip_station_sim.plant.GetBodyByName(manip_station_sim.object_base_link_name, manip_station_sim.object),
#         manip_station_sim.X_WObject, manip_station_sim.station.GetMutableSubsystemContext(manip_station_sim.plant, context))

# simulator.Initialize()
# simulator.get_mutable_context().set_time(0)
# print("Sim time set to 0")

# plan_scheduler.AddPlans([plan_list2[0]], [gripper_setpoint_list2[0]])
# print("Added plans")
# simulator.StepTo(5)
# simulator.get_mutable_context().set_time(0)
# plan_scheduler.AddPlans([plan_list2[1]], [gripper_setpoint_list2[1]])
# simulator.StepTo(5)
# simulator.get_mutable_context().set_time(0)
# plan_scheduler.AddPlans([plan_list2[2]], [gripper_setpoint_list2[2]])
# simulator.StepTo(5)

from environment import ManipStationEnvironment

env = ManipStationEnvironment(is_visualizing=False)

# q0_kuka = [0, 0, 0, -1.75, 0, 1.0, 0]

# from robot_plans import JointSpacePlanGoToTarget
# 
# new_plan = JointSpacePlanGoToTarget(q_target=q0_kuka, duration=2.0)
# 
# env.plan_scheduler.setPlan(new_plan, 0.05)
# 
# env.simulator.StepTo(2.0)
# 
# env.plan_scheduler.setPlan(new_plan, 0.0)
# 
# env.simulator.StepTo(4.0)


action_1 = [0, 0, 0, 0, 0, -0.2, 0, 0.05]
action_1_rev = [0,0,0,0,0,0.2,0,0.05]
action_2 = [0, 0, 0, -0.1, 0, 0, 0, 0.05]

print("starting")

time_start = time.time()

for i in range(10):
    env.step(action_1)
    env.step(action_1_rev)

time_end = time.time()

print(env.plan_scheduler.end_time)
print(time_end - time_start)
import numpy as np
import time

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import SceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import Demultiplexer
from underactuated.meshcat_visualizer import MeshcatVisualizer

from plan_runner.manipulation_station_plan_runner_diagram import CreateManipStationPlanRunnerDiagram
from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
from plan_runner.plan_utils import *
from robot_plans import JointSpacePlanRelative

object_file_path = FindResourceOrThrow(
    "drake/examples/manipulation_station/models/061_foam_brick.sdf")
q0_kuka = [0, 0, 0, -1.75, 0, 1.0, 0]

class ManipStationEnvironment(object):
    def __init__(self, real_time_rate=0, is_visualizing=False):

        # Create manipulation station simulator
        self.manip_station_sim = ManipulationStationSimulator(
            time_step=2e-3,
            object_file_path=object_file_path,
            object_base_link_name="base_link",)

        # Create plan runner
        plan_runner, self.plan_scheduler = CreateManipStationPlanRunnerDiagram(
            station=self.manip_station_sim.station,
            kuka_plans=[],
            gripper_setpoint_list=[],
            rl_environment=True)
        self.manip_station_sim.plan_runner = plan_runner

        # Create builder and add systems
        builder = DiagramBuilder()
        builder.AddSystem(self.manip_station_sim.station)
        builder.AddSystem(plan_runner)

        # Connect systems
        builder.Connect(plan_runner.GetOutputPort("gripper_setpoint"),
                self.manip_station_sim.station.GetInputPort("wsg_position"))
        builder.Connect(plan_runner.GetOutputPort("force_limit"),
                self.manip_station_sim.station.GetInputPort("wsg_force_limit"))

        demux = builder.AddSystem(Demultiplexer(14, 7))
        builder.Connect(
            plan_runner.GetOutputPort("iiwa_position_and_torque_command"),
            demux.get_input_port(0))
        builder.Connect(demux.get_output_port(0),
                        self.manip_station_sim.station.GetInputPort("iiwa_position"))
        builder.Connect(demux.get_output_port(1),
                        self.manip_station_sim.station.GetInputPort("iiwa_feedforward_torque"))
        builder.Connect(self.manip_station_sim.station.GetOutputPort("iiwa_position_measured"),
                        plan_runner.GetInputPort("iiwa_position"))
        builder.Connect(self.manip_station_sim.station.GetOutputPort("iiwa_velocity_estimated"),
                        plan_runner.GetInputPort("iiwa_velocity"))

        # Add meshcat visualizer
        if is_visualizing:
            scene_graph = self.manip_station_sim.station.get_mutable_scene_graph()
            viz = MeshcatVisualizer(scene_graph,
                                    is_drawing_contact_force = True,
                                    plant = self.manip_station_sim.plant)
            builder.AddSystem(viz)
            builder.Connect(self.manip_station_sim.station.GetOutputPort("pose_bundle"),
                            viz.GetInputPort("lcm_visualization"))
            builder.Connect(self.manip_station_sim.station.GetOutputPort("contact_results"),
                            viz.GetInputPort("contact_results"))

        # Build diagram
        self.diagram = builder.Build()
        if is_visualizing:
            print("Setting up visualizer...")
            viz.load()
            time.sleep(2.0)

        # Construct Simulator
        self.simulator = Simulator(self.diagram)
        self.manip_station_sim.simulator = self.simulator

        self.simulator.set_publish_every_time_step(False)
        self.simulator.set_target_realtime_rate(real_time_rate)

        self.context = self.diagram.GetMutableSubsystemContext(
            self.manip_station_sim.station, self.simulator.get_mutable_context())

        # Set initial state of the robot
        self.reset()
        print("Environment loaded, ready to begin in 2 sec")
        time.sleep(2.0)

    def step(self, action):
        assert len(action) == 8
        next_plan = JointSpacePlanRelative(delta_q=action[:-1], duration=0.1)

        sim_duration = self.plan_scheduler.setNextPlan(next_plan, action[-1])
        self.simulator.StepTo(sim_duration)

        return self._getObservation()

    def reset(self):
        # set initial state of the robot
        self.manip_station_sim.station.SetIiwaPosition(q0_kuka, self.context)
        self.manip_station_sim.station.SetIiwaVelocity(np.zeros(7), self.context)
        self.manip_station_sim.station.SetWsgPosition(0.05, self.context)
        self.manip_station_sim.station.SetWsgVelocity(0, self.context)

        # set initial hinge angles of the cupboard.
        # setting hinge angle to exactly 0 or 90 degrees will result in intermittent contact
        # with small contact forces between the door and the cupboard body.
        left_hinge_joint = self.manip_station_sim.plant.GetJointByName("left_door_hinge")
        left_hinge_joint.set_angle(
            context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context), angle=-0.001)

        right_hinge_joint = self.manip_station_sim.plant.GetJointByName("right_door_hinge")
        right_hinge_joint.set_angle(
            context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context), angle=0.001)

        # set initial pose of the object
        if self.manip_station_sim.object_base_link_name is not None:
            self.manip_station_sim.plant.tree().SetFreeBodyPoseOrThrow(
               self.manip_station_sim.plant.GetBodyByName(self.manip_station_sim.object_base_link_name, self.manip_station_sim.object),
                self.manip_station_sim.X_WObject, self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context))

        self.simulator.Initialize()

        # Need to return observation

    def _getObservation(self):
        return None

    def _getReward(self):
        return None
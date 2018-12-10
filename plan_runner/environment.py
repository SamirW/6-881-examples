import numpy as np
import time

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import SceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.multibody_tree.math import SpatialVelocity
from pydrake.systems.analysis import Simulator, RungeKutta3Integrator
from pydrake.systems.primitives import Demultiplexer
from underactuated.meshcat_visualizer import MeshcatVisualizer

from plan_runner.manipulation_station_plan_runner_diagram import CreateManipStationPlanRunnerDiagram
from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
from plan_runner.plan_utils import *
from robot_plans import JointSpacePlanRelative

from env_utils import *

object_file_path = FindResourceOrThrow(
    "drake/examples/manipulation_station/models/061_foam_brick.sdf")

class ManipStationEnvironment(object):
    def __init__(self, real_time_rate=0, is_visualizing=False):
        # Store for continuity
        self.is_visualizing = is_visualizing
        self.real_time_rate = real_time_rate

        self.build(real_time_rate=0, is_visualizing=is_visualizing)

    def build(self, real_time_rate=0, is_visualizing=False):
        # Create manipulation station simulator
        self.manip_station_sim = ManipulationStationSimulator(
            time_step=5e-3,
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
                                    is_drawing_contact_force = False,
                                    plant = self.manip_station_sim.plant)
            builder.AddSystem(viz)
            builder.Connect(self.manip_station_sim.station.GetOutputPort("pose_bundle"),
                            viz.GetInputPort("lcm_visualization"))
            # builder.Connect(self.manip_station_sim.station.GetOutputPort("contact_results"),
                            # viz.GetInputPort("contact_results"))

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

        self.left_hinge_joint = self.manip_station_sim.plant.GetJointByName("left_door_hinge")
        self.right_hinge_joint = self.manip_station_sim.plant.GetJointByName("right_door_hinge")
        
        # Goal for training
        self.goal_position = np.array([0.85, 0, 0.31])

        # Object body
        self.obj = self.manip_station_sim.plant.GetBodyByName(self.manip_station_sim.object_base_link_name, self.manip_station_sim.object)

        # Properties for RL
        max_action = np.ones(8) * 0.1
        max_action[-1] = 0.03
        low_action = -1*max_action
        low_action[-1] = 0
        self.action_space = ActionSpace(low=low_action, high=max_action)
        self.state_dim = self._getObservation().shape[0]
        self._episode_steps = 0
        self._max_episode_steps = 75
        
        # Set initial state of the robot
        self.reset_sim = False
        self.reset()

    def step(self, action):
        assert len(action) == 8
        next_plan = JointSpacePlanRelative(delta_q=action[:-1], duration=0.1)

        # sim_duration = self.plan_scheduler.setNextPlan(next_plan, action[-1])
        sim_duration = self.plan_scheduler.setNextPlan(next_plan, action[-1])

        try:
            self.simulator.StepTo(sim_duration)
        except:
            self.reset_sim = True
            return self._getObservation(), -50, True, None

        reward = self._getReward()

        if reward > -0.1:
            done = True
        else:
            self._episode_steps += 1
            if self._episode_steps == self._max_episode_steps:
                done = True
            else:
                done = False

        return self._getObservation(), reward, done, None

    def reset(self):

        if self.reset_sim:
            print("Resetting")
            self.build(real_time_rate=self.real_time_rate, is_visualizing=self.is_visualizing)

        while True:
            p_WQ_new = np.random.uniform(low=[0.05, -0.1, 0.5], high=[0.5, 0.1, 0.5])
            # p_WQ_new = np.array([0.2, 0, 0.5])
            passed, q_home_full = GetConfiguration(p_WQ_new)
            if passed:
                break
        q_home_kuka = GetKukaQKnots(q_home_full)[0]


        # set initial hinge angles of the cupboard.
        # setting hinge angle to exactly 0 or 90 degrees will result in intermittent contact
        # with small contact forces between the door and the cupboard body.
        self.left_hinge_joint.set_angle(
            context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context), angle=-np.pi/2+0.001)

        self.right_hinge_joint.set_angle(
            context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context), angle=np.pi/2-0.001)

        # set initial pose of the object
        # self.manip_station_sim.SetObjectTranslation(p_WQ_new+np.array([0.01,0,-0.022]))
        if self.manip_station_sim.object_base_link_name is not None:
            self.manip_station_sim.tree.SetFreeBodyPoseOrThrow(
               self.manip_station_sim.plant.GetBodyByName(self.manip_station_sim.object_base_link_name, self.manip_station_sim.object),
                self.manip_station_sim.X_WObject, self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context))

        if self.manip_station_sim.object_base_link_name is not None:
            self.manip_station_sim.tree.SetFreeBodySpatialVelocityOrThrow(
               self.manip_station_sim.plant.GetBodyByName(self.manip_station_sim.object_base_link_name, self.manip_station_sim.object),
                SpatialVelocity(np.zeros(3), np.zeros(3)), self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context))

        # set initial state of the robot
        self.manip_station_sim.station.SetIiwaPosition(q_home_kuka, self.context)
        self.manip_station_sim.station.SetIiwaVelocity(np.zeros(7), self.context)
        self.manip_station_sim.station.SetWsgPosition(0.02, self.context)
        self.manip_station_sim.station.SetWsgVelocity(0, self.context)

        self.simulator.Initialize()

        self._episode_steps = 0

        return self._getObservation()

    def seed(self, seed):
        np.random.seed(seed)

    def _getObservation(self):
        kuka_position = self.manip_station_sim.station.GetIiwaPosition(self.context)
        object_position = self.manip_station_sim.tree.EvalBodyPoseInWorld(
            context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context),
            body=self.obj).translation()
        return np.append(kuka_position, object_position)

    def _getReward(self):
        # object_position = self.manip_station_sim.tree.EvalBodyPoseInWorld(
        #     context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context),
        #     body=self.obj).translation()
        gripper_pose = self.manip_station_sim.tree.CalcRelativeTransform(
            context=self.manip_station_sim.station.GetMutableSubsystemContext(self.manip_station_sim.plant, self.context),
            frame_A=self.manip_station_sim.world_frame,
            frame_B=self.manip_station_sim.gripper_frame).translation()
        dist = np.linalg.norm(self.goal_position-gripper_pose)
        return -dist
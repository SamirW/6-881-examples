from pydrake.multibody import inverse_kinematics
from pydrake.common.eigen_geometry import Isometry3
from pydrake.math import RollPitchYaw, RotationMatrix

from plan_runner.manipulation_station_plan_runner import *

import numpy as np

# Define global variables used for IK.
plant = station.get_mutable_multibody_plant()
tree = plant.tree()
context_plant = plant.CreateDefaultContext()

iiwa_model = plant.GetModelInstanceByName("iiwa")
gripper_model = plant.GetModelInstanceByName("gripper")

world_frame = plant.world_frame()
gripper_frame = plant.GetFrameByName("body", gripper_model)

R_EEa = RotationMatrix(X_EEa.rotation())

def GetKukaQKnots(q_knots):
    """
    q returned by IK consists of the configuration of all bodies in a MultibodyTree.
    In this case, it includes the iiwa arm, the cupboard doors, the gripper and the manipulands, but
    the trajectory sent to iiwa only needs the configuration of iiwa itself.

    This function takes in an array of shape (n, plant.num_positions()),
    and returns an array of shape (n, 7), which only has the configuration of the iiwa arm.
    """
    if len(q_knots.shape) == 1:
        q_knots.resize(1, q_knots.size)
    n = q_knots.shape[0]
    q_knots_kuka = np.zeros((n, 7))
    for i, q_knot in enumerate(q_knots):
        q_knots_kuka[i] = tree.GetPositionsFromArray(iiwa_model, q_knot)

    return q_knots_kuka


def InverseKinPointwise(p_WQ_start, p_WQ_end, duration,
                        num_knot_points,
                        q_initial_guess,
                        InterpolatePosition=None,
                        InterpolateOrientation=None,
                        position_tolerance=0.005,
                        theta_bound=0.005 * np.pi, # 0.9 degrees
                        is_printing=False):
    """
    Calculates a joint space trajectory for iiwa by repeatedly calling IK. The first IK is initialized (seeded)
    with q_initial_guess. Subsequent IKs are seeded with the solution from the previous IK.

    Positions for point Q (p_EQ) and orientations for the end effector, generated respectively by
    InterpolatePosition and InterpolateOrientation, are added to the IKs as constraints.

    @param p_WQ_start: The first argument of function InterpolatePosition (defined below).
    @param p_WQ_end: The second argument of function InterpolatePosition (defined below).
    @param duration: The duration of the trajectory returned by this function in seconds.
    @param num_knot_points: number of knot points in the trajectory.
    @param q_initial_guess: initial guess for the first IK.
    @param InterpolatePosition: A function with signature (start, end, num_knot_points, i). It returns
       p_WQ, a (3,) numpy array which describes the desired position of Q at knot point i in world frame.
    @param InterpolateOrientation: A function with signature
    @param position_tolerance: tolerance for IK position constraints in meters.
    @param theta_bound: tolerance for IK orientation constraints in radians.
    @param is_printing: whether the solution results of IKs are printed.
    @return: qtraj: a 7-dimensional cubic polynomial that describes a trajectory for the iiwa arm.
    @return: q_knots: a (n, num_knot_points) numpy array (where n = plant.num_positions()) that stores solutions
        returned by all IKs. It can be used to initialize IKs for the next trajectory.
    """
    q_knots = np.zeros((num_knot_points + 1, plant.num_positions()))
    q_knots[0] = q_initial_guess

    for i in range(num_knot_points):
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()

        # Orientation constraint
        R_WEa_ref = InterpolateOrientation(i, num_knot_points)
        ik.AddOrientationConstraint(
            frameAbar=world_frame, R_AbarA=R_WEa_ref,
            frameBbar=gripper_frame, R_BbarB=R_EEa,
            theta_bound=theta_bound)

        # Position constraint
        p_WQ = InterpolatePosition(p_WQ_start, p_WQ_end, num_knot_points, i)
        ik.AddPositionConstraint(
            frameB=gripper_frame, p_BQ=p_EQ,
            frameA=world_frame,
            p_AQ_lower=p_WQ - position_tolerance,
            p_AQ_upper=p_WQ + position_tolerance)

        prog = ik.prog()
        # use the robot posture at the previous knot point as
        # an initial guess.
        prog.SetInitialGuess(q_variables, q_knots[i])
        result = prog.Solve()
        if is_printing:
            print i, ": ", result
        q_knots[i + 1] = prog.GetSolution(q_variables)

    t_knots = np.linspace(0, duration, num_knot_points + 1)
    q_knots_kuka = GetKukaQKnots(q_knots)
    qtraj = PiecewisePolynomial.Cubic(
        t_knots, q_knots_kuka.T, np.zeros(7), np.zeros(7))

    return qtraj, q_knots

def GetHomeConfiguration(is_printing=False):
    """
    Returns a configuration of the MultibodyPlant in which point Q (defined by global variable p_EQ)
    in robot EE frame is at p_WQ_home, and orientation of frame Ea is R_WEa_ref.
    """
    # get "home" pose
    ik_scene = inverse_kinematics.InverseKinematics(plant)

    theta_bound = 0.005 * np.pi # 0.9 degrees
    X_EEa = GetEndEffectorWorldAlignedFrame()
    R_EEa = RotationMatrix(X_EEa.rotation())

    ik_scene.AddOrientationConstraint(
        frameAbar=world_frame, R_AbarA=R_WEa_ref,
        frameBbar=gripper_frame, R_BbarB=R_EEa,
        theta_bound=theta_bound)

    p_WQ0 = p_WQ_home
    p_WQ_lower = p_WQ0 - 0.005
    p_WQ_upper = p_WQ0 + 0.005
    ik_scene.AddPositionConstraint(
        frameB=gripper_frame, p_BQ=p_EQ,
        frameA=world_frame,
        p_AQ_lower=p_WQ_lower, p_AQ_upper=p_WQ_upper)

    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), np.zeros(plant.num_positions()))
    result = prog.Solve()
    if is_printing:
        print result
    return prog.GetSolution(ik_scene.q())

def GetConfiguration(p_WQ_home, is_printing=False):
    """
    Returns a configuration of the MultibodyPlant in which point Q (defined by global variable p_EQ)
    in robot EE frame is at p_WQ_home, and orientation of frame Ea is R_WEa_ref.
    """
    # get "home" pose
    ik_scene = inverse_kinematics.InverseKinematics(plant)

    theta_bound = 0.005 * np.pi # 0.9 degrees
    X_EEa = GetEndEffectorWorldAlignedFrame()
    R_EEa = RotationMatrix(X_EEa.rotation())

    ik_scene.AddOrientationConstraint(
        frameAbar=world_frame, R_AbarA=R_WEa_ref,
        frameBbar=gripper_frame, R_BbarB=R_EEa,
        theta_bound=theta_bound)

    p_WQ0 = p_WQ_home
    p_WQ_lower = p_WQ0 - 0.01
    p_WQ_upper = p_WQ0 + 0.01
    ik_scene.AddPositionConstraint(
        frameB=gripper_frame, p_BQ=p_EQ,
        frameA=world_frame,
        p_AQ_lower=p_WQ_lower, p_AQ_upper=p_WQ_upper)

    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), np.zeros(plant.num_positions()))
    result = prog.Solve()
    if is_printing:
        print result
    return (result==False), prog.GetSolution(ik_scene.q())

def InterpolateStraightLine(p_WQ_start, p_WQ_end, num_knot_points, i):
        return (p_WQ_end - p_WQ_start)/num_knot_points*(i+1) + p_WQ_start

def InterpolatePitchAngle(i, num_knot_points):
        assert i <= num_knot_points
        pitch_start = np.pi / 180 * 135
        pitch_end = np.pi / 180 * 90
        pitch_angle = pitch_start + (pitch_end - pitch_start) / num_knot_points * i
        return RollPitchYaw(0, pitch_angle, 0).ToRotationMatrix()

class ActionSpace(object):
    def __init__(self, low=None, high=None, dtype=np.float32):
        assert low.shape == high.shape
        self.low = low.astype(dtype)
        self.high = high.astype(dtype)
        self.shape = self.low.shape

    def sample(self):
        return np.random.uniform(low=self.low, high=self.high)

import example_robot_data
import example_parallel_robots
from toolbox_parallel_robots.projections import configurationProjection
import pinocchio as pin
import numpy as np

robot = example_robot_data.load("b1_leg")
model_simp = robot.model
model, constraint_models, actuation_model, visual_model, collision_model = example_parallel_robots.load("b1_leg3d")
q0_simp = robot.q0


data = model.createData()
data_simp = model_simp.createData()

q_ref = pin.neutral(model)
q_ref[0] = 0.0
q_ref[1] = 0.8
q_ref[2] = -1.6
q_ref[3] = 1.8
q_ref[4] = 1.6
# q_ref[5] = 0
joints_lock_names = ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"]
LOOP_JOINT_IDS_Q = []
LOOP_JOINT_IDS_V = []
for i, name in enumerate(joints_lock_names):
    jId = model.getJointId(name)
    for niq in range(model.joints[jId].nq):
        LOOP_JOINT_IDS_Q.append(model.joints[jId].idx_q + niq)
    for niv in range(model.joints[jId].nv):
        LOOP_JOINT_IDS_V.append(model.joints[jId].idx_v + niv)
SERIAL_JOINT_IDS_Q = [i for i in range(model.nq) if i not in LOOP_JOINT_IDS_Q]
SERIAL_JOINT_IDS_V = [i for i in range(model.nv) if i not in LOOP_JOINT_IDS_V]
robot_constraint_datas = [cm.createData() for cm in constraint_models]
w = np.ones(model.nv)
w[SERIAL_JOINT_IDS_V] = 1e5
W = np.diag(w)
q0 = configurationProjection(
    model,
    model.createData(),
    constraint_models,
    robot_constraint_datas,
    q_ref,
    W,
)

pin.computeAllTerms(model, data, q0, np.zeros(model.nv))
pin.computeAllTerms(model_simp, data_simp, q0_simp, np.zeros(model_simp.nv))

print("Model:", model)
print("Simplified Model:", model_simp)
print("Condition number (full model):", np.linalg.cond(data.M))
print("Condition number (simplified model):", np.linalg.cond(data_simp.M))
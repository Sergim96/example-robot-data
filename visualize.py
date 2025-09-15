import time
import example_robot_data
from pinocchio.visualize import MeshcatVisualizer

robot = example_robot_data.load("b1_leg_RR")
# robot = example_robot_data.load("kangaroo_legs")
model = robot.model
print(model)
collision_model = robot.collision_model
visual_model = robot.visual_model
q0 = robot.q0

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()

while True:
    viz.display(q0)
    time.sleep(0.1)
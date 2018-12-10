from environment import ManipStationEnvironment
import numpy as np
import time

env = ManipStationEnvironment(is_visualizing=True)
action_1 = [0, 0, 0, 0, 0, -0.2, 0, 0.02]
# action_1 = np.zeros(8)
# action_1_rev = [0,0,0,0,0,0.2,0,0.05]
# action_2 = [0, 0, 0, -0.1, 0, 0, 0, 0.05]
# close_grip = [0, 0, 0, 0, 0, 0, 0, 0]

# print("starting")

# time_start = time.time()

for i in range(1):
	env.reset()
	for j in range(5):
		env.step(action_1)

#     for i in range(10):
#         env.step(action_1_rev)

# time_end = time.time()

# print(env.plan_scheduler.end_time)
# print(time_end - time_start)
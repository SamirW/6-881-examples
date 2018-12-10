from plan_runner.environment import ManipStationEnvironment
import numpy as np
import torch
import time
import argparse
import os

import utils
import TD3

# Runs policy for X episodes and returns average reward
def evaluate_policy(policy, eval_episodes=10):
	avg_reward = 0.
	for _ in xrange(eval_episodes):
		obs = env.reset()
		done = False
		while not done:
			action = policy.select_action(np.array(obs))
			obs, reward, done, _ = env.step(action)
			avg_reward += reward

	avg_reward /= eval_episodes

	print("---------------------------------------")
	print("Evaluation over %d episodes: %f" % (eval_episodes, avg_reward))
	print("---------------------------------------")
	return avg_reward

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument("--seed", default=0, type=int)
	parser.add_argument("--num_evals", default=10, type=int)
	parser.add_argument("--visualize", action="store_true")
	args = parser.parse_args()

	policy_name = "TD3"
	env_name = "ManipStation"

	file_name = "%s_%s_%s" % (policy_name, env_name, str(args.seed))
	print("---------------------------------------")
	print("Settings: %s" % (file_name))
	print("---------------------------------------")

	if not os.path.exists("./results"):
		exit()

	env = ManipStationEnvironment(is_visualizing=args.visualize)
	print("Starting in 1 second...")
	time.sleep(1)
	print("Starting!")

	# Set seeds
	env.seed(args.seed)
	torch.manual_seed(args.seed)
	np.random.seed(args.seed)
	
	state_dim = env.state_dim
	action_dim = env.action_space.shape[0] 
	max_action = float(env.action_space.high[0])

	policy = TD3.TD3(state_dim, action_dim, max_action)
	policy.load(file_name, directory="./pytorch_models")

	evaluate_policy(policy, eval_episodes=args.num_evals)
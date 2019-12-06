from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time

from absl import app
from absl import flags
from absl import logging

import gin
import tensorflow as tf

from tf_agents.agents.ddpg import critic_network
from tf_agents.agents.sac import sac_agent
from tf_agents.drivers import dynamic_step_driver
from tf_agents.environments import suite_gym
from tf_agents.environments import tf_py_environment
from tf_agents.eval import metric_utils
from tf_agents.metrics import tf_metrics
from tf_agents.networks import actor_distribution_network
from tf_agents.networks import normal_projection_network
from tf_agents.policies import greedy_policy
from tf_agents.policies import random_tf_policy
from tf_agents.replay_buffers import tf_uniform_replay_buffer
from tf_agents.utils import common
import copy
import gym
import scrimmage.utils
import random
import os
import time

train_checkpoint_interval=100,
policy_checkpoint_interval=100,
rb_checkpoint_interval=100,
#https://github.com/tensorflow/agents/blob/master/tf_agents/agents/sac/examples/v2/train_eval.py
num_iterations = 1000000 # @param {type:"integer"}

initial_collect_steps = 100 # @param {type:"integer"}
collect_steps_per_iteration = 1 # @param {type:"integer"}
replay_buffer_capacity = 1000000 # @param {type:"integer"}

batch_size = 256 # @param {type:"integer"}
root_dir = "/Users/tramtran/scrimmage/swarm/checkpoints"
critic_learning_rate = 3e-4 # @param {type:"number"}
actor_learning_rate = 3e-4 # @param {type:"number"}
alpha_learning_rate = 3e-4 # @param {type:"number"}
target_update_tau = 0.005 # @param {type:"number"}
target_update_period = 1 # @param {type:"number"}
gamma = 0.99 # @param {type:"number"}
reward_scale_factor = 1.0 # @param {type:"number"}
gradient_clipping = None # @param

actor_fc_layer_params = (256, 256)
critic_joint_fc_layer_params = (256, 256)

log_interval = 5000 # @param {type:"integer"}

num_eval_episodes = 30 # @param {type:"integer"}
eval_interval = 100 # @param {type:"integer"}

def compute_avg_return(environment, policy, num_episodes=5):

  total_return = 0.0
  for _ in range(num_episodes):

    time_step = copy.deepcopy(environment.reset())
    episode_return = 0.0

    while not time_step.is_last():
      action_step = policy.action(time_step)
      time_step = environment.step(action_step.action)
      episode_return += time_step.reward
    total_return += episode_return

  avg_return = total_return / num_episodes
  return avg_return.numpy()[0]
def normal_projection_net(action_spec,init_means_output_factor=0.1):
  return normal_projection_network.NormalProjectionNetwork(
      action_spec,
      mean_transform=None,
      state_dependent_std=True,
      init_means_output_factor=init_means_output_factor,
      std_transform=sac_agent.std_clip_transform,
      scale_distribution=True)

def get_action(obs):
    policy_checkpointer = common.Checkpointer(
        ckpt_dir=os.path.join(root_dir, 'policy'),
        policy=eval_policy,
        global_step=global_step)
    policy_checkpointer.initialize_or_restore()
    return policy_checkpointer

# Used for non-learning mode
def actor_init_func(action_space, obs_space, params):
    return get_action()

def test_openai():
    try:
        env = gym.make('scrimmage-v0')
    except gym.error.Error:
        mission_file = scrimmage.utils.find_mission('openai_mission.xml')

        gym.envs.register(
            id='scrimmage-v0',
            entry_point='scrimmage.bindings:ScrimmageOpenAIEnv',
            max_episode_steps=1e9,
            reward_threshold=1e9,
            kwargs={"enable_gui": False,
                    "mission_file": mission_file}
        )
        # env = gym.make('scrimmage-v0')
        train_env = tf_py_environment.TFPyEnvironment(suite_gym.load('scrimmage-v0'))
        eval_env = tf_py_environment.TFPyEnvironment(suite_gym.load('scrimmage-v0'))

        observation_spec = train_env.observation_spec()
        action_spec = train_env.action_spec()
        critic_net = critic_network.CriticNetwork(
                        (observation_spec, action_spec),
                        observation_fc_layer_params=None,
                        action_fc_layer_params=None,
                        joint_fc_layer_params=critic_joint_fc_layer_params)

        actor_net = actor_distribution_network.ActorDistributionNetwork(observation_spec,action_spec,
        fc_layer_params=actor_fc_layer_params,continuous_projection_net=normal_projection_net)
        global_step = tf.compat.v1.train.get_or_create_global_step()
        tf_agent = sac_agent.SacAgent(
        train_env.time_step_spec(),
        action_spec,
        actor_network=actor_net,
        critic_network=critic_net,
        actor_optimizer=tf.compat.v1.train.AdamOptimizer(
        learning_rate=actor_learning_rate),
        critic_optimizer=tf.compat.v1.train.AdamOptimizer(
        learning_rate=critic_learning_rate),
        alpha_optimizer=tf.compat.v1.train.AdamOptimizer(
        learning_rate=alpha_learning_rate),
        target_update_tau=target_update_tau,
        target_update_period=target_update_period,
        td_errors_loss_fn=tf.compat.v1.losses.mean_squared_error,
        gamma=gamma,
        reward_scale_factor=reward_scale_factor,
        gradient_clipping=gradient_clipping,
        train_step_counter=global_step)
        tf_agent.initialize()
        # train_metrics = [tf_metrics.NumberOfEpisodes(prefix='Train'),env_steps,average_return,tf_metrics.AverageEpisodeLengthMetric(prefix='Train',buffer_size=num_eval_episodes,batch_size=tf_env.batch_size),]
        eval_policy = greedy_policy.GreedyPolicy(tf_agent.policy)
        collect_policy = tf_agent.collect_policy

        replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(
        data_spec=tf_agent.collect_data_spec,
        batch_size=train_env.batch_size,
        max_length=replay_buffer_capacity)
        initial_collect_driver = dynamic_step_driver.DynamicStepDriver(
        train_env,
        collect_policy,
        observers=[replay_buffer.add_batch],
        num_steps=initial_collect_steps)
        # initial_collect_driver.run()
        dataset = replay_buffer.as_dataset(
        num_parallel_calls=3, sample_batch_size=batch_size, num_steps=2).prefetch(3)

        train_checkpointer = common.Checkpointer(ckpt_dir=os.path.join(root_dir, 'train'),agent=tf_agent,global_step=global_step)
        policy_checkpointer = common.Checkpointer(ckpt_dir=os.path.join(root_dir, 'policy'),policy=eval_policy,global_step=global_step)
        rb_checkpointer = common.Checkpointer(ckpt_dir=os.path.join(root_dir, 'replay_buffer'),max_to_keep=1,replay_buffer=replay_buffer)

        train_checkpointer.initialize_or_restore()
        rb_checkpointer.initialize_or_restore()
        iterator = iter(dataset)
        collect_driver = dynamic_step_driver.DynamicStepDriver(
        train_env,
        collect_policy,
        observers=[replay_buffer.add_batch],
        num_steps=collect_steps_per_iteration)
        tf_agent.train = common.function(tf_agent.train)
        collect_driver.run = common.function(collect_driver.run)
        tf_agent.train_step_counter.assign(0)
        # avg_return = compute_avg_return(eval_env, eval_policy, num_eval_episodes)
        # returns = [avg_return]

        print("___________________________time_step___________________________")
        print(train_env.time_step_spec())
        # for _ in range(num_iterations):
        #     for _ in range(collect_steps_per_iteration):
        #         collect_driver.run()
        #     experience, unused_info = next(iterator)
        #     train_loss = tf_agent.train(experience)
        #     step = tf_agent.train_step_counter.numpy()
        #     if step % log_interval == 0:
        #         print('step = {0}: loss = {1}'.format(step, train_loss.loss))
        #     if step % eval_interval == 0:
        #         avg_return = compute_avg_return(eval_env, eval_policy, num_eval_episodes)
        #         print('step = {0}: Average Return = {1}'.format(step, avg_return))
        #         returns.append(avg_return)
        #     global_step_val = global_step.numpy()
        #     if global_step_val % train_checkpoint_interval == 0:
        #         train_checkpointer.save(global_step=global_step_val)
        #
        #     if global_step_val % policy_checkpoint_interval == 0:
        #         policy_checkpointer.save(global_step=global_step_val)
        #
        #     if global_step_val % rb_checkpoint_interval == 0:
        #         rb_checkpointer.save(global_step=global_step_val)

    # the observation is the x position of the vehicle
    # note that a deepcopy is used when a history
    # of observations is desired. This is because
    # the sensor plugin edits the data in-place
        # obs = []
        # temp_obs = copy.deepcopy(env.reset())
        # obs.append(temp_obs)
        # total_reward = 0
        # for _ in range(num_iterations):
        #     temp_obs = copy.deepcopy(env.reset())
        #     for i in range(200):
        #         action = get_action(temp_obs)
        #         temp_obs, reward, done = env.step(action)[:3]
        #         obs.append(copy.deepcopy(temp_obs))
        #         total_reward += reward
        #         print("Total Reward: ")
        #         print(total_reward)
        #         print("\n")
        #         if done:
        #             break

        train_env.close()
    print("Total Reward:")

if __name__ == '__main__':
    test_openai()

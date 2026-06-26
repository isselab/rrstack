# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri, Thorsten Berger
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer reinforcement learning package.

import argparse
import os

from roboracer_rl.gym_env import RoboRacerEnv


def parse_args():
    parser = argparse.ArgumentParser(
        description='Train a minimal RL steering controller in Gazebo.'
    )
    parser.add_argument('--algorithm', choices=['dqn', 'ppo'], default='dqn')
    parser.add_argument('--timesteps', type=int, default=100_000)
    parser.add_argument('--output', default='models/roboracer_dqn')
    parser.add_argument('--lidar-bins', type=int, default=36)
    parser.add_argument('--fixed-speed', type=float, default=1.5)
    parser.add_argument('--step-duration', type=float, default=0.1)
    parser.add_argument('--episode-seconds', type=float, default=30.0)
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--reset-service', default='/reset_world')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    try:
        from stable_baselines3 import DQN, PPO
        from stable_baselines3.common.monitor import Monitor
    except ImportError as exc:
        raise SystemExit(
            'Training requires Stable-Baselines3. Install dependencies with:\n'
            '  pip install gymnasium stable-baselines3'
        ) from exc

    env = RoboRacerEnv(
        scan_topic=args.scan_topic,
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        reset_service=args.reset_service,
        lidar_bins=args.lidar_bins,
        fixed_speed=args.fixed_speed,
        step_duration=args.step_duration,
        episode_seconds=args.episode_seconds,
    )
    env = Monitor(env)

    if args.algorithm == 'dqn':
        model = DQN(
            'MlpPolicy',
            env,
            learning_rate=1e-4,
            buffer_size=50_000,
            learning_starts=1_000,
            batch_size=64,
            gamma=0.98,
            train_freq=4,
            target_update_interval=1_000,
            exploration_fraction=0.25,
            exploration_final_eps=0.05,
            verbose=1,
        )
    else:
        model = PPO(
            'MlpPolicy',
            env,
            learning_rate=3e-4,
            n_steps=512,
            batch_size=64,
            gamma=0.98,
            verbose=1,
        )

    try:
        model.learn(total_timesteps=args.timesteps)
        output = args.output
        os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
        model.save(output)
        print(f'Saved trained model to {output}.zip')
    finally:
        env.close()


if __name__ == '__main__':
    main()

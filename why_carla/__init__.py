from gym.envs.registration import register

register(
    id='why_carla-v0',
    entry_point='why_carla.envs:CarlaEnv',
)
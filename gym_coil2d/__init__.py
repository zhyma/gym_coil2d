from gym.envs.registration import register

register(
    id='coil2d-v0',
    entry_point='gym_coil2d.envs:Coil2DEnv',
)
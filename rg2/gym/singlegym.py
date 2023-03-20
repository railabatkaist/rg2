class rggymenv(VecEnv):
    def __init__(self, impl):
        super().__init__(impl)

        self.observation_space = spaces.Box(
            low=-1e6, high=1e6, shape=(self.num_obs,), dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-10, high=10, shape=(self.num_acts,), dtype=np.float32
        )

        assert self.num_envs == 1, "Only single environment is supported"

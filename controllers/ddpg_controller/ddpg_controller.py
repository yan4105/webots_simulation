"""ddpg_controller controller."""

from controller import Robot
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV


class ddpg_supervisor(SupervisorCSV):
    def __init__(self):
        super().__init__()

    def get_observations(self):
        return [1, 1, 1, 1]

    def get_reward(self, action):
        return 1

    def get_info(self):
        return None

    def is_done(self):
        return False


supervisor = ddpg_supervisor()
print("running")

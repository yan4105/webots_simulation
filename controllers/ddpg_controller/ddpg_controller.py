# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
#from PPOAgent import PPOAgent, Transition
#from utilities import normalizeToRange

class ddpg_controller(SupervisorCSV):
    def __init__(self):
        super().__init__()
        self.observationSpace = 4 # The agent has four inputs
        self.actionSpace = 2 # the agent can perform two actions
        self.robot = None
        self.respawnRobot()
        #self.poleEndpoint = self.supervisor.getFromDef("POLE_ENDPOINT")
        self.messageReceived = None # Variable to save the messages received from the robot
        self.episodeCount = 0 # Episode counter
        self.episodeLimit = 10000 # Max number of episodes allowed
        self.stepPerEpisode = 200 # Max number of steps per episode
        self.episodeScore = 0 # Score accumulated during an episode
        self.episodeScoreList = [] # A list to save all the episode scores, used to check if task is solved

    def respawnRobot(self):
        if self.robot is not None:
            # Despawn existing robot
            self.robot.remove()

        # Respawn robot in starting position and state
        rootNode = self.supervisor.getRoot() # This gets the root of the scene tree
        childrenField = rootNode.getField("children") # This gets a list of all the children, ie. objects of the scene
        childrenField.importMFNode(-2, "Robot.wbo") # Load robot from file and add to second-to-last position

        # Get the new robot and pole endpoint references
        self.robot = self.supervisor.getFromDef("ROBOT")
        #self.poleEndpoint = self.supervisor.getFromDef("POLE_ENDPOINT")

    def get_observations(self):
        self.messageReceived = self.handle_receiver()
        return [1, 1, 1, 1]

    def get_reward(self, action=None):
        # TODO: change this to match our robot
        # -10 for topple
        # abs(pos, orig) as reward ?
        return 1

    def is_done(self):
        return False

    def solved(self):
        if len(self.episodeScoreList) > 100: # Over 100 trials thus far
            if np.mean(self.episodeScoreList[-100:]) > 195.0: # Last 100 episodes' scores average value
                return True
        return False

    def reset(self):
        self.respawnRobot()
        self.supervisor.simulationResetPhysics() # Reset the simulation physics to start over
        self.messageReceived = None
        return [0.0 for _ in range(self.observationSpace)]

    def get_info(self):
        return None

supervisor = ddpg_controller()
#agent = PPOAgent(supervisor.observationSpace, supervisor.actionSpace)
solved = False


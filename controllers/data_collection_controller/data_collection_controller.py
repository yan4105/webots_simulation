# Inputs: 1) position of four motors 2) velocity of four motors 3) IMU
# Outputs: 4)


from controller import Robot
import numpy as np
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
import ast
import math
import csv
import random
from PPOAgent import PPOAgent, Transition
from utilities import normalizeToRange

class ddpg_controller(SupervisorCSV):
    def __init__(self):
        super().__init__()
        self.observationSpace = 30 # The agent has four inputs
        self.actionSpace = 36 # the agent can perform two actions
        self.robot = None
        self.terrain = None
        self.resetTerrain()
        self.respawnRobot()
        self.IMU = self.supervisor.getFromDef("IMU")
        #self.poleEndpoint = self.supervisor.getFromDef("POLE_ENDPOINT")
        self.messageReceived = None # Variable to save the messages received from the robot
        self.episodeCount = 0 # Episode counter
        self.episodeLimit = 200000 # Max number of episodes allowed
        self.stepPerEpisode = 100 # Max number of steps per episode
        self.episodeScore = 0 # Score accumulated during an episode
        self.episodeScoreList = [] # A list to save all the episode scores, used to check if task is solved
        self.last_x = 0
        self.max_x = 0
        self.stationary_count = 0
        self.step_count = 1
        self.done = False

    def resetTerrain(self):
        if self.terrain is not None:
            self.terrain.remove()
        rootNode = self.supervisor.getRoot()  # This gets the root of the scene tree
        childrenField = rootNode.getField("children")  # This gets a list of all the children, ie. objects of the scen
        childrenField.importMFNode(-1, "terrain.wbo") # Load robot from file and add to second-to-last position
        self.terrain = self.supervisor.getFromDef("TERRAIN")
        self.terrain.randomSeed = math.floor(random.random() * 10)

    def respawnRobot(self):
        if self.robot is not None:
            self.robot.remove()
        rootNode = self.supervisor.getRoot() # This gets the root of the scene tree
        childrenField = rootNode.getField("children") # This gets a list of all the children, ie. objects of the scene
        childrenField.importMFNode(-2, "logging.wbo") # Load robot from file and add to second-to-last position

        self.robot = self.supervisor.getFromDef("ROBOT")


    def get_observations(self):
        self.messageReceived = self.handle_receiver()
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            #print("message received", message)
            return message
        else:
            return [0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0]

    def get_reward(self, action=None):
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            x, h, y = message[27], message[28], message[29]
            if abs(x - self.last_x) < 1e-6:
                self.stationary_count += 1
            else:
                self.stationary_count = 0
            self.last_x = x
        return 0

    def is_done(self):
        if self.stationary_count > 30:
            if self.is_topple():
                print("log: fell over")
            else:
                print("log: stable")
                self.store_data()
            return True
        return False

    def is_topple(self):
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            x, h, y = message[27], message[28], message[29]
            roll, pitch, yaw = message[0], message[1], message[2]
            cri = abs(roll) + abs(pitch)
            if cri > 1.5:
                return True
        return False

    def solved(self):
        return False

    def reset(self):
        #self.resetTerrain()
        self.respawnRobot()
        self.supervisor.simulationResetPhysics() # Reset the simulation physics to start over
        self.messageReceived = None
        self.last_x = 0
        self.max_x = 0
        self.stationary_count = 0
        self.step_count = 0
        self.done = False
        return [0.0 for _ in range(self.observationSpace)]

    def get_info(self):
        return None

    def store_data(self):
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            roll, pitch, yaw, \
         hipx_a_pos, hipx_b_pos, hipx_c_pos, hipx_d_pos,\
         hipx_a_vel, hipx_b_vel, hipx_c_vel, hipx_d_vel, \
         hipy_a_pos, hipy_b_pos, hipy_c_pos, hipy_d_pos,  \
         hipy_a_vel, hipy_b_vel, hipy_c_vel, hipy_d_vel,  \
         leg_a_pos, leg_b_pos, leg_c_pos, leg_d_pos,  \
         leg_a_vel, leg_b_vel, leg_c_vel, leg_d_vel,  \
         x, h, y = message
        with open("data.txt", mode="a") as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=',')
            csv_writer.writerow(message)
        return

supervisor = ddpg_controller()
agent = PPOAgent(supervisor.observationSpace, supervisor.actionSpace)
agent.load("")
solved = False

# Run outer loop until the episodes limit is reached or the task is solved
while not solved and supervisor.episodeCount < supervisor.episodeLimit:
    observation = supervisor.reset() # Reset robot and get starting observation
    supervisor.episodeScore = 0
    for step in range(supervisor.stepPerEpisode):
        #print(step)
        # In training mode the agent samples from the probability distribution, naturally implementing exploration
        selectedAction, actionProb = agent.work(observation, type_="selectAction")

        # Step the supervisor to get the current selectedAction's reward, the new observation and whether we reached
        # the done condition
        newObservation, reward, done, info = supervisor.step([selectedAction])

        # Save the current state transition in agent's memory
        trans = Transition(observation, selectedAction, actionProb, reward, newObservation)
        agent.storeTransition(trans)

        if done:
            # Save the episode's score
            supervisor.episodeScoreList.append(supervisor.episodeScore)
            agent.trainStep(batchSize=step)
            solved = supervisor.solved() # Check whether the task if solved
            break

        supervisor.episodeScore += reward # Accumulate episode reward
        observation = newObservation # observation for next step is current step's newObservation
    print("Episode #", supervisor.episodeCount, "score:", supervisor.episodeScore)
    agent.save("")
    supervisor.episodeCount += 1 # Increment episode counter

if not solved:
    print("Task is not solved, deploying agent for testing")
elif solved:
    print("Task is solved, deploying agent for testing...")
observation = supervisor.reset()

#while True:
#    selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
#    observation, _, _, _ = supervisor.step([selectedAction])
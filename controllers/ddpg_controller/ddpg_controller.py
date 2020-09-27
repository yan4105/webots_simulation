# Inputs: 1) position of four motors 2) velocity of four motors 3) IMU
# Outputs: 4)


from controller import Robot
import numpy as np
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
import ast
from PPOAgent import PPOAgent, Transition
from utilities import normalizeToRange

class ddpg_controller(SupervisorCSV):
    def __init__(self):
        super().__init__()
        self.observationSpace = 11 # The agent has four inputs
        self.actionSpace = 8 # the agent can perform two actions
        self.robot = None
        self.respawnRobot()
        self.IMU = self.supervisor.getFromDef("IMU")
        #self.poleEndpoint = self.supervisor.getFromDef("POLE_ENDPOINT")
        self.messageReceived = None # Variable to save the messages received from the robot
        self.episodeCount = 0 # Episode counter
        self.episodeLimit = 20000 # Max number of episodes allowed
        self.stepPerEpisode = 100 # Max number of steps per episode
        self.episodeScore = 0 # Score accumulated during an episode
        self.episodeScoreList = [] # A list to save all the episode scores, used to check if task is solved
        self.abs_last = 0
        self.got_big_reward = 0

    def respawnRobot(self):
        #print("respawn called")
        if self.robot is not None:
            # Despawn existing robot
        #    print("called")
            self.robot.remove()
        # Respawn robot in starting position and state
        #print("self.robot is None")
        rootNode = self.supervisor.getRoot() # This gets the root of the scene tree
        childrenField = rootNode.getField("children") # This gets a list of all the children, ie. objects of the scene
        childrenField.importMFNode(-2, "Robot_imu.wbo") # Load robot from file and add to second-to-last position

        # Get the new robot and pole endpoint references
        self.robot = self.supervisor.getFromDef("ROBOT")


    def get_observations(self):
        # Leg positions
        #hipy_a_pos = normalizeToRange(self.robot.hip()[2], -0.4, 0.4, -1.0, 1.0)
        # Linear velocity on z axis
        #cartVelocity = normalizeToRange(self.robot.getVelocity()[2], -0.2, 0.2, -1.0, 1.0, clip=True)
        # Angular velocity x of endpoint
        #endPointVelocity = normalizeToRange(self.poleEndpoint.getVelocity()[3], -1.5, 1.5, -1.0, 1.0, clip=True)

        self.messageReceived = self.handle_receiver()
        #if self.messageReceived is not None:
        #    poleAngle = normalizeToRange(float(self.messageReceived[0]), -0.23, 0.23, -1.0, 1.0, clip=True)
        #else:
        #    # Method is called before self.messageReceived is initialized
        #    poleAngle = 0.0
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            #print("message received", message)
            return message
        else:
            return [0,0,0,0,0,0,0,0,0,0,0]

    def get_reward(self, action=None):
        # TODO: change this to match our robot
        # -10 for topple
        # abs(pos, orig) as reward ?
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            roll, pitch, yaw = message[0], message[1], message[2]
            #print("reward", (abs(roll) + abs(pitch) - self.abs_last) * 10)
            cri = (roll + pitch) / 10
            if cri > self.abs_last:
                reward = (cri - self.abs_last) * 1000
            else:
                reward = -0.1
            self.abs_last = cri
            if (cri > 0.05):
                self.got_big_reward += 1
            #    print("got 20000")
                reward += 20000
            elif (cri > 0.04):
                self.got_big_reward += 1
            #    print("got 10000")
                reward += 10000
            elif (cri > 0.03):
                self.got_big_reward += 1
            #    print("got 5000")
                reward += 5000
            elif (cri > 0.02):
                self.got_big_reward += 1
            #    print("got 2000")
                reward += 2000
            elif (cri > 0.005):
            #    print("got 500")
                reward += 500
            else:
                reward -= 1000
            #else:
            #    print(cri)
            #    if (self.got_big_reward > 0):
            #        reward -= 1000
            #    else:
            #        reward -= 1000
            if (cri > 0.5):
            #    print("got big reward")
                reward += 10000
            #if (cri > 0.02):
            #    reward = cri * 100
            #if (cri > 0.01):
            #    reward = cri
            #else:
            #    reward = -0.01
            #print(self.abs_last)
            return reward
        return 0

    def is_done(self):
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            roll, pitch, yaw = message[0], message[1], message[2]
            #print("reward", (abs(roll) + abs(pitch) - self.abs_last) * 10)
            cri = roll + pitch
            if cri < -0.01:
                return True
        return False

    def solved(self):
        #if len(self.episodeScoreList) > 100: # Over 100 trials thus far
        #    if np.mean(self.episodeScoreList[-100:]) > 1000.0: # Last 100 episodes' scores average value
        #        return True
        return False

    def reset(self):
        #print("reset called")
        self.respawnRobot()
        self.supervisor.simulationResetPhysics() # Reset the simulation physics to start over
        self.messageReceived = None
        return [0.0 for _ in range(self.observationSpace)]

    def get_info(self):
        return None

supervisor = ddpg_controller()
agent = PPOAgent(supervisor.observationSpace, supervisor.actionSpace)
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
    supervisor.episodeCount += 1 # Increment episode counter

if not solved:
    print("Task is not solved, deploying agent for testing")
elif solved:
    print("Task is solved, deploying agent for testing...")
observation = supervisor.reset()
while True:
    selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
    observation, _, _, _ = supervisor.step([selectedAction])
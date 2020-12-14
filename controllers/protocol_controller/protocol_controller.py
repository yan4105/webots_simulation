# Inputs: 1) position of four motors 2) velocity of four motors 3) IMU
# Outputs: 4)


from controller import Robot
import numpy as np
from deepbots.supervisor.controllers.supervisor_emitter_receiver import SupervisorCSV
import ast
import math
import csv
from PPOAgent import PPOAgent, Transition
from utilities import normalizeToRange

LOG = False
STATIONARY_THRES = 5000
MOVE_THRES = 1e-3

class ddpg_controller(SupervisorCSV):
    def __init__(self):
        super().__init__()
        self.observationSpace = 30 # The agent has four inputs
        self.actionSpace = 36 # the agent can perform two actions
        self.robot = None
        self.respawnRobot()
        self.IMU = self.supervisor.getFromDef("IMU")
        #self.poleEndpoint = self.supervisor.getFromDef("POLE_ENDPOINT")
        self.messageReceived = None # Variable to save the messages received from the robot
        self.episodeCount = 0 # Episode counter
        self.episodeLimit = 2000000 # Max number of episodes allowed
        self.stepPerEpisode = 400000 # Max number of steps per episode
        self.episodeScore = 0 # Score accumulated during an episode
        self.episodeScoreList = [] # A list to save all the episode scores, used to check if task is solved
        self.last_x = -0.5
        self.max_x = 0
        self.stationary_count = 0
        self.step_count = 1
        self.last_big_x = 0
        self.done = False
        self.state = 0 # 0 means level, 1 means step, 2 means level, 3 means turn
        self.last_hipx_pos = [0, 0, 0, 0]
        self.last_leg_raised = 0
        self.last_leg_raised_count = 0
        self.last_leg_raised_THRES = 30
        self.last_cri = 0
        self.last_cri_count = 0
        self.last_h = 0
        self.last_h_count = 0
        self.last_yaw = 0
        self.last_yaw_count = 0

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
        childrenField.importMFNode(-2, "Robot_walk.wbo") # Load robot from file and add to second-to-last position

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
            self.store_message(message)
            return message
        else:
            return [0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0]

    def is_topple(self, message):
        roll, pitch, yaw = message[0], message[1], message[2]
        cri = abs(roll) + abs(pitch)
        if abs(cri) > 2.1:
            return True

    def is_leg_raised(self, hipy):
        for hip in hipy:
            if hip < 0.45:
                return True
        return False

    def get_reward(self, action=None):
        # TODO: change this to match our robot
        # -10 for topple
        # abs(pos, orig) as reward ?
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            roll, pitch, yaw = message[0], message[1], message[2]
            x, h, y = message[27], message[28], message[29]
            [hipx_a_pos, hipx_b_pos, hipx_c_pos, hipx_d_pos] = [message[3], message[4], message[5], message[6]]
            [hipy_a_pos, hipy_b_pos, hipy_c_pos, hipy_d_pos] = [message[11], message[12], message[13], message[14]]
            hipx_pos = [hipx_a_pos, hipx_b_pos, hipx_c_pos, hipx_d_pos]
            hipy_pos = [hipy_a_pos, hipy_b_pos, hipy_c_pos, hipy_d_pos]
            leg_raised = np.argmax(hipy_pos)
            is_leg_raised = self.is_leg_raised(hipy_pos)
            if self.last_leg_raised == leg_raised:
                self.last_leg_raised_count += 1
            else:
                self.last_leg_raised = leg_raised
                self.last_leg_raised_count = 0

            posture = hipy_a_pos + hipy_b_pos + hipy_c_pos + hipy_d_pos
            posture = 4 - posture + h * 10
            movement = [self.last_hipx_pos[i] - hipx_pos[i] for i in range(len(self.last_hipx_pos))]
            movement = [abs(i) for i in movement]
            movement = sum(movement)
            posture += movement
            if is_leg_raised:
                posture += 1
            else:
                posture = 0
            if self.state == 0 or self.state == 2:
                cri = abs(roll) + abs(pitch)
                #if abs(h - self.last_h) < MOVE_THRES:
                #    self.last_h_count += 1
                #else:
                #    self.last_h = h
                #    self.last_h_count = 0
                #cri = abs(roll) + abs(pitch)
                #if abs(self.last_cri - cri) < MOVE_THRES:
                #    self.last_cri_count += 1
                #else:
                #    self.last_cri_count = 0
                #    self.last_cri = cri
                return 0.25 - cri
                #if self.last_cri_count > STATIONARY_THRES / 2:
                #    return -10
                #return 0.15 - cri
            elif self.state == 1:
                #if self.stationary_count > STATIONARY_THRES / 2:
                #    return -1
                #if self.last_leg_raised_count > self.last_leg_raised_THRES:
                #    return -1
                if self.messageReceived is not None:
                    # if abs(cri) > 2.1:
                    #    return -self.stepPerEpisode // 2
                    if self.is_topple(message):
                        return -1000000
                    return x - self.last_x
                    #if math.floor(x) > self.last_big_x:
                    #    self.last_big_x = math.floor(x)
                    #    return 100 * self.last_big_x + posture * 100
                    #if x - self.max_x < MOVE_THRES:
                    #    self.stationary_count += 1
                    #else:
                    #    self.stationary_count = 0
                    #    self.max_x = x
                    #    return 1000
                    #if self.stationary_count > STATIONARY_THRES / 2:
                    #    return -1
                    #return 0
                print("no message received in state 1")
                return 0
            elif self.state == 3:
                if self.messageReceived is not None:
                    message = self.messageReceived
                    message = [float(i) for i in message]
                    roll, pitch, yaw = message[0], message[1], message[2]
                    if not yaw < self.last_yaw and abs(yaw - self.last_yaw) < MOVE_THRES:
                        self.last_yaw_count += 1
                    else:
                        self.last_yaw = yaw
                        self.last_h_count = 0
                    return 0.5 - yaw
            else:
                return -100
        print("no message received")
        return -1000 / self.stepPerEpisode

    # 0 is not done, 1 is done, 2 is we are not waiting
    def is_done(self):
        if self.messageReceived is not None:
            message = self.messageReceived
            message = [float(i) for i in message]
            if self.is_topple(message):
                self.state = 4
                return 2
        else:
            return 0
        if self.state == 0 or self.state == 2:
            if self.messageReceived is not None:
                message = self.messageReceived
                message = [float(i) for i in message]
                roll, pitch, yaw = message[0], message[1], message[2]
                x, h, y = message[27], message[28], message[29]
                #print(y)
                #print("reward", (abs(roll) + abs(pitch) - self.abs_last) * 10)
                cri = abs(roll) + abs(pitch)
                if abs(cri) < 0.2:
                    return 1
                if self.last_h_count > STATIONARY_THRES or self.last_cri_count > STATIONARY_THRES:
                    #return 2
                    return 0
            #return False
            return 0
        elif self.state == 1:
            if self.messageReceived is not None:
                message = self.messageReceived
                message = [float(i) for i in message]
                roll, pitch, yaw = message[0], message[1], message[2]
                x, h, y = message[27], message[28], message[29]
                if self.stationary_count > STATIONARY_THRES:
                    #return 2
                    return 0
                if x - self.last_x > 0.001:
                    self.last_x = x
                    return 1
            return 0
        elif self.state == 3:
            if self.messageReceived is not None:
                message = self.messageReceived
                message = [float(i) for i in message]
                roll, pitch, yaw = message[0], message[1], message[2]
                if yaw < 0.2:
                    return 1
                if self.last_yaw_count > STATIONARY_THRES:
                #    return 2
                    return 0
            return 0

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
        self.last_x = -0.5
        self.max_x = 0
        self.stationary_count = 0
        self.step_count = 0
        self.done = False
        self.state = 0
        self.last_hipx_pos = [0, 0, 0, 0]
        self.last_leg_raised = 0
        self.last_leg_raised_count = 0
        self.last_leg_raised_THRES = 30
        self.last_cri = 0
        self.last_cri_count = 0
        return [0.0 for _ in range(self.observationSpace)]

    def update_state(self):
        self.state = (self.state + 1 ) % 4
        self.stationary_count = 0
        print("state is " + str(self.state))

    def get_info(self):
        return None

    def store_message(self, message):
        roll, pitch, yaw, \
        hipx_a_pos, hipx_b_pos, hipx_c_pos, hipx_d_pos, \
        hipx_a_vel, hipx_b_vel, hipx_c_vel, hipx_d_vel, \
        hipy_a_pos, hipy_b_pos, hipy_c_pos, hipy_d_pos, \
        hipy_a_vel, hipy_b_vel, hipy_c_vel, hipy_d_vel, \
        leg_a_pos, leg_b_pos, leg_c_pos, leg_d_pos, \
        leg_a_vel, leg_b_vel, leg_c_vel, leg_d_vel, \
        x, h, y = message
        if LOG:
            with open("data.txt", mode="a") as csv_file:
                csv_writer = csv.writer(csv_file, delimiter=',')
                csv_writer.writerow(message)


supervisor = ddpg_controller()
agent = PPOAgent(supervisor.observationSpace, supervisor.actionSpace)
agent.load("")
solved = False

def chooseWeight(state):
    if state == 0 or state == 2:
        return ""
    elif state == 1:
        return "reset"
    else:
        return "turn"

# Run outer loop until the episodes limit is reached or the task is solved
while not solved and supervisor.episodeCount < supervisor.episodeLimit:
    observation = supervisor.reset() # Reset robot and get starting observation
    supervisor.episodeScore = 0
    for step in range(supervisor.stepPerEpisode):
        agent.load(chooseWeight(supervisor.state))
        #print(step)
        # In training mode the agent samples from the probability distribution, naturally implementing exploration
        selectedAction, actionProb = agent.work(observation, type_="selectAction")

        # Step the supervisor to get the current selectedAction's reward, the new observation and whether we reached
        # the done condition
        newObservation, reward, done, info = supervisor.step([selectedAction])
        print(supervisor.state, reward)

        # Save the current state transition in agent's memory
        trans = Transition(observation, selectedAction, actionProb, reward, newObservation)
        agent.storeTransition(trans)

        if done != 0:
            if supervisor.state == 4:
                print("topple")
                break
            if done == 2:
                supervisor.episodeScoreList.append(supervisor.episodeScore)
                agent.trainStep(batchSize=step)
                agent.save(chooseWeight(supervisor.state))
                break
            # Save the episode's score
            supervisor.episodeScoreList.append(supervisor.episodeScore)
            agent.trainStep(batchSize=step)
            #solved = supervisor.solved() # Check whether the task if solved
            agent.save(chooseWeight(supervisor.state))
            supervisor.update_state()
            #break

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
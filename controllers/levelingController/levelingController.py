# Inputs: 1) position of four motors 2) velocity of four motors 3) IMU
# Outputs: 4)

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math

from controller import Robot
from deepbots.robots.controllers.robot_emitter_receiver_csv import RobotEmitterReceiverCSV
import ast # for string to list

class levelingRobot(RobotEmitterReceiverCSV):
    def __init__(self):
        super().__init__()
        self.IMU = self.robot.getInertialUnit("inertial unit")
        self.IMU.enable(self.get_timestep())
        self.hipy_a = self.robot.getMotor("hipy_a")
        self.hipy_a.setPosition(float("inf"))
        self.hipy_a.setVelocity(0)
        self.hipy_a.getPositionSensor().enable(self.get_timestep())
        self.hipy_b = self.robot.getMotor("hipy_b")
        self.hipy_b.setPosition(float("inf"))
        self.hipy_b.setVelocity(0)
        self.hipy_b.getPositionSensor().enable(self.get_timestep())
        self.hipy_c = self.robot.getMotor("hipy_c")
        self.hipy_c.setPosition(float("inf"))
        self.hipy_c.setVelocity(0)
        self.hipy_c.getPositionSensor().enable(self.get_timestep())
        self.hipy_d = self.robot.getMotor("hipy_d")
        self.hipy_d.setPosition(float("inf"))
        self.hipy_d.setVelocity(1)
        self.hipy_d.getPositionSensor().enable(self.get_timestep())

    def create_message(self):
        # Read the sensor value, convert to string and save it in a list
        roll, pitch, yaw = self.IMU.getRollPitchYaw()
        hipy_a_pos = self.hipy_a.getPositionSensor().getValue()
        hipy_b_pos = self.hipy_b.getPositionSensor().getValue()
        hipy_c_pos = self.hipy_c.getPositionSensor().getValue()
        hipy_d_pos = self.hipy_d.getPositionSensor().getValue()
        hipy_a_vel = self.hipy_a.getVelocity()
        hipy_b_vel = self.hipy_b.getVelocity()
        hipy_c_vel = self.hipy_c.getVelocity()
        hipy_d_vel = self.hipy_d.getVelocity()
        message_list = [roll, pitch, yaw, hipy_a_pos, hipy_b_pos, hipy_c_pos, hipy_d_pos,
                        hipy_a_vel, hipy_b_vel, hipy_c_vel, hipy_d_vel]
        #print(ast.literal_eval(str(message_list))[0])
        #print("message:", str(message_list))
        return message_list
        #return ["abcd"]

    def use_message_data(self, message):
        #print ("actor got this:", message)
        action = math.floor(float(message[0])) # Convert the string message into an action integer
        motor_idx = action // 2
        action = action % 2
        motor = None
        if motor_idx == 0:
            motor = self.hipy_a
        elif motor_idx == 1:
            motor = self.hipy_b
        elif motor_idx == 2:
            motor = self.hipy_c
        else:
            motor = self.hipy_d
        #print(motor_idx,action)
        if action == 0:
            motor.setVelocity(0)
        else:
            motor.setVelocity(1)

# Create the robot controller object and run it
robot_controller = levelingRobot()
robot_controller.run() # Run method is implemented by the framework, just need to call it

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

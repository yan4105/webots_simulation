# Inputs: 1) position of four motors 2) velocity of four motors 3) IMU
# Outputs: 4)

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import random

from controller import Robot
from deepbots.robots.controllers.robot_emitter_receiver_csv import RobotEmitterReceiverCSV
import ast # for string to list

class loggingRobot(RobotEmitterReceiverCSV):
    def __init__(self):
        super().__init__()
        self.IMU = self.robot.getInertialUnit("inertial unit")
        self.IMU.enable(self.get_timestep())
        self.GPS = self.robot.getGPS("GPS")
        self.GPS.enable(self.get_timestep())
        self.init_motors()
        #self.random_init()

    def create_message(self):
        roll, pitch, yaw = self.IMU.getRollPitchYaw()
        x, h, y = self.GPS.getValues()
        hipx_a_pos = self.hipx_a.getPositionSensor().getValue()
        hipx_b_pos = self.hipx_b.getPositionSensor().getValue()
        hipx_c_pos = self.hipx_c.getPositionSensor().getValue()
        hipx_d_pos = self.hipx_d.getPositionSensor().getValue()
        hipx_a_vel = self.hipx_a.getVelocity()
        hipx_b_vel = self.hipx_b.getVelocity()
        hipx_c_vel = self.hipx_c.getVelocity()
        hipx_d_vel = self.hipx_d.getVelocity()
        hipy_a_pos = self.hipy_a.getPositionSensor().getValue()
        hipy_b_pos = self.hipy_b.getPositionSensor().getValue()
        hipy_c_pos = self.hipy_c.getPositionSensor().getValue()
        hipy_d_pos = self.hipy_d.getPositionSensor().getValue()
        hipy_a_vel = self.hipy_a.getVelocity()
        hipy_b_vel = self.hipy_b.getVelocity()
        hipy_c_vel = self.hipy_c.getVelocity()
        hipy_d_vel = self.hipy_d.getVelocity()
        leg_a_pos = self.leg_a.getPositionSensor().getValue()
        leg_b_pos = self.leg_b.getPositionSensor().getValue()
        leg_c_pos = self.leg_c.getPositionSensor().getValue()
        leg_d_pos = self.leg_d.getPositionSensor().getValue()
        leg_a_vel = self.leg_a.getVelocity()
        leg_b_vel = self.leg_b.getVelocity()
        leg_c_vel = self.leg_c.getVelocity()
        leg_d_vel = self.leg_d.getVelocity()
        message_list = [roll, pitch, yaw,  # 3
                        hipx_a_pos, hipx_b_pos, hipx_c_pos, hipx_d_pos,  # 7
                        hipx_a_vel, hipx_b_vel, hipx_c_vel, hipx_d_vel,  # 11
                        hipy_a_pos, hipy_b_pos, hipy_c_pos, hipy_d_pos,  # 15
                        hipy_a_vel, hipy_b_vel, hipy_c_vel, hipy_d_vel,  # 19
                        leg_a_pos, leg_b_pos, leg_c_pos, leg_d_pos,  # 23
                        leg_a_vel, leg_b_vel, leg_c_vel, leg_d_vel,  # 27
                        x, h, y]  # 30
        # print(ast.literal_eval(str(message_list))[0])
        # print("message:", str(message_list))
        return message_list

    def use_message_data(self, message):
        #motors = [self.hipx_a, self.hipx_b, self.hipx_c, self.hipx_d]
        #print ("actor got this:", message)
        #action = math.floor(float(message[0])) # Convert the string message into an action integer
        #motor_idx = action // 3
        #action = action % 3
        #motor = motors[motor_idx]
        pass

    def init_motors(self):
        self.hipx_a = self.robot.getMotor("hipx_a")
        self.hipx_a.setPosition(0)
        #self.hipx_a.setPosition(float("inf"))
        #self.hipx_a.setVelocity(0)
        self.hipx_a.getPositionSensor().enable(self.get_timestep())
        self.hipx_b = self.robot.getMotor("hipx_b")
        self.hipx_b.setPosition(0)
        #self.hipx_b.setPosition(float("inf"))
        #self.hipx_b.setVelocity(0)
        self.hipx_b.getPositionSensor().enable(self.get_timestep())
        self.hipx_c = self.robot.getMotor("hipx_c")
        self.hipx_c.setPosition(0)
        #self.hipx_c.setPosition(float("inf"))
        #self.hipx_c.setVelocity(0)
        self.hipx_c.getPositionSensor().enable(self.get_timestep())
        self.hipx_d = self.robot.getMotor("hipx_d")
        self.hipx_d.setPosition(0)
        #self.hipx_d.setPosition(float("inf"))
        #self.hipx_d.setVelocity(0)
        self.hipx_d.getPositionSensor().enable(self.get_timestep())

        self.hipy_a = self.robot.getMotor("hipy_a")
        self.hipy_a.setPosition(self.r())
        self.hipy_a.getPositionSensor().enable(self.get_timestep())
        self.hipy_b = self.robot.getMotor("hipy_b")
        self.hipy_b.setPosition(self.r())
        self.hipy_b.getPositionSensor().enable(self.get_timestep())
        self.hipy_c = self.robot.getMotor("hipy_c")
        self.hipy_c.setPosition(self.r())
        self.hipy_c.getPositionSensor().enable(self.get_timestep())
        self.hipy_d = self.robot.getMotor("hipy_d")
        self.hipy_d.setPosition(self.r())
        self.hipy_d.getPositionSensor().enable(self.get_timestep())

        self.leg_a = self.robot.getMotor("leg_a")
        self.leg_a.setPosition(0)
        #self.leg_a.setPosition(float("inf"))
        #self.leg_a.setVelocity(0)
        self.leg_a.getPositionSensor().enable(self.get_timestep())
        self.leg_b = self.robot.getMotor("leg_b")
        self.leg_b.setPosition(0)
        #self.leg_b.setPosition(float("inf"))
        #self.leg_b.setVelocity(0)
        self.leg_b.getPositionSensor().enable(self.get_timestep())
        self.leg_c = self.robot.getMotor("leg_c")
        self.leg_c.setPosition(0)
        #self.leg_c.setPosition(float("inf"))
        #self.leg_c.setVelocity(0)
        self.leg_c.getPositionSensor().enable(self.get_timestep())
        self.leg_d = self.robot.getMotor("leg_d")
        self.leg_d.setPosition(0)
        #self.leg_d.setPosition(float("inf"))
        #self.leg_d.setVelocity(0)
        self.leg_d.getPositionSensor().enable(self.get_timestep())

    def r(self):
        return random.random()

    def random_init(self):
        motors = [self.hipy_a, self.hipy_b, self.hipy_c, self.hipy_d]
        log = []
        for motor in motors:
            r = 1 #if random.random() > 0.5 else -1
            r *= random.random() * 10
            log.append(r)
            motor.setPosition(r)
        print(log)


# Create the robot controller object and run it
robot_controller = loggingRobot()
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

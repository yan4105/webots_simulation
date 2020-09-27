"""robotController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from deepbots.robots.controllers.robot_emitter_receiver_csv import RobotEmitterReceiverCSV

class CartpoleRobot(RobotEmitterReceiverCSV):
    def __init__(self):
        super().__init__()
        self.positionSensor = self.robot.getPositionSensor("polePosSensor")
        self.positionSensor.enable(self.get_timestep())
        self.wheel1 = self.robot.getMotor("wheel1")
        self.wheel1.setPosition(float("inf"))
        self.wheel1.setVelocity(0.0)
        self.wheel2 = self.robot.getMotor("wheel2")
        self.wheel2.setPosition(float('inf'))
        self.wheel2.setVelocity(0.0)
        self.wheel3 = self.robot.getMotor("wheel3")
        self.wheel3.setPosition(float("inf"))
        self.wheel3.setVelocity(0.0)
        self.wheel4 = self.robot.getMotor("wheel4")
        self.wheel4.setPosition(float("inf"))
        self.wheel4.setVelocity(0.0)

    def create_message(self):
        # Read the sensor value, convert to string and save it in a list
        message = str(self.positionSensor.getValue())
        return message

    def use_message_data(self, message):
        action = int(message[0]) # Convert the string message into an action integer

        if action == 0:
            motorSpeed = 5.0
        elif action == 1:
            motorSpeed = -5.0
        else:
            motorSpeed = 0.0

        # Set the motor's velocities based on the action received
        self.wheel1.setVelocity(motorSpeed)
        self.wheel2.setVelocity(motorSpeed)
        self.wheel3.setVelocity(motorSpeed)
        self.wheel4.setVelocity(motorSpeed)

# Create the robot controller object and run it
robot_controller = CartpoleRobot()
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

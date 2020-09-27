"""levelingController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from deepbots.robots.controllers.robot_emitter_receiver_csv import RobotEmitterReceiverCSV

class levelingRobot(RobotEmitterReceiverCSV):
    def __init__(self):
        super().__init__()
        #self.positionSensor = self.robot.getPositionSensor("polePosSensor")
        #self.positionSensor.enable(self.get_timestep())

    def create_message(self):
        # Read the sensor value, convert to string and save it in a list
        #message = [str(self.positionSensor.getValue())]
        #return message
        return ["abcd"]

    def use_message_data(self, message):
        #action = int(message[0]) # Convert the string message into an action integer

        return 0

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

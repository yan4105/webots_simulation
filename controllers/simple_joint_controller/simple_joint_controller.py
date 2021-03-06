"""simple_joint_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

control_structures = [["hipx_a", "hipy_a", "leg_a"],
                      ["hipx_b", "hipy_b", "leg_b"],
                      ["hipx_c", "hipy_c", "leg_c"],
                      ["hipx_d", "hipy_d", "leg_d"]]


def set_motor_speed(motorName, speed):
    motor = robot.getMotor(motorName)
    motor.setPosition(float("inf"))
    motor.setVelocity(speed)

# set motor speed
def set_all_motors(speed):
    for structure in control_structures:
        for motor_name in structure:
            set_motor_speed(motor_name, speed)


set_all_motors(0.05)

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

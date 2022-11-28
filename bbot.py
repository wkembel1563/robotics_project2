from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import math

class BehavioralBot:
    def __init__(self):
        self.orient_to_candle = False
        self.touch_port = Port.S1      
        self.touch_port2 = Port.S4
        self.color_port = Port.S3      
        self.fan_port = Port.D
        self.ultrasonic_port = Port.S2 
        self.motorL_port = Port.A
        self.motorR_port = Port.B

    """
    BEHAVIORS
        independent functions which individually recommend what they think the robot should do and 
        assign a priority value to their recommendation
    
    OUTPUT 
        control_signal (string)
            - what the behavior thinks the robot should do given the current inputs, 
              assuming it is the highest priority
            - this string will pass to execute_signal() to be translated into a control function
            - possible control signal values = 'forward', 'turn_right_90', etc...

        priority (int)
            - how important the behavior thinks it is should be given the input
            - values: 0-1?

    """
    def wander(self):
        priority = 0
        sensor = UltrasonicSensor(self.ultrasonic_port)
        distance = sensor.distance()
        touchSensor = TouchSensor(self.touch_port)
        touchSensor2 = TouchSensor(self.touch_port2)
        if distance and not touchSensor.pressed() and not touchSensor2.pressed() and distance > 100: #If we are too far away from the wall, Wander.
            priority = 8
        elif not distance and not touchSensor.pressed():
            priority = 8
        else:
            priority = 0

        control_signal = "move forward"

        return priority, control_signal
        

    def wall_following(self):
        sensor = UltrasonicSensor(self.ultrasonic_port)
        touchSensor = TouchSensor(self.touch_port)
        touchSensor2 = TouchSensor(self.touch_port2)

        distance = sensor.distance()
        control_signal = None
        priority = 0
        if distance and not touchSensor.pressed() and not touchSensor2.pressed():
            if distance <= 70 and distance >= 30:
                priority = 9
                control_signal = "move forward"
            elif distance < 30 and distance >= 0:
                priority = 9
                control_signal = "slight left"
            elif distance <= 100 and distance > 70: 
                priority = 9
                control_signal = "slight right"
            elif distance <= 130 and distance > 100: 
                priority = 6
                control_signal = "slight right"
            else:
                priority = 2
                control_signal = "turn right"
        elif distance < 75 and (touchSensor.pressed() or touchSensor2.pressed()):
            priority = 9
            control_signal = "turn left"
        else: # turns right if bump and no distance
            priority = 9
            control_signal = "turn right"
        return priority, control_signal
        

    def goal_detection(self):
        # HIGH = 10
        # LOW = 0
        control_signal = ""
        priority = 0

        # check color sensor
        # high priority if green, otherwise 0
        if ColorSensor(self.color_port).color() == Color.GREEN and not self.orient_to_candle:
            priority = 10

        # search for candle 
        # move forward 1/4m once green has been detected
        if priority > 0:
            control_signal = "orient to candle"
        else:
            control_signal = "do nothing"

        return priority, control_signal


    def extinguish(self):
        control_signal = None
        priority = 0

        # check color sensor
        # high priority if green tile reached and already moved to candle
        if ColorSensor(self.color_port).color() == Color.GREEN and self.orient_to_candle:
            priority = 10

        if priority == 10:
            control_signal = "extinguish fire"
        else:
            control_signal = "do nothing"

        return priority, control_signal


    """
    SIGNAL TRANSLATION
        Translates control signal into action. Call the control function you want
        to use here.
    """
    def execute_signal(self, control_signal, ev3):
        print("EXECUTING: " + control_signal)

        if control_signal == "move forward":
            self.forward_one()

        elif control_signal == "slight left":
            rightMotor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
            rightMotor.run_time(375, 1900, then=Stop.HOLD, wait=False)
            leftMotor = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
            leftMotor.run_time(365, 1900, then=Stop.HOLD, wait=True)

        elif control_signal == "slight right":
            leftMotor = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
            leftMotor.run_time(375, 1900, then=Stop.HOLD, wait=False)
            rightMotor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
            rightMotor.run_time(365, 1900, then=Stop.HOLD, wait=True)

        elif control_signal == "turn right":
            self.turn_right()

        elif control_signal == "turn left":
            self.turn_left()

        elif control_signal == "orient to candle":
            self.face_candle()

        elif control_signal == "extinguish fire":
            self.run_fan()
            self.play_celebration(ev3)

        elif control_signal == "do nothing":
            print("Error. Do nothing action givin highest priority.")
            self.play_failure_warning(ev3)

        else:
            print("Invalid Signal")
            exit(1)


    """
    CONTROL FUNCTIONS
        the actions the robot can take. implement the physical manipulation of motors here.

    i.e. turn_right_x(), forward(), etc. 
    """

    # forward function from project 1
    def forward_one(self):
        radius_from_center = .08565
        rotational_vel_rad = 3.491
        wheel_radius = .028
        rotational_vel_deg = rotational_vel_rad * (180/math.pi)

        t = (radius_from_center / (2*rotational_vel_rad * wheel_radius)) * (math.pi/2)
        ms = t * 1000

        rightMotor = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears=None)
        rightMotor.run_time(375, 1900, then=Stop.HOLD, wait=False)

        leftMotor = Motor(Port.B, positive_direction=Direction.CLOCKWISE, gears=None)
        leftMotor.run_time(375, 1915, then=Stop.HOLD, wait=True)

    # turn right 90 degrees function from project 1
    def turn_right(self):
        # define the motors
        leftMotor = Motor(self.motorL_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
        rightMotor = Motor(self.motorR_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)

        # reverse first because every time a turn is activated it will come after a collision
        leftMotor.run_time(300, 900-400, then=Stop.HOLD, wait=False)
        rightMotor.run_time(300, 900-400, then=Stop.HOLD, wait=True)

        robot_radius = 0.08565 # radius of robot in (m), defined with caliper
        arc_len = (1/4) * 2 * math.pi * robot_radius

        wheel_radius = 0.033 # defined in in (m) using calipers
        move_angle = arc_len / wheel_radius

        # convert angle to degrees
        move_angle_deg = move_angle * (180/math.pi)

        # turn
        leftMotor = Motor(self.motorL_port, positive_direction=Direction.CLOCKWISE, gears=None)
        rightMotor = Motor(self.motorR_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)

        leftMotor.run_angle(speed=90, rotation_angle=move_angle_deg-10, then=Stop.HOLD, wait=False)
        rightMotor.run_angle(speed=90, rotation_angle=move_angle_deg-10, then=Stop.HOLD, wait=True)

    # turn left 90 degrees function from project 1
    def turn_left(self):

        leftMotor = Motor(self.motorL_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
        rightMotor = Motor(self.motorR_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)

        # reverse first because every time a turn is activated it will come after a collision
        leftMotor.run_time(300, 900-400, then=Stop.HOLD, wait=False)
        rightMotor.run_time(300, 900-400, then=Stop.HOLD, wait=True)

        robot_radius = 0.08565 # radius of robot in (m), defined with caliper
        arc_len = (1/4) * 2 * math.pi * robot_radius

        # find angle wheel needs to move through
        # move_angle * wheel_radius = arc_len
        wheel_radius = 0.033 # defined in in (m) using calipers
        move_angle = arc_len / wheel_radius

        # convert angle to degrees
        move_angle_deg = move_angle * (180/math.pi)

        # turn
        leftMotor = Motor(self.motorL_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
        rightMotor = Motor(self.motorR_port, positive_direction=Direction.CLOCKWISE, gears=None)

        leftMotor.run_angle(speed=90, rotation_angle=move_angle_deg-10, then=Stop.HOLD, wait=False)
        rightMotor.run_angle(speed=90, rotation_angle=move_angle_deg-10, then=Stop.HOLD, wait=True)

    # move deeper into goal area
    def forward_to_candle(self):
        forward_angle = 30
        rightMotor = Motor(self.motorR_port, positive_direction=Direction.CLOCKWISE, gears=None)
        leftMotor = Motor(self.motorL_port, positive_direction=Direction.CLOCKWISE, gears=None)
        rightMotor.run_angle(speed=90, rotation_angle=forward_angle, then=Stop.HOLD, wait=False)
        leftMotor.run_angle(speed=90, rotation_angle=forward_angle, then=Stop.HOLD, wait=True)

    # make small turns to help define goal area
    def orient_turn(self, angle, direction):
        if direction == "L":
            rightMotor = Motor(self.motorR_port, positive_direction=Direction.CLOCKWISE, gears=None)
            leftMotor = Motor(self.motorL_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
            rightMotor.run_angle(speed=90, rotation_angle=angle, then=Stop.HOLD, wait=False)
            leftMotor.run_angle(speed=90, rotation_angle=angle, then=Stop.HOLD, wait=True)

        elif direction == "R":
            rightMotor = Motor(self.motorR_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
            leftMotor = Motor(self.motorL_port, positive_direction=Direction.CLOCKWISE, gears=None)
            rightMotor.run_angle(speed=90, rotation_angle=angle, then=Stop.HOLD, wait=False)
            leftMotor.run_angle(speed=90, rotation_angle=angle, then=Stop.HOLD, wait=True)

        else:
            print("Error, no direction specified")

    # orient towards the candle
    def face_candle(self):
        # turn left 
        rotation_angle = 30
        self.orient_turn(rotation_angle, "L")

        # let robot settle
        time.sleep(1)

        # check sensor
        # if still green, turn back and extinguish
        if ColorSensor(self.color_port).color() == Color.GREEN:
            self.orient_turn(rotation_angle, "R")
            self.orient_to_candle = True

        # if not green, turn back, move forward, extinguish
        else:
            self.orient_turn(rotation_angle, "R")
            self.forward_to_candle()
            self.orient_to_candle = True

    # move forward 1/4m (1/4 square tile) to extinguish the candle
    def face_candle_old(self):

        turn_limit = 10
        rotation_angle = 20
        count = 0

        # find left edge of goal area
        # move left until green color is gone, mark angle
        rightMotor = Motor(self.motorR_port, positive_direction=Direction.CLOCKWISE, gears=None)
        leftMotor = Motor(self.motorL_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
        while ColorSensor(self.color_port).color() == Color.GREEN and count < turn_limit:
            print("finding left edge")
            rightMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=False)
            leftMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=True)

            count = count + 1

        # if edge not found, 1/2 original pos probably close to center
        if count == turn_limit:
            print("left edge not found")
            rightMotor = Motor(self.motorR_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
            leftMotor = Motor(self.motorL_port, positive_direction=Direction.CLOCKWISE, gears=None)
            while count//2 > 0:
                rightMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=False)
                leftMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=True)

                count = count - 1

        else:
            # reset to green
            count = 0
            rightMotor = Motor(self.motorR_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
            leftMotor = Motor(self.motorL_port, positive_direction=Direction.CLOCKWISE, gears=None)
            while ColorSensor(self.color_port).color() != Color.GREEN and count < turn_limit:
                print("left edge found. finding green area again")
                rightMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=False)
                leftMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=True)

                count = count + 1

            # if reset limit is reached, then mistake was made
            # do nothing and let another behavior take over
            if count == turn_limit:
                print("reset limit reached")
                return

            # check distance to right edge
            count = 0
            while ColorSensor(self.color_port).color() == Color.GREEN and count < turn_limit:
                print("finding right edge ")
                rightMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=False)
                leftMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=True)

                count = count + 1

            # move to center of right turn arc
            # this is the probably center of goal square
            rightMotor = Motor(self.motorR_port, positive_direction=Direction.CLOCKWISE, gears=None)
            leftMotor = Motor(self.motorL_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
            while count//2 > 0:
                print("centering")
                rightMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=False)
                leftMotor.run_angle(speed=90, rotation_angle=rotation_angle, then=Stop.HOLD, wait=True)

                count = count - 1
            
        # prepare to extinguish fire
        self.orient_to_candle = True

    # blow out candle with fan
    def run_fan(self):
        fan_speed = 360 # deg/s
        fan_motor = Motor(self.fan_port, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)

        # spin fan for 5s
        fan_motor.run(fan_speed)
        time.sleep(5)
        fan_motor.stop()

    def play_failure_warning(self, ev3):
        ev3.speaker.play_file(SoundFile.BOO)

    def play_celebration(self, ev3):
        ev3.speaker.play_file(SoundFile.CHEERING)

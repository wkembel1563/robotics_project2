#!/usr/bin/env pybricks-micropython
from bbot import *

# Create your objects here.
ev3 = EV3Brick()
bot = BehavioralBot()

# run bot until goal is reached
goal_reached = False
while not goal_reached:
    """
    Behaviors
    - wander
    - wall following
    - goal detection
    - extinguish
    """
    # get behavior actions and their priorities
    wp, w_control = bot.wander()
    wfp, wf_control = bot.wall_following()
    #gdp, gd_control = bot.goal_detection()
    #exp, ex_control = bot.extinguish()
    # wfp, wf_control = 0, ""
    gdp, gd_control = 0, ""
    exp, ex_control = 0, ""

    # decide on a control signal to use
    # wander
    if wp > wfp and wp > gdp and wp > exp:
        print("Action chosen: wander")
        control_signal = w_control

    # wall following
    elif wfp > gdp and wfp > exp:
        print("Action chosen: wall following")
        control_signal = wf_control

    # goal detection
    elif gdp > exp:
        print("Action chosen: goal detection")
        control_signal = gd_control

    # extinguish
    else:
        print("Action chosen: extinguish")
        control_signal = ex_control
        goal_reached = True

    # take highest priority action
    bot.execute_signal(control_signal, ev3)
from .src import mastering_app
from machinekit import hal as hal
from machinekit import rtapi as rt

def main():
    try: 
        rt.init_RTAPI()
        sleep(0.1)
    except RuntimeError as e:
        pass
    
    if hal.Signal('mod_success').get() == True:
        l = mastering_app.Leveller(name="Masterer")
        l.init_attributes()
        l.fine_calibration = True
        l.max_vel = 0.8
        l.max_acc = 1.5
        notify("Please press \OK\ to start the mastering program.", warning=True)
        l.start()
        notify("THIS IS NOT AN ERROR. The mastering of the robot has finished. The robot has moved to the pose that should hold all zeros as joint angles. Press ABORT to exit this program, power off the robot, restart, and press ZERO ALL JOINTS from the SETTINGS tab before powering on again.", error=True)
    else:
        notify("It looks like this configuration is not suitable for mastering the robot. Please look for details in the applicable instructions.", error=True)

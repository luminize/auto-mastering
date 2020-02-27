from .src import mastering_app
from .src import change_configuration
from machinekit import hal as hal
from machinekit import rtapi as rt

def main():
    movel("home", v=0.08, a=0.08)
    try: 
        rt.init_RTAPI()
        notify("HAL initialized")
        sleep(1.0)
    except RuntimeError as e:
        print(e.message)
        #if e.message == "signal 'mod_success' does not exist"
        #pass
    
    try:
        hal.Signal("mod_success")
    except RuntimeError as e:
        if e.message == "signal 'mod_success' does not exist":
            change_configuration.change_config()
            notify("HAL configuration changed")
            sleep(1.0)
    if hal.Signal('mod_success').get() == True:
        l = mastering_app.Leveller(name="Masterer")
        l.init_attributes()
        l.fine_calibration = True
        l.calib_fast_iter = 2
        l.calib_slow_iter = 4
        l.max_vel = 0.4
        l.max_acc = 0.8
        notify("Please press OK to start the mastering program.", warning=True)
        l.info_callback = notify
        l.start()
        notify("THIS IS NOT AN ERROR. The mastering of the robot has finished. The robot has moved to the pose that should hold all zeros as joint angles. Press ABORT to exit this program, power off the robot, restart, and press ZERO ALL JOINTS from the SETTINGS tab before powering on again.", error=True)
    else:
        notify("It looks like this configuration is not suitable for mastering the robot. Please look for details in the applicable instructions.", error=True)

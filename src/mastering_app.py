#!/usr/bin/env python
import sys
import attr
import time
from machinekit import hal as hal
from machinekit import rtapi as rt
from machinetalk.protobuf.sample_pb2 import Sample
from fysom import Fysom, FysomError
from math import pi
from pandas import pandas as pd
from StringIO import StringIO


@attr.s
class Recorder(object):
    hal = attr.ib()
    name = attr.ib(default="")
    ring = attr.ib(default="")
    sampler = attr.ib(default="")
    max_samples = attr.ib(default=20)
    homing_offset = attr.ib(default=0.5)
    # to be reset each time a new recording is made
    samples = attr.ib(default='record,series,flt_timestamp,sensor,direction,rotation\n')
    ring = attr.ib(default="")
    sampler = attr.ib(default="")
    recorder_active = attr.ib(default=False)
    sample = attr.ib(default=0)
    fine_calibration = attr.ib(default=False)

    def reset(self):
        self.samples = 'record,series,flt_timestamp,sensor,direction,rotation\n'
        self.ring = ""
        self.sampler = ""
        self.recorder_active = False
        self.sample = 0

    def start(self):
        self.prev_sensor_val = 0
        self.curr_sensor_val = 0
        self.prev_position = 0
        self.curr_position = 0
        self.prev_position_ts = 0
        self.curr_position_ts = 0
        # fo initializing curr and prev sensor val on the first time
        self.new_recording = True
        self.sensor = 0
        self.record_next_rotation = False
        #filename="data_20200113_1.csv"
        self.series = -1
        self.record = 0
        self.recorder_active = False
        try:
            # try to attach to ring
            self.r = self.hal.Ring(self.ring)
            # flush the ring first
            self.hal.Pin("{}.{}".format(self.sampler, "record")).set(0)
            time.sleep(0.01)
            for i in self.r:
                self.r.shift()
            self.hal.Pin("{}.{}".format(self.sampler, "record")).set(1)
            self.recorder_active = True
        except NameError as e:
            print(e)

    def stop(self):
        self.hal.Pin("{}.{}".format(self.sampler, "record")).set(0)

    def process_samples(self):
        if (self.ring == "") or (self.sampler == "") or (self.recorder_active == False):
            print("No sampler or ring given, or recorder not active")
        else:
            # inspect the ring:
            for i in self.r:
                self.sample += 1
                b = i.tobytes()
                s = Sample()
                s.ParseFromString(b)
                if (s.HasField("v_bool") == True):
                    ts = s.timestamp
                    v = s.v_bool
                    t = "bit"
                    # to not detect an edge the first time
                    # initialize prev_sensor_val correctly
                    if self.new_recording == True:
                        self.prev_sensor_val = v
                        self.curr_sensor_val = v
                        self.new_recording = False
                    self.prev_sensor_val = self.curr_sensor_val
                    self.curr_sensor_val = v
                    self.curr_timestamp = ts
                if (s.HasField("v_double") == True):
                    ts = s.timestamp
                    v = s.v_double
                    t = "flt"
                    self.prev_position = self.curr_position
                    self.prev_position_ts = self.curr_position_ts
                    self.curr_position = v
                    self.curr_position_ts = ts
                if (s.HasField("v_uint32") == True):
                    ts = s.timestamp
                    v = s.v_uint32
                    t = "u32"
                    self.series = v
                if (s.HasField("v_int32") == True):
                    v = s.v_int32
                    t = "s32"
                # the rotation value _after_ a sensor bit change
                if (self.record_next_rotation == True) and (t == 'flt'):
                    self.record += 1
                    if self.prev_position < self.curr_position:
                        self.direction = "up"
                    else:
                        self.direction = "down"
                    print("{},{},{},{},{},{}".format(
                        self.record,
                        self.series,
                        self.curr_timestamp,
                        self.sensor,
                        self.direction,
                        self.curr_position))
                    self.samples += "{},{},{},{},{},{}\n".format(
                        self.record,
                        self.series,
                        self.curr_timestamp,
                        self.sensor,
                        self.direction,
                        self.curr_position)
                    # reset the record_next_rotation bool
                    self.record_next_rotation = False
                    # extra: do not record a second time
                    self.prev_sensor_val = self.curr_sensor_val 
                # detect change in sensor bit, record _next_ rotation value
                if self.prev_sensor_val != self.curr_sensor_val:
                    self.sensor = self.curr_sensor_val
                    self.record_next_rotation = True
                self.r.shift()

    def get_samples(self):
        return self.samples    

    def set_sampler(self, sampler_name="", ring_name=""):
        if (sampler_name == "") or (ring_name == ""):
            print("No sampler or ring given")
        else:
            self.reset()
            self.sampler = sampler_name
            self.ring = ring_name


@attr.s
class Joint(object):
    name = attr.ib(default="")
    jp_name = attr.ib(default="")
    sampler_name = attr.ib(default="")
    ring_name = attr.ib(default="")
    max_vel = attr.ib(default=0.1)
    max_acc = attr.ib(default=0.05)
    min_vel = attr.ib(default=0.01)
    min_acc = attr.ib(default=0.01)
    curr_pos = attr.ib(default=0.)
    pos_cmd = attr.ib(default=0.)
    offset = attr.ib(default=0.)
    calibrated = attr.ib(default=False)
    data_raw = attr.ib(default="")
    calibration = attr.ib(default="callback")

    def jp_pin(self, pinname=""):
        return "{}.{}".format(self.jp_name, pinname)

    def sampler_pin(self, pinname=""):
        return "{}.{}".format(self.sampler_name, pinname)

    def calculate_offset(self, sample_values="", nominal=0):
        if (sample_values!=""):
            d = pd.read_csv(StringIO(sample_values))
            self.raw_data = d
            d_u = d.loc[d['direction']=='up']
            d_d = d.loc[d['direction']=='down']
            mean_u = d_u.rotation.mean()
            mean_d = d_d.rotation.mean()
            self.offset = ((mean_d + mean_u) / 2) - nominal
            self.calibrated = True


@attr.s
class Leveller(object):
    name = attr.ib()
    fsm = attr.ib(Fysom())
    poses = attr.ib({
            1: [.0, .0, .0, .0, .0, .0],
            2: [.0, .0, -pi/2, .0, pi/2, .0],
            3: [.0, .0, -pi/2, -pi/2, pi/2, .0],
            4: [.0, .0, -pi/4, -pi/2, .0, (pi/2 - pi/6)],
            5: [.0, .0, -pi/4, .0, .0, (-pi/6)],
            6: [.0, pi/4, .0,  .0, -pi/2, (-pi/6)],
            7: [-pi, -pi/4, .0,  .0, -pi/2, (-pi/6)],
            8: [-pi, -pi/4, -pi,  pi, -pi/2, (-pi/6)],
            9: [.0, pi/4, .0,  .0, -pi/4, -pi],
    })
    joints = attr.ib({})
    calibration_moves = attr.ib({
            1: [1],
            # search tilt at 1 and record joint angle
            # move to 7 and then 8
            # search tilt at 8 and record joint angle
            # calculate offset of difference
            2: [6, 7, 8],
            3: [3],
            4: [1],
            # search tilt at 4 and record joint angle
            # move to 5
            # search tilt at 5 and record joint angle
            # 5 is parallel with 3
            5: [4, 5],
            6: [2],
    })
    # sequence determines the order in which joints get calibrated
    # the calibration moves define the poses in chich this is done
    sequence = attr.ib([6, 3, 4, 5, 2])
    # current joint to be calibrated
    curr_joint = attr.ib(default=0)
    # with its current calibration moves
    moves = attr.ib(default=[])
    # remember the current move 
    moves_index=attr.ib(default=0)
    fine_calibration=attr.ib(default=False)
    rt = attr.ib(rt.init_RTAPI())
    hal = attr.ib(hal)
    max_vel = attr.ib(default=0.1)
    max_acc = attr.ib(default=0.1)
    calib_fast_iter = attr.ib(default=3)
    calib_fast_vel = attr.ib(default=0.1)
    calib_fast_acc = attr.ib(default=1.6)
    calib_fast_amp = attr.ib(default=0.4)
    calib_slow_iter = attr.ib(default=10)
    calib_slow_vel = attr.ib(default=0.005)
    calib_slow_acc = attr.ib(default=0.1)
    calib_slow_amp = attr.ib(default=0.1)
    speed_factor = attr.ib(default=1.)
    fully_automatic_levelling = attr.ib(default=True)

    def start(self):
        self.fsm.t_activate_joint()

    def move_to_pose(self, nr=0):
        if nr in self.poses:
            longest_distance = self.find_longest_distance(nr)
            print("longest distance = %s rad" % longest_distance)
            self.calc_joint_move(longest_distance)
            for i in range(1,7):
                j = self.joints[i]
                print("{} -> {}".format(j.jp_pin("max-vel"), j.max_vel))
                print("{} -> {}".format(j.jp_pin("max-acc"), j.max_acc))
                print("{} -> {}".format(j.jp_pin("pos-cmd"), j.pos_cmd))
                self.hal.Pin(j.jp_pin("max-vel")).set(j.max_vel)
                self.hal.Pin(j.jp_pin("max-acc")).set(j.max_acc)
                self.hal.Pin(j.jp_pin("pos-cmd")).set(j.pos_cmd)

    def find_longest_distance(self, nr=0):
        abs_dist = 0
        longest_distance = 0
        joint_nr = 0
        for i in range(1,7):
            curr_pos = self.hal.Signal('joint%s_ros_pos_fb' % i).get()
            j = self.joints[i]
            j.curr_pos = curr_pos
            j.pos_cmd = self.poses[nr][i-1] + j.offset
            abs_dist = abs(j.pos_cmd - j.curr_pos)
            if abs_dist > longest_distance:
                longest_distance = abs_dist
        return longest_distance

    def calc_joint_move(self, l_dist=0):
        if l_dist > 0:
            for i in range(1,7):
                j = self.joints[i]
                dist = abs(j.pos_cmd - j.curr_pos)
                print("distance joint {} is {} rad".format(i, dist))
                j.max_acc = self.max_acc * (dist / l_dist) * self.speed_factor
                j.max_vel = self.max_vel * (dist / l_dist) * self.speed_factor
                # prevent very slow moves nearing zero velocity
                if j.max_acc < j.min_acc: j.max_acc = j.min_acc
                if j.max_vel < j.min_vel: j.max_vel = j.min_vel

    def initialize(self):
        for i in range(1,7):
            j = self.joints[i]
            j.jp_name = "jp{}.0".format(i)
            j.sampler_name = "sampler{}".format(i)
            j.ring_name = "sampler{}.ring".format(i)
        self.recorder = Recorder(name="sample_recorder", hal=self.hal)

    def disable_all_jplan(self):
        for i in range(1,7):
            j = self.joints[i-1]
            self.hal.Pin(j.pin('enable')).set(0)

    def enable_all_jplan(self):
        for i in range(1,7):
            j = self.joints[i-1]
            self.hal.Pin(j.pin('enable')).set(1)

    def oscillate_joint(self, nr=0, nominal=0., amplitude=0.5, nr_measurements=5, speed=.1, accel=3.0, offset=.0):
        if nr in self.joints:
            direction = 1
            center = nominal + offset
            target = center + (amplitude * direction)
            # let's set delta as 5% of the amplitude
            delta = amplitude * 0.05
            measurement = 0
            j = self.joints[nr]
            self.hal.Pin(j.jp_pin("max-vel")).set(speed)
            self.hal.Pin(j.jp_pin("max-acc")).set(accel)
            self.hal.Pin(j.jp_pin("pos-cmd")).set(target)
            # set correct settings of the ring of the joint
            self.recorder.reset()
            self.recorder.set_sampler(sampler_name=j.sampler_name,
                                      ring_name=j.ring_name)
            self.wait_on_finish_moving()
            self.recorder.start()
            while (measurement < nr_measurements):
                # process the records in the ring
                self.recorder.process_samples()
                # get the current position
                curr_pos = self.hal.Signal('joint%s_ros_pos_fb' % (nr)).get()
                if direction > 0:
                    if curr_pos > (target - delta):
                        direction *= -1
                        target = center + (amplitude * direction)
                        measurement += 1
                        print("Measurement: %s" % measurement)
                        self.hal.Pin(j.jp_pin("pos-cmd")).set(target)
                if direction < 0:
                    if curr_pos < (target + delta):
                        direction *= -1
                        target = center + (amplitude * direction)
                        self.hal.Pin(j.jp_pin("pos-cmd")).set(target)
                time.sleep(0.1)
            self.recorder.stop()
            self.hal.Pin(j.jp_pin("pos-cmd")).set(center)
            j.calculate_offset(sample_values=self.recorder.get_samples(), nominal=nominal)

    def calibrate_1(self):
        j = self.joints[self.curr_joint]
        print("calibration of %s" % j.name)

    def calibrate_2(self):
        j = self.joints[self.curr_joint]
        print("calibration of %s" % j.name)
        # roughly seek around the joint 2 nominal angle
        nominal_pos = self.poses[self.moves[self.moves_index]][self.curr_joint -1]
        self.oscillate_joint(nr=self.curr_joint,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough 1st angle of joint 2 at signal is %s' % (nominal_pos + j.offset))
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        if self.fine_calibration == True:
            self.oscillate_joint(nr=self.curr_joint,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos,
                             offset=j.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
            print('accurate 1st angle of joint 2 at signal is %s' % (nominal_pos + j.offset))
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        self.wait_on_finish_moving()
        # first part is done, now save this offset value and change poses
        # to the 180 degree twisted setup and repeat
        offset_1 = nominal_pos + j.offset
        j.offset = 0
        self.moves_index += 1
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        self.moves_index += 1
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        # take new measurments for the 2nd offset
        nominal_pos = self.poses[self.moves[self.moves_index]][self.curr_joint -1]
        self.oscillate_joint(nr=self.curr_joint,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough 2nd angle of joint 2 at signal is %s' % (nominal_pos + j.offset))
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        if self.fine_calibration == True:
            self.oscillate_joint(nr=self.curr_joint,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos,
                             offset=j.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
            print('accurate 2nd angle of joint 2 at signal is %s' % (nominal_pos + j.offset))
        # move to the calculated offset value
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        self.wait_on_finish_moving()
        offset_2 = nominal_pos + j.offset
        # the average offset is the real offset of joint 2
        j.offset = (offset_1 + offset_2) / 2
        print('offset of joint 2 is %s' % j.offset)

    def calibrate_3(self):
        j = self.joints[self.curr_joint]
        print("calibration of %s" % j.name)
        # first rough calculation,
        nominal_pos = self.poses[self.moves[self.moves_index]][self.curr_joint - 1]
        self.oscillate_joint(nr=self.curr_joint,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough offset of joint 3 is %s' % j.offset)
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        if self.fine_calibration == True:
            self.oscillate_joint(nr=self.curr_joint,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos,
                             offset=j.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
        # move to the calculated offset value
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        self.wait_on_finish_moving()
        print('accurate offset of joint 3 is %s' % j.offset)
 
    def calibrate_4(self):
        j = self.joints[self.curr_joint]
        print("calibration of %s" % j.name)
        # first rough calculation,
        nominal_pos = self.poses[self.moves[self.moves_index]][self.curr_joint - 1]
        self.oscillate_joint(nr=self.curr_joint,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough offset of joint 4 is %s' % j.offset)
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        if self.fine_calibration == True:
            self.oscillate_joint(nr=self.curr_joint,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos,
                             offset=j.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
        # move to the calculated offset value
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        print('accurate offset of joint 4 is %s' % (nominal_pos + j.offset))
        self.wait_on_finish_moving()

    def calibrate_5(self):
        j3 = self.joints[3]
        j = self.joints[self.curr_joint]
        print("calibration of %s" % j.name)
        # first remember old home value and find angle for joint 3
        joint_3_home = j3.offset
        # roughly seek around the joint 3 nominal angle
        nominal_pos = self.poses[self.moves[self.moves_index]][2]
        self.oscillate_joint(nr=3,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough angle of joint 3 at signal is %s' % (nominal_pos + j3.offset))
        self.hal.Pin(j3.jp_pin("pos-cmd")).set(nominal_pos + j3.offset)
        self.wait_on_finish_moving()
        if self.fine_calibration == True:
            self.oscillate_joint(nr=3,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos ,
                             offset=j3.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
            print('accurate angle of joint 3 at signal is %s' % (nominal_pos + j3.offset))
        # the angle of jont 3 at which the IMU signals is now known,
        # now rotate joint 4 and joint 6 and find the offset of joint 5
        # since joint 5 will be in line with joint 3 and 4, that offset
        # is the correct deviation from "zero"
        self.moves_index += 1
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()

        nominal_pos = self.poses[self.moves[self.moves_index]][self.curr_joint - 1]
        self.oscillate_joint(nr=self.curr_joint,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough offset of joint 5 is %s' % (nominal_pos + j.offset))
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        self.wait_on_finish_moving()
        if self.fine_calibration == True:
            self.oscillate_joint(nr=self.curr_joint,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos,
                             offset=j.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
        # move to the calculated offset value
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        print('accurate offset of joint 5 is %s' % j.offset)
        self.wait_on_finish_moving()
        # put old home value of joint 3 back again
        self.joints[3].offset = joint_3_home

    def calibrate_6(self):
        j = self.joints[self.curr_joint]
        print("calibration of %s" % j.name)
        # first rough calculation, 
        nominal_pos = self.poses[self.moves[self.moves_index]][self.curr_joint - 1]
        self.oscillate_joint(nr=self.curr_joint,
                            nr_measurements=self.calib_fast_iter,
                            nominal=nominal_pos,
                            accel=self.calib_fast_acc,
                            amplitude=self.calib_fast_amp,
                            speed=self.calib_fast_vel)
        print('rough offset of joint 6 is %s' % j.offset)
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        # then do accurate calculation
        if self.fine_calibration == True:
            self.oscillate_joint(nr=self.curr_joint,
                             nr_measurements=self.calib_slow_iter,
                             nominal=nominal_pos,
                             offset=j.offset,
                             accel=self.calib_slow_acc,
                             amplitude=self.calib_slow_amp,
                             speed=self.calib_slow_vel)
        # move to the calculated offset value
        self.hal.Pin(j.jp_pin("pos-cmd")).set(nominal_pos + j.offset)
        print('accurate offset of joint 6 is %s' % j.offset)
        self.wait_on_finish_moving()

    def wait_on_finish_moving(self):
        timeout = 2000
        t = 0
        prev_pos_arr = [.0, .0, .0, .0, .0, .0]
        curr_pos_arr = [self.hal.Pin('jp1.0.curr-pos').get(),
                           self.hal.Pin('jp2.0.curr-pos').get(),
                           self.hal.Pin('jp3.0.curr-pos').get(),
                           self.hal.Pin('jp4.0.curr-pos').get(),
                           self.hal.Pin('jp5.0.curr-pos').get(),
                           self.hal.Pin('jp6.0.curr-pos').get()]
        motion = (prev_pos_arr != curr_pos_arr)
        #while((self.hal.Pin('jplanners_active.out').get() == True) and (t < timeout)):
        while (motion and (t < timeout)):
            time.sleep (0.05)
            t += 1
            prev_pos_arr = curr_pos_arr
            curr_pos_arr = [self.hal.Pin('jp1.0.curr-pos').get(),
                    self.hal.Pin('jp2.0.curr-pos').get(),
                    self.hal.Pin('jp3.0.curr-pos').get(),
                    self.hal.Pin('jp4.0.curr-pos').get(),
                    self.hal.Pin('jp5.0.curr-pos').get(),
                    self.hal.Pin('jp6.0.curr-pos').get()]

            #print prev_pos_arr
            #print curr_pos_arr
            motion = (prev_pos_arr != curr_pos_arr)
            #print motion
        if (t >= timeout):
             print("timeout")

    def oninitial(self, e):
        self.initialize()
        self.fsm.t_setup()

    def onsetup(self, e):
        # set indexes to pick the first joint number in sequence
        self.sequence_index = 0
        # set index to first list entry
        self.moves_index = 0
        self.fsm.t_idle()

    def onactivate_joint(self, e):
        # set the first joint to calibrate
        self.curr_joint = self.sequence[self.sequence_index]
        # select the moves list to use
        self.moves = self.calibration_moves[self.curr_joint]
        self.fsm.t_move_to_first_pose()

    def onmove_to_first_pose(self, e):
        self.move_to_pose(self.moves[self.moves_index])
        self.wait_on_finish_moving()
        self.fsm.t_calibrate_joint()

    def oncalibrate_joint(self,e):
        self.joints[self.curr_joint].calibration()
        self.fsm.t_joint_calibrated()
    
    def onjoint_calibrated(self,e):
        self.joints[self.curr_joint].calibrated = True
        self.fsm.t_set_next_joint()
    
    def onset_next_joint(self,e):
        # set index to first list entry
        self.moves_index = 0
        if self.sequence_index < (len(self.sequence) - 1):
            # set indexes to pick the first joint number in sequence
            self.sequence_index += 1
            if self.fully_automatic_levelling == True:
                self.fsm.t_activate_joint()
        else:
            # just start over with an entire new set if we want.
            self.sequence_index = 0
            self.fsm.t_move_zero()

    def onmove_to_zero(self,e):
        self.move_to_pose(7)
        self.wait_on_finish_moving()
        self.move_to_pose(1)
        self.wait_on_finish_moving()

    def sync_jp_with_current_pos(self, i=0):
        if (i != 0):
            j = self.joints[i]
            curr_pos = self.hal.Signal('joint%s_ros_pos_fb' % i).get()
            self.hal.Pin(j.jp_pin("pos-cmd")).set(curr_pos)
            self.wait_on_finish_moving()
            print("joint %s HAL synced" % i)

    def init_attributes(self):
        self.joints={
            1: Joint(name='joint_1', calibration=self.calibrate_1),
            2: Joint(name='joint_2', calibration=self.calibrate_2),
            3: Joint(name='joint_3', calibration=self.calibrate_3),
            4: Joint(name='joint_4', calibration=self.calibrate_4),
            5: Joint(name='joint_5', calibration=self.calibrate_5),
            6: Joint(name='joint_6', calibration=self.calibrate_6),
            }
        self.fsm = Fysom(
            {
            'initial': {'state': 'initial', 'event': 'init', 'defer': True},
            'events': [
                {
                    'name': 't_setup',
                    'src': ['initial', 'fault'],
                    'dst': 'setup',
                    },
                {'name': 't_idle', 'src': 'setup', 'dst': 'idle'},
                {
                    'name': 't_activate_joint',
                    'src': ['idle', 'set_next_joint', 'joint_calibrated'],
                    'dst': 'activate_joint'},
                {
                    'name': 't_move_to_first_pose',
                    'src': ['activate_joint'],
                    'dst': 'move_to_first_pose'
                    },
                {'name': 't_calibrate_joint', 'src': 'move_to_first_pose', 'dst': 'calibrate_joint'},
                {'name': 't_joint_calibrated', 'src': 'calibrate_joint', 'dst': 'joint_calibrated'},
                {'name': 't_set_next_joint', 'src': 'joint_calibrated', 'dst': 'set_next_joint'},
                {'name': 't_move_zero', 'src': 'set_next_joint', 'dst': 'move_to_zero'},
                {'name': 't_end', 'src': 'move_to_zero', 'dst': 'finished'},
                {
                    'name': 't_error',
                    'src': [
                        'initial',
                        'setup',
                        'idle',
                        'activate_joint',
                        'move_to_first_pose',
                        'start_next_move',
                        'calibrate_joint',
                        'joint_calibrated',
                        'set_next_joint',
                        'move_to_zero'
                    ],
                    'dst': 'fault',
                    },
                {
                    'name': 't_abort',
                    'src': [
                        'initial',
                        'setup',
                        'idle',
                        'activate_joint',
                        'move_to_first_pose',
                        'calibrate_joint',
                        'joint_calibrated',
                        'set_next_joint',
                        'move_to_zero'
                    ],
                    'dst': 'abort',
                    },
                ],
                'callbacks': {
                    'oninitial': self.oninitial,
                    'onsetup': self.onsetup,
                    'onactivate_joint': self.onactivate_joint,
                    'onmove_to_first_pose': self.onmove_to_first_pose,
                    'oncalibrate_joint' : self.oncalibrate_joint,
                    'onjoint_calibrated': self.onjoint_calibrated,
                    'onset_next_joint': self.onset_next_joint,
                    'onmove_to_zero': self.onmove_to_zero}
            })
        self.fsm.init()


if __name__ == "__main__":
    l = Leveller(name="Masterer")
    l.init_attributes()
    try:
        import manhole
        manhole.install(locals={
            'l': l,
        })
    except Exception:
        print("Manhole not installed - run 'sudo pip2 install manhole'")
    
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting app")
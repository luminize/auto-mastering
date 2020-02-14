#!/usr/bin/env python

import attr
import time
from machinekit import hal as hal
from machinekit import rtapi as rt
from machinetalk.protobuf.sample_pb2 import Sample
from fysom import Fysom, FysomError
from math import pi

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

    def set_sampler(self, sampler_inst="", ring_name=""):
        if (sampler_inst == "") or (ring_name == ""):
            print("No sampler or ring given")
        else:
            self.reset()
            self.sampler = sampler_inst
            self.ring = ring_name

@attr.s
class Joint(object):
    name = attr.ib(default="")
    jp_name = attr.ib(default="")
    max_vel = attr.ib(default=0.1)
    max_acc = attr.ib(default=0.1)
    curr_pos = attr.ib(default=0.)
    pos_cmd = attr.ib(default=0.)
    offset = attr.ib(default=0.)
    calibrated = attr.ib(default=False)

    def jp_pin(self, pinname=""):
        return "{}.{}".format(self.jp_name, pinname)

@attr.s
class Leveller(object):
    name = attr.ib()
    fsm = attr.ib(Fysom(
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
                'name': 't_activate',
                'src': ['idle', 'save_offsets'],
                'dst': 'activate_joint'},
            {
                'name': 't_move_to_pose',
                'src': ['activate_joint', 'joint_calibrated'],
                'dst': 'move_to_pose'
                },
            {'name': 't_calibrate', 'src': 'move_to_pose', 'dst': 'calibrate_joint'},
            {'name': 't_calibrated', 'src': 'calibrate_joint', 'dst': 'joint_calibrated'},
            {'name': 't_save', 'src': 'joint_calibrated', 'dst': 'save_offsets'},
            {'name': 't_move_zero', 'src': 'save_offsets', 'dst': 'move_to_zero'},
            {'name': 't_end', 'src': 'move_to_zero', 'dst': 'finished'},
            {
                'name': 't_error',
                'src': [
                    'initial',
                    'setup',
                    'idle',
                    'activate_joint',
                    'move_to_pose',
                    'calibrate_joint',
                    'joint_calibrated',
                    'save_offsets',
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
                    'move_to_pose',
                    'calibrate_joint',
                    'joint_calibrated',
                    'save_offsets',
                    'move_to_zero'
                ],
                'dst': 'abort',
                },
            ],
        }
    ))
    poses = attr.ib({
            1: [.0, .0, .0, .0, .0, .0],
            2: [-pi/2, .0, -pi/2, .0, pi/2, pi/2],
            3: [-pi/2, .0, -pi/2, pi/2, pi/2, pi/2],
            4: [.0, .0, -pi/4, -pi/2, .0, (pi/2 - pi/6)],
            5: [.0, .0, -pi/4, .0, .0, (-pi/6)],
            6: [.0, pi/4, .0,  .0, -pi/2, (-pi/6)],
            7: [-pi, -pi/4, .0,  .0, -pi/2, (-pi/6)],
            8: [-pi, -pi/4, -pi,  pi, -pi/2, (-pi/6)],
    })
    joints = attr.ib({
            0: Joint(name='joint_1'),
            1: Joint(name='joint_2'),
            2: Joint(name='joint_3'),
            3: Joint(name='joint_4'),
            4: Joint(name='joint_5'),
            5: Joint(name='joint_6'),
    })
    rt = attr.ib(rt.init_RTAPI())
    hal = attr.ib(hal)
    max_vel = attr.ib(default=0.1)
    max_acc = attr.ib(default=0.1)
    speed_factor = attr.ib(default=1.)
    
    def move_to_pose(self, nr=0):
        if nr in self.poses:
            longest_distance = self.find_longest_distance(nr)
            print("longest distance = %s rad" % longest_distance)
            self.calc_joint_move(longest_distance)
            for i in range(0,6):
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
        joint_nr = -1
        for i in range(0,6):
            curr_pos = self.hal.Signal('joint%s_ros_pos_fb' % (i+1)).get()
            j = self.joints[i]
            j.curr_pos = curr_pos
            j.pos_cmd = self.poses[nr][i]
            abs_dist = abs(j.pos_cmd - j.curr_pos)
            if abs_dist > longest_distance:
                longest_distance = abs_dist
        return longest_distance

    def calc_joint_move(self, l_dist=0):
        if l_dist > 0:
            for i in range(0,6):
                j = self.joints[i]
                dist = abs(j.pos_cmd - j.curr_pos)
                print("distance joint {} is {} rad".format(1+i, dist))
                j.max_acc = self.max_acc * (dist / l_dist) * self.speed_factor
                j.max_vel = self.max_vel * (dist / l_dist) * self.speed_factor

    def initialize(self):
        for i in range(0,6):
            j = self.joints[i]
            j.jp_name = "jp{}.0".format(i+1)
        self.recorder = Recorder(name="sample_recorder", hal=self.hal)

    def disable_all_jplan(self):
        for i in range(0,6):
            j = self.joints[i]
            self.hal.Pin(j.pin('enable')).set(0)

    def enable_all_jplan(self):
        for i in range(0,6):
            j = self.joints[i]
            self.hal.Pin(j.pin('enable')).set(1)

    def calibrate_all(self):
        pass

    def calibrate_1(self):
        pass

    def calibrate_2(self):
        pass

    def calibrate_3(self):
        pass

    def calibrate_4(self):
        pass

    def calibrate_5(self):
        pass

    def calibrate_6(self):
        pass

def init_levelling():
    l = Leveller(name="Masterer")
    l.initialize()
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

if __name__ == "__main__":
    init_levelling()

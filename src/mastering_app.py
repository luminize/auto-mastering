#!/usr/bin/env python

import attr
import time
from machinekit import hal as hal
from machinekit import rtapi as rt
from fysom import Fysom, FysomError
from math import pi


@attr.s
class Joint(object):
    name = attr.ib(default="")
    jp_name = attr.ib(default="")
    max_vel = attr.ib(default=0.1)
    max_acc = attr.ib(default=0.1)
    curr_pos = attr.ib(default=0.)
    pos_cmd = attr.ib(default=0.)

    def pin(self, pinname=""):
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
                print("{} -> {}".format(j.pin("max-vel"), j.max_vel))
                print("{} -> {}".format(j.pin("max-acc"), j.max_acc))
                print("{} -> {}".format(j.pin("pos-cmd"), j.pos_cmd))
                self.hal.Pin(j.pin("max-vel")).set(j.max_vel)
                self.hal.Pin(j.pin("max-acc")).set(j.max_acc)
                self.hal.Pin(j.pin("pos-cmd")).set(j.pos_cmd)

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

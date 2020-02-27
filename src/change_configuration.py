#!/usr/bin/env python

import time
from machinekit import hal as hal
from machinekit import rtapi as rt

def insert_component(sourcepin, target1, source1):
    source = hal.Pin(sourcepin)
    if source.linked:
        sig = source.signal
        sigtype = sig.type
        sigreaders = sig.readers
        targets_found = 0
        targets = []
        for pinname in hal.pins():
            temppin = hal.Pin(pinname)
            if (temppin.signal == sig) and (temppin.name != source.name):
                targets_found += 1
                targets.append(temppin.name)
            if targets_found == sigreaders:
                break
        # now we've found the signal, delete it
        # link from source to new comp
        # link with signal name from new comp to original targets
        oldname = sig.name
        newname = 'new_{}'.format(sig.name)
        res = hal.delsig(sig.name)

        sig1 = hal.Signal(oldname, sigtype)
        sig2 = hal.Signal(newname, sigtype)
        sig1.link(source.name)
        if target1 != '':
            newpin_in = hal.Pin(target1)
            sig1.link(target1)

        newpin_out = hal.Pin(source1)
        sig2.link(source1)
        for t in targets:
            sig2.link(t)

def insert_jplanners():
    mod_success = hal.newsig("mod_success", hal.HAL_BIT)
    mod_success.set(0)

    # check if 'pbmsgs' component exists
    c = hal.components
    if 'pbmsgs' not in c:
        rt.loadrt('pbmsgs')

    # create 'series' signal
    series = hal.newsig("series", hal.HAL_U32)
    series.set(0)

    rt.newinst('ornv2', 'jplanners_active', pincount=8)

    # start changing the HAL configuration
    for i in range(1,7):

        # create jplanner
        rt.newinst('jplan', 'jp%s' %i)
        hal.addf('jp%s.update' %i, 'robot_hw_thread', 68+i)
        hal.Pin('jp%s.0.active' %i).link('jplanners_active.in%s' %(i-1))
        time.sleep(0.005)
        # copy current position
        hal.Pin('jp%s.0.pos-cmd' %i).set(hal.Pin('hal_hw_interface.joint_%s.pos-fb' %i).get())

        # set values for jplanner
        hal.Pin('jp%s.0.enable' %i).set(1)
        hal.Pin('jp%s.0.max-acc' %i).set(0.1)
        hal.Pin('jp%s.0.max-vel' %i).set(0.1)

        # get component to insert _after_
        source = 'hal_hw_interface.joint_%s.pos-cmd' %i
        rt.newinst('mux2v2', 'joint%s_mux' %i)
        hal.addf('joint%s_mux.funct' %i, 'robot_hw_thread', 68+2*i)
        time.sleep(0.005)
        # insert the mux component _after_ source component
        # the target1 is the new pin to be connected to the source component
        # the source1 is the pin the existing signals are to be connected to
        insert_component(source, 'joint%s_mux.in0' %i, 'joint%s_mux.out' %i)
        
        # connect to the inserted mux component
        hal.Pin('jp%s.0.curr-pos' %i).link('joint%s_mux.in1' %i)

        # create sample_channel
        rt.newinst('sample_channel_pb', 'sampler%s'  %i, '--', 'samples=bfu','names=sensor,rotation,series','cycles=2000')
        hal.addf('sampler%s.record_sample' %i, 'robot_hw_thread')
        hal.Signal('joint%s_ros_pos_fb' %i).link('sampler%s.in-flt.1' %i)
        hal.Pin('lcec.0.6.din-7').link('sampler%s.in-bit.1' %i)

        # link series signal to sample_channel series pin
        series.link('sampler%s.in-u32.1' %i)

        # select the jplanner channel
        hal.Pin('joint%s_mux.sel' %i).set(1)
        # leave info in HAL that this script was succesful
        
        mod_success.set(1)

    hal.addf('jplanners_active.funct', 'robot_hw_thread', 75)

def change_config():
    
    # connect to HAL
    try: 
        rt.init_RTAPI()
    except RuntimeError as e:
        pass

    #hal.stop_threads()
    time.sleep(0.1)

    # check for an existing signal "mod_success"
    try:
        hal.Signal("mod_success")
    except RuntimeError as e:
        if e.message == "signal 'mod_success' does not exist":
            # good to go, no previous attempt
            insert_jplanners()
            #hal.start_threads()

if __name__ == "__main__":
    change_config()


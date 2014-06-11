#!/usr/bin/python2
import signal
import sys
import epics
from epics.devices import scan, scaler
from epics.motor import MotorException
import time
import multiprocessing as mp
import threading
import numpy as np
import scipy as sp
import ipdb
from settings import xfm as cfg

class StageStack(object):

    def __init__(self, **kwargs):
        "keywords should be name=epics_motor_pv"
        self.axes = []
        for key in kwargs:
            try:
                setattr(self, key, epics.Motor(kwargs[key]))
                self.axes.append(key)
            except MotorException:
                setattr(self, key, epics.PV(kwargs[key]))

    def toggle_lock_state(self):
        state = self.lock_state.get()
        self.lock_state.put(1-state)
        for axis in self.axes:
            getattr(self, axis).disabled = 1-state

class SXM_Manager(object):

    def __init__(self, task_queue):
        self.task_queue = task_queue

        self.soft_prefix = cfg.soft_prefix
        self.ioc_prefix = cfg.ioc_prefix
        self.xfd_prefix = cfg.xfd_prefix
        self.cam_prefix = cfg.cam_prefix

        self.batch_x_width_pvname = '{:s}B1:scan1_{:d}:Width.VAL'
        self.batch_x_center_pvname = '{:s}B1:scan1_{:d}:Center.VAL'
        self.batch_x_npts_pvname = '{:s}B1:scan1_{:d}:NumPts.VAL'
        self.batch_y_width_pvname = '{:s}B1:scan2_{:d}:Width.VAL'
        self.batch_y_center_pvname = '{:s}B1:scan2_{:d}:Center.VAL'
        self.batch_y_npts_pvname = '{:s}B1:scan2_{:d}:NumPts.VAL'
        self.batch_dwell_pvname = '{:s}B1:scan_{:d}:Dwell.VAL'
        self.batch_comment_pvname = '{:s}B1:scan_{:d}:Comment.VAL'

        self.callbacks = cfg.callbacks

        # define scan records
        print('Initialize scan records...')
        for sc in cfg.scan_records:
            setattr(self, sc, scan.Scan(self.ioc_prefix+sc))
            # Add time estimate callback
            for name in ['NPTS', 'P1WD', 'P1SI']:
                self.callbacks[getattr(self, sc).PV(name).pvname] = 'update_time_estimate'
            # Add time remaining callback
            if sc in ['scan2', 'Fscan1']:
                self.callbacks[getattr(self, sc).PV('CPT').pvname] = 'update_time_remaining'

        # define scalers
        print('Initialize scalers...')
        for sc in cfg.scalers:
            setattr(self, sc, scaler.Scaler(self.ioc_prefix+sc))

        # define stage stacks
        print('Intialize stage stacks...')
        for stack in cfg.stage_stacks.keys():
            setattr(self, stack, StageStack(**cfg.stage_stacks[stack]))

        # Define MEDM/EPICS interfaces
        self.scan_axis_name_1 = epics.PV(self.soft_prefix+'scan_axis_name_1')
        self.scan_axis_name_2 = epics.PV(self.soft_prefix+'scan_axis_name_2')
        self.abort_scan = epics.PV(self.ioc_prefix+'AbortScans.PROC')
        self.thinking = epics.PV(self.soft_prefix+'thinking.VAL')

        # Define common scan axes
        self.scan_axes = cfg.scan_axes

        # Define lock state
        self.lock_state = cfg.lock_state

        # Define Step or Fly State
        self.stepfly = 'step'

        # Add move/define axes
        self.move_define_axes = cfg.move_define_axes

        # Add control PVs & callbacks
        print('Initialize callbacks...')
        self.c_pvs = {}
        for c_pv in self.callbacks.keys():
            self.c_pvs[c_pv] = epics.PV(c_pv, callback=self.on_changes)

    def run(self):
        while True:
            next_task = self.task_queue.get()
            if next_task is None:
                print('{:s}: Exiting').format(self.name)
                self.task_queue.task_done()
                break
            else:
                pvname, value = next_task
                getattr(self, self.callbacks[pvname])(pvname=pvname, value=value)
                self.task_queue.task_done()
        return

    def on_changes(self, *args, **kwargs):
        self.task_queue.put([kwargs['pvname'], kwargs['value']])

    @staticmethod
    def heartbeat_wait(task_queue, pv, value):
        time.sleep(2.5)
        pv.put(1-value)

    def toggle_heartbeat(self, pvname, value=None, **kws):
        t = threading.Thread(target=self.heartbeat_wait, args=(self.task_queue,self.c_pvs[pvname],value))
        t.start()

    def toggle_lock_state(self, pvname, value, **kws):
        if value>0:
            getattr(self, self.lock_state[value]).toggle_lock_state()

    def do_1d_scan(self, pvname, value, **kwargs):

        self.thinking.put(1)
        if value == 1:

            if self.stepfly == 'step':

                self.scan1.BSPV = ''
                self.scan1.BSCD = 1
                self.scan1.ASPV = ''
                self.scan1.ASCD = 1

                self.scan1.P1SM = 0 #Linear
                self.scan1.P1AR = 1 # Relative
                self.scan1.PASM = 2 # Prior Pos
                self.scan1.PDLY = 0

                self.scan1.EXSC = 1

        self.thinking.put(0)

    def do_2d_scan(self, pvname, value, **kwargs):

        self.thinking.put(1)
        if value == 1:

            if self.stepfly == 'step':

                self.scan2.BSPV = ''
                self.scan2.BSCD = 1
                self.scan2.ASPV = ''
                self.scan2.ASCD = 1

                self.scan1.P1SM = 0 #Linear
                self.scan2.P1SM = 0 #Linear
                self.scan1.P1AR = 1 # Relative
                self.scan2.P1AR = 1 # Relative
                self.scan1.PASM = 2 # Prior Pos
                self.scan2.PASM = 2 # Prior Pos
                self.scan1.PDLY = 0
                self.scan2.PDLY = 0

                self.scan2.T1PV = self.scan1.PV('EXSC').pvname

                self.scan2.EXSC = 1

        self.thinking.put(0)

    def select_scan_type(self, value, **kwargs):

        self.thinking.put(1)

        if value == 0: # CFG
            pass
        elif value == 1: # Ptycho
            pass

        self.thinking.put(0)

    def toggle_stepfly_state(self, pvname, value, **kwargs):
        if value==0:
            self.stepfly = 'step'
        else:
            self.stepfly = 'fly'
            self.select_scan_axes(None, 0) # Sample XY
            self.c_pvs['2xfmS1:scan_axes_select.VAL'].put(0)
        self.update_time_estimate()

    def select_scan_axes(self, pvname, value, **kwargs):
        self.thinking.put(1)

        x_positioner, y_positioner = self.scan_axes[value]
        x_stage, x_axis = x_positioner.split('.')
        y_stage, y_axis = y_positioner.split('.')

        if self.stepfly == 'step':
            self.scan1.P1PV = getattr(getattr(self, x_stage), x_axis).PV('VAL').pvname
            self.scan2.P1PV = getattr(getattr(self, y_stage), y_axis).PV('VAL').pvname
        else: #Fly
            self.FscanH.P1PV = getattr(getattr(self, x_stage), x_axis).PV('VAL').pvname
            self.Fscan1.P1PV = getattr(getattr(self, y_stage), y_axis).PV('VAL').pvname

        try:
            self.scan_axis_name_1.put(getattr(getattr(self, x_stage), x_axis).DESC)
            self.scan_axis_name_2.put(getattr(getattr(self, y_stage), y_axis).DESC)
        except:
            self.scan_axis_name_1.put('X Positioner')
            self.scan_axis_name_2.put('Y Positioner')

        self.thinking.put(0)

    def pinhole_move(self, pvname, value, **kwargs):
        self.thinking.put(1)
        self.pinhole.x.move(epics.caget(self.soft_prefix+'pinhole_x_{:d}.VAL'.format(value)))
        self.pinhole.y.move(epics.caget(self.soft_prefix+'pinhole_y_{:d}.VAL'.format(value)))
        self.thinking.put(0)

    def pinhole_define(self, pvname, value, **kwargs):
        self.thinking.put(1)
        epics.caput(self.soft_prefix+'pinhole_x_{:d}.VAL'.format(value), self.pinhole.x.VAL)
        epics.caput(self.soft_prefix+'pinhole_y_{:d}.VAL'.format(value), self.pinhole.y.VAL)
        self.thinking.put(0)

    def stage_stack_move(self, value, wait=False, **kwargs):
        if value>0:
            stack, location = self.move_define_axes[value]

            for axis in getattr(self, stack).axes:
                if axis not in ['top', 'middle', 'theta']:
                    getattr(getattr(self, stack), axis).move(epics.caget(self.soft_prefix+'{:s}_{:s}_{:s}.VAL'.format(stack, axis, location)), wait=wait)

    def stage_stack_define(self, value, **kwargs):
        if value>0:
            stack, location = self.move_define_axes[value]

            for axis in getattr(self, stack).axes:
                if axis not in ['top', 'middle', 'theta']:
                    epics.caput(self.soft_prefix+'{:s}_{:s}_{:s}.VAL'.format(stack, axis, location), getattr(getattr(self, stack), axis).VAL )

    def update_dwell(self, pvname, value, **kwargs):
        value /= 1e3 # ms to s
        # Send dwell to all the scalers
        for sc in cfg.scalers:
            try:
                getattr(self, sc).CountTime(value)
            except:
                print("Couldn't set dwell time for {:s}.".format(sc))
                pass

        # Send dwell to the fluorescence detector
        pass

        # Send dwell to the ptycho camera
        pass

        self.update_time_estimate()

    def update_time_estimate(self, **kwargs):
        try:
            dwell = self.c_pvs[self.soft_prefix+'dwell.VAL'].get()/1e3
            if self.stepfly == 'step':
                n_pts = self.scan1.NPTS
                n_lines = self.scan2.NPTS
                oh = cfg.time_estimate_overhead['step']
            elif self.stepfly == 'fly':
                n_pts = self.FscanH.NPTS
                n_lines = self.Fscan1.NPTS
                oh = cfg.time_estimate_overhead['fly']
            time_per_line = oh[0]+dwell*n_pts*oh[1]
            time_per_image = oh[2]+time_per_line*n_lines*oh[3]
            epics.caput(self.soft_prefix+'hr_estimate_line.VAL', np.floor(time_per_line/3600.0))
            epics.caput(self.soft_prefix+'min_estimate_line.VAL', np.mod(time_per_line, 3600.0)/60.0)
            epics.caput(self.soft_prefix+'hr_estimate_image.VAL', np.floor(time_per_image/3600.0))
            epics.caput(self.soft_prefix+'min_estimate_image.VAL', np.mod(time_per_image, 3600.0)/60.0)
        except AttributeError:
            pass

        if self.stepfly == 'fly':
            self.calculate_fly_ranges()

    def update_time_remaining(self, **kwargs):
        try:
            dwell = self.c_pvs[self.soft_prefix+'dwell.VAL'].get()/1e3
            if self.stepfly == 'step':
                n_pts = self.scan1.NPTS
                n_lines = self.scan2.NPTS
                c_line = self.scan2.CPT
                oh = cfg.time_estimate_overhead['step']
            elif self.stepfly == 'fly':
                n_pts = self.FscanH.NPTS
                n_lines = self.Fscan1.NPTS
                c_line = self.Fscan1.CPT
                oh = cfg.time_estimate_overhead['fly']
            if c_line>n_lines:
                c_line = n_lines
            time_per_line = oh[0]+dwell*n_pts*oh[1]
            time_per_image = oh[2]+time_per_line*(n_lines-c_line)*oh[3]
            epics.caput(self.soft_prefix+'hr_remaining.VAL', np.floor(time_per_image/3600.0), timeout=0.1)
            epics.caput(self.soft_prefix+'min_remaining.VAL', np.mod(time_per_image, 3600.0)/60.0, timeout=0.1)
        except AttributeError:
            raise

    def alignment_mode(self, value, **kwargs):
        if value == 0: # Measurement mode
            # Move OSA in
            self.stage_stack_move(value=5)
            if self.c_pvs[self.soft_prefix+'scan_type_select.VAL'].get() == 0: # CFG
                self.stage_stack_move(value=13)
            elif self.c_pvs[self.soft_prefix+'scan_type_select.VAL'].get() == 1: # Ptycho
                self.stage_stack_move(value=14)

        elif value == 1: # Alignment mode
            # Move OSA out
            self.stage_stack_move(value=6)
            # Move CCD in
            self.stage_stack_move(value=12)

    def push_to_batch(self, value, **kwargs):
        if value>0:
            value -= 1
            try:
                if self.stepfly == 'step':
                    x_center = self.scan1.PV('P1CP').get()
                    x_width = self.scan1.PV('P1WD').get()
                    x_pts = self.scan1.PV('NPTS').get()
                    y_center = self.scan2.PV('P1CP').get()
                    y_width = self.scan2.PV('P1WD').get()
                    y_pts = self.scan2.PV('NPTS').get()
                elif self.stepfly == 'fly':
                    x_center = self.FscanH.PV('P1CP').get()
                    x_width = self.FscanH.PV('P1WD').get()
                    x_pts = self.FscanH.PV('NPTS').get()
                    y_center = self.Fscan1.PV('P1CP').get()
                    y_width = self.Fscan1.PV('P1WD').get()
                    y_pts = self.Fscan1.PV('NPTS').get()
                comment = epics.caget('2xfm:userStringCalc10.DD')
                dwell = self.c_pvs[self.soft_prefix+'dwell.VAL'].get()

                print(self.batch_x_center_pvname.format(self.soft_prefix, value), x_center)
                print(self.batch_x_width_pvname.format(self.soft_prefix, value), x_width)
                print(self.batch_x_npts_pvname.format(self.soft_prefix, value), x_pts)
                print(self.batch_y_center_pvname.format(self.soft_prefix, value), y_center)
                print(self.batch_y_width_pvname.format(self.soft_prefix, value), y_width)
                print(self.batch_y_npts_pvname.format(self.soft_prefix, value), y_pts)
                print(self.batch_comment_pvname.format(self.soft_prefix, value), comment)
                print(self.batch_dwell_pvname.format(self.soft_prefix, value), dwell)

                #epics.caput(self.batch_x_center_pvname.format(self.soft_prefix, value), x_center)
                #epics.caput(self.batch_x_width_pvname.format(self.soft_prefix, value), x_width)
                #epics.caput(self.batch_x_npts_pvname.format(self.soft_prefix, value), x_pts)
                #epics.caput(self.batch_y_center_pvname.format(self.soft_prefix, value), y_center)
                #epics.caput(self.batch_y_width_pvname.format(self.soft_prefix, value), y_width)
                #epics.caput(self.batch_y_npts_pvname.format(self.soft_prefix, value), y_pts)
                #epics.caput(self.batch_comment_pvname.format(self.soft_prefix, value), comment)
                #epics.caput(self.batch_dwell_pvname.format(self.soft_prefix, value), dwell)

            except:
                raise

    def calculate_fly_ranges(self, **kwargs):
        # Min speed
        min_vel = epics.caget('2xfmS1:FlySetup:MinBaseVel.VAL')
        max_vel = epics.caget('2xfmS1:FlySetup:MaxVelocity.VAL')
        if min_vel == 0.0:
            return
        if max_vel == 0.0:
            return
        dwell = self.c_pvs[self.soft_prefix+'dwell.VAL'].get()/1e3
        step = self.FscanH.PV('P1SI').get()
        width = self.FscanH.PV('P1WD').get()
        npts = self.FscanH.PV('NPTS').get()

        if np.mod(step/0.0001, 1.0)>0.0:
            epics.caput('2xfmS1:warning1.VAL', 'Step size should be multiple of 100nm')
            epics.caput('2xfmS1:show_warning1.VAL', 1)
        else:
            epics.caput('2xfmS1:warning1.VAL', '')
            epics.caput('2xfmS1:show_warning1.VAL', 0)

        try:
            epics.caput(self.soft_prefix+'max_dwell.VAL', step/min_vel)
            epics.caput(self.soft_prefix+'min_dwell.VAL', step/max_vel)

            epics.caput(self.soft_prefix+'max_step.VAL', dwell/min_vel)
            epics.caput(self.soft_prefix+'min_step.VAL', 0.0001)

            epics.caput(self.soft_prefix+'max_width.VAL', npts*dwell/min_vel)
            epics.caput(self.soft_prefix+'min_width.VAL', npts*dwell/max_vel)
        except:
            raise

    def align_cfg(self, **kwargs):
        # Move cfg in
        self.move_stage_stack(value=2, wait=True)

        # set scaler count time
        self.scaler1.time = 0.1

        def move_to_peak(xl, yl):
            xmax, ymax, m = None, None, None
            for x in xl:
                for y in yl:
                    self.tx_det.x.move(x, wait=True)
                    self.tx_det.y.move(y, wait=True)
                    # Trigger scaler
                    self.scaler1.count()
                    if not m:
                        m = self.scaler1.channel_5
                        xm = x
                        ym = y
                    else:
                        if self.scaler1.channel_5>m:
                            m=self.scaler1.channel_5
                            xm = x
                            ym = y

            self.tx_det.x.move(xm)
            self.tx_det.y.move(ym)

        # Scan 1 1.2x1.2mm
        cur_pos_x = self.tx_det.x.get()
        cur_pos_y = self.tx_det.y.get()
        step_x = 0.06 #mm
        step_y = 0.06 #mm
        xl,yl = np.meshgrid(np.arange(-10,11)*step_x+cur_pos_x,np.arange(-10,11)*step_y+cur_pos_y)
        move_to_peak(xl, yl)
        # Scan 2 0.12x0.12mm
        cur_pos_x = self.tx_det.x.get()
        cur_pos_y = self.tx_det.y.get()
        step_x = 0.006 #mm
        step_y = 0.006 #mm
        xl,yl = np.meshgrid(np.arange(-10,11)*step_x+cur_pos_x,np.arange(-10,11)*step_y+cur_pos_y)
        move_to_peak(xl, yl)

def handle_exit(queue):
    def signal_handler(signal,frame):
        queue.put(None)
        print('Caught ctrl-c, waiting for threads to exit')
        sys.exit(0)
    return signal_handler

def mainloop():
    task_queue = mp.JoinableQueue()
    signal_handler = handle_exit(task_queue)
    signal.signal(signal.SIGINT, signal_handler)
    sxm = SXM_Manager(task_queue)
    print('Ready.')
    sxm.run()
    ipdb.set_trace()
    task_queue.join()

if __name__ == '__main__':
    mainloop()

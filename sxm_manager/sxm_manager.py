#!/usr/bin/python2
import signal
import glob
import sys
import os
import shutil
import epics
from devices import scan, scaler
from epics.motor import MotorException
import time
import multiprocessing as mp
import threading
import numpy as np
import scipy as sp
import ipdb
from settings import xfm as cfg
from beamon import beamline

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

    def redefine_to(self, vals):
        # Redefine the current position
        # Vals should be dict like {'x':0,'y':0}
        try:
            for axis in vals.keys():
                getattr(self, axis).set_position(vals[axis])
        except:
            raise

class SXM_Manager(object):

    def __init__(self, task_queue):

        # To avoid callbacks running on startup they must be 'enabled'
        # by adding them to this list on the first change.
        self.enabled_callbacks = []

        self.task_queue = task_queue

        self.soft_prefix = cfg.soft_prefix
        self.ioc_prefix = cfg.ioc_prefix
        self.xfd_prefix = cfg.xfd_prefix
        self.cam_prefix = cfg.cam_prefix

        self.batch_x_width_pvname = '{:s}B1:Scan1_{:d}:Width.VAL'
        self.batch_x_center_pvname = '{:s}B1:Scan1_{:d}:Center.VAL'
        self.batch_x_npts_pvname = '{:s}B1:Scan1_{:d}:NumPts.VAL'
        self.batch_y_width_pvname = '{:s}B1:Scan2_{:d}:Width.VAL'
        self.batch_y_center_pvname = '{:s}B1:Scan2_{:d}:Center.VAL'
        self.batch_y_npts_pvname = '{:s}B1:Scan2_{:d}:NumPts.VAL'
        self.batch_dwell_pvname = '{:s}B1:Scan_{:d}:Dwell.VAL'
        self.batch_comment_pvname = '{:s}B1:Scan_{:d}:Comment.VAL'

        self.callbacks = cfg.callbacks

        # define scan records
        print('Initialize scan records...')
        for sc in cfg.scan_records:
            setattr(self, sc, scan.Scan(self.ioc_prefix+sc))
            # Add time estimate callback
            for name in ['NPTS', 'P1WD', 'P1SI']:
                self.callbacks[getattr(self, sc).PV(name).pvname] = 'update_time_estimate'
            # Add time remaining callback
            if sc in ['scan1', 'FscanH']:
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

        self.c_pvs[self.soft_prefix+'heartbeat.VAL'].put(1-self.c_pvs[self.soft_prefix+'heartbeat.VAL'].get())

    def run(self):
        while True:
            next_task = self.task_queue.get()
            if next_task is None:
                print('{:s}: Exiting').format(self.name)
                self.task_queue.task_done()
                break
            else:
                pvname, value = next_task
                if pvname in self.enabled_callbacks:
                    getattr(self, self.callbacks[pvname])(pvname=pvname, value=value)
                else:
                    self.enabled_callbacks.append(pvname)
                self.task_queue.task_done()
        return

    def on_changes(self, *args, **kwargs):
        self.task_queue.put([kwargs['pvname'], kwargs['value']])

    @staticmethod
    def heartbeat_wait(task_queue, pv, value):
        time.sleep(2.5)
        SXM_Manager.update_maps_status()
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

                if self.scan1.P1PV == '2xfm:m24.VAL':
                    self.scan1.P1WD = -1*abs(self.scan1.P1WD)

                self.scan1.EXSC = 1

            if self.stepfly == 'fly':
                print('1D Fly Scans not supported')

        self.thinking.put(0)

    def do_2d_scan(self, pvname, value, **kwargs):

        self.thinking.put(1)
        if value == 1:

            if self.stepfly == 'step':

                if self.scan1.P1PV == '2xfm:m24.VAL':
                    self.scan1.P1WD = -1*abs(self.scan1.P1WD)

                self.scan2.T1PV = self.scan1.PV('EXSC').pvname

                self.scan2.EXSC = 1

            if self.stepfly == 'fly':

                self.Fscan1.T1PV = self.FscanH.PV('EXSC').pvname

                self.Fscan1.EXSC = 1

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

        try:
            if self.stepfly == 'step':
                self.scan1.P1PV = getattr(getattr(self, x_stage), x_axis).PV('VAL').pvname
                self.scan2.P1PV = getattr(getattr(self, y_stage), y_axis).PV('VAL').pvname
                self.scan_axis_name_1.put(getattr(getattr(self, x_stage), x_axis).DESC)
                self.scan_axis_name_2.put(getattr(getattr(self, y_stage), y_axis).DESC)
            else: #Fly
                if value==0:
                    self.FscanH.P1PV = '2xfm:FLYuserCalc10.A'
                    self.scan_axis_name_1.put('Sample X')
                else:
                    self.FscanH.P1PV = getattr(getattr(self, x_stage), x_axis).PV('VAL').pvname
                    self.scan_axis_name_1.put(getattr(getattr(self, x_stage), x_axis).DESC)
                self.Fscan1.P1PV = getattr(getattr(self, y_stage), y_axis).PV('VAL').pvname
                self.scan_axis_name_2.put(getattr(getattr(self, y_stage), y_axis).DESC)
        except:
            self.scan_axis_name_1.put('X Positioner')
            self.scan_axis_name_2.put('Y Positioner')
            raise

        self.thinking.put(0)

    def pinhole_move(self, pvname, value, **kwargs):
        self.thinking.put(1)
        self.pinhole.x.move(epics.caget(self.soft_prefix+'pinhole_x_{:d}.VAL'.format(value)))
        self.pinhole.y.move(epics.caget(self.soft_prefix+'pinhole_y_{:d}.VAL'.format(value)))
        self.thinking.put(0)

    def pinhole_define(self, pvname, value, **kwargs):
        # All of the pinhole positions are defined relative to the
        # 500 micron pinhole which is set to (x,y)=(0,0)
        self.thinking.put(1)

        if value==5: # 500micron pinhole
            self.pinhole.redefine_to({'x':0, 'y':0})
            time.sleep(0.1)
        epics.caput(self.soft_prefix+'pinhole_x_{:d}.VAL'.format(value), self.pinhole.x.VAL)
        epics.caput(self.soft_prefix+'pinhole_y_{:d}.VAL'.format(value), self.pinhole.y.VAL)
        time.sleep(0.1)
        self.pinhole_move(pvname, value)
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

            if stack=='osa' and location=='in':
                self.osa.redefine_to({'x':0, 'y':0})
                # Define OSA Out as +3.5mm relative to OSA In
                epics.caput(self.soft_prefix+'osa_x_out.VAL', 3.5)
                epics.caput(self.soft_prefix+'osa_y_out.VAL', 0.0)
            elif stack=='zp20' and location=='in':
                # Define zp20 out as x=-25
                epics.caput(self.soft_prefix+'zp20_x_out.VAL', 25.0)
                epics.caput(self.soft_prefix+'zp20_y_out.VAL', self.zp20.y.VAL)
                epics.caput(self.soft_prefix+'zp20_z_out.VAL', self.zp20.z.VAL)
            elif stack=='zp10' and location=='in':
                # Define zp10 out as x=+10
                epics.caput(self.soft_prefix+'zp10_x_out.VAL', 10.0)
                epics.caput(self.soft_prefix+'zp10_y_out.VAL', self.zp20.y.VAL)
                epics.caput(self.soft_prefix+'zp10_z_out.VAL', self.zp20.z.VAL)
            elif stack=='tx_det' and location=='1':
                # CCD in
                self.tx_det.redefine_to({'x':0, 'y':0})

            for axis in getattr(self, stack).axes:
                if axis not in ['top', 'middle', 'theta']:
                    epics.caput(self.soft_prefix+'{:s}_{:s}_{:s}.VAL'.format(stack, axis, location), getattr(getattr(self, stack), axis).VAL )

    def update_dwell(self, pvname, value, **kwargs):
        # Updating any dwell time (master, step, fly) updates the time estimate

        if pvname == self.soft_prefix+'dwell.VAL':

            try:# set step scan master dwell
                epics.caput('2xfm:userTran1.P', value/1e3, timeout=0.1)
            except:
                print("couldn't set step scan dwell.")
            try:# set fly scan master dwell
                epics.caput('2xfm:FlySetup:DwellTime.VAL', value, timeout=0.1)
            except:
                print("couldn't set fly scan dwell.")

        self.update_time_estimate()

    def update_time_estimate(self, **kwargs):
        try:
            if self.stepfly == 'step':
                dwell = epics.caget('2xfm:userTran1.P', timeout=0.1)
                n_pts = self.scan1.NPTS
                n_lines = self.scan2.NPTS
                oh = cfg.time_estimate_overhead['step']
            elif self.stepfly == 'fly':
                dwell = epics.caget('2xfm:FlySetup:DwellTime.VAL', timeout=0.1)*1e-3
                n_pts = self.FscanH.NPTS
                n_lines = self.Fscan1.NPTS
                oh = cfg.time_estimate_overhead['fly']
            time_per_line = oh[0]+dwell*n_pts*oh[1]
            time_per_image = oh[2]+time_per_line*n_lines*oh[3]
            epics.caput(self.soft_prefix+'hr_estimate_line.VAL', np.floor(time_per_line/3600.0), timeout=0.1)
            epics.caput(self.soft_prefix+'min_estimate_line.VAL', np.mod(time_per_line, 3600.0)/60.0, timeout=0.1)
            epics.caput(self.soft_prefix+'hr_estimate_image.VAL', np.floor(time_per_image/3600.0), timeout=0.1)
            epics.caput(self.soft_prefix+'min_estimate_image.VAL', np.mod(time_per_image, 3600.0)/60.0, timeout=0.1)
        except AttributeError:
            raise

        if self.stepfly == 'fly':
            self.calculate_fly_ranges()

    def update_time_remaining(self, **kwargs):
        try:
            if self.stepfly == 'step':
                dwell = epics.caget('2xfm:userTran1.P', timeout=0.1)
                n_pts = self.scan1.NPTS
                n_lines = self.scan2.NPTS
                c_line = self.scan2.CPT
                oh = cfg.time_estimate_overhead['step']
            elif self.stepfly == 'fly':
                dwell = epics.caget('2xfm:FlySetup:DwellTime.VAL', timeout=0.1)*1e-3
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
        except TypeError:
            # Probably a caget timed out
            pass
        except AttributeError:
            raise

    def alignment_mode(self, value, **kwargs):
        epics.caput('2xfmS1:alignment_mode_status.VAL', 1)
        if value == 0: # Measurement mode
            # Move OSA in
            self.stage_stack_move(value=5)
            if self.c_pvs[self.soft_prefix+'scan_type_select.VAL'].get() == 0: # CFG
                self.stage_stack_move(value=13, wait=True)
            elif self.c_pvs[self.soft_prefix+'scan_type_select.VAL'].get() == 1: # Ptycho
                self.stage_stack_move(value=14, wait=True)

        elif value == 1: # Alignment mode
            # Move OSA out
            self.stage_stack_move(value=6, wait=True)
            # Move CCD in
            self.stage_stack_move(value=12, wait=True)
        epics.caput('2xfmS1:alignment_mode_status.VAL', 0)

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
                    dwell = 1e-3*self.c_pvs[self.soft_prefix+'dwell.VAL'].get()
                elif self.stepfly == 'fly':
                    x_center = self.FscanH.PV('P1CP').get()
                    x_width = self.FscanH.PV('P1WD').get()
                    x_pts = self.FscanH.PV('NPTS').get()
                    y_center = self.Fscan1.PV('P1CP').get()
                    y_width = self.Fscan1.PV('P1WD').get()
                    y_pts = self.Fscan1.PV('NPTS').get()
                    dwell = self.c_pvs[self.ioc_prefix+':FlySetup:DwellTime.VAL'].get()
                comment = epics.caget('2xfm:userStringCalc10.DD')

                epics.caput(self.batch_x_center_pvname.format(self.ioc_prefix, value), x_center)
                epics.caput(self.batch_x_width_pvname.format(self.ioc_prefix, value), x_width)
                epics.caput(self.batch_x_npts_pvname.format(self.ioc_prefix, value), x_pts)
                epics.caput(self.batch_y_center_pvname.format(self.ioc_prefix, value), y_center)
                epics.caput(self.batch_y_width_pvname.format(self.ioc_prefix, value), y_width)
                epics.caput(self.batch_y_npts_pvname.format(self.ioc_prefix, value), y_pts)
                epics.caput(self.batch_comment_pvname.format(self.ioc_prefix, value), comment)
                epics.caput(self.batch_dwell_pvname.format(self.ioc_prefix, value), dwell)

            except:
                raise

    def calculate_fly_ranges(self, **kwargs):
        # Min speed
        min_vel = epics.caget('2xfm:FlySetup:MinBaseVel.VAL')
        max_vel = epics.caget('2xfm:FlySetup:MaxVelocity.VAL')
        if min_vel == 0.0:
            return
        if max_vel == 0.0:
            return
        dwell = self.c_pvs[self.soft_prefix+'dwell.VAL'].get()/1e3
        step = abs(self.FscanH.PV('P1SI').get())
        width = abs(self.FscanH.PV('P1WD').get())
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

    def generate_config(self, pvname, value, **kwargs):
        if value==1:
            user = epics.caget('2xfmS1:user_string.VAL')
            run = epics.caget('2xfmS1:run_string.VAL')

            beamline.generate_config(user, run)

    def maps_process_now(self, pvname, value, **kwargs):
        if value==1:
            user = epics.caget('2xfmS1:user_string.VAL')
            run = epics.caget('2xfmS1:run_string.VAL')

            fn_livejob = '/mnt/xfm0/data/2ide/{run:s}/{user:s}/livejob_{run:s}_2ide_{user:s}.txt'.format(**{'user':user, 'run':run})

            if os.path.isfile(fn_livejob):
                shutil.move(fn_livejob, '/mnt/xfm0/data/jobs')

    @staticmethod
    def update_maps_status():
        user = epics.caget('2xfmS1:user_string.VAL')
        run = epics.caget('2xfmS1:run_string.VAL')
        machines = {0: 'XFM3', 1: 'XFM3B', 2: 'XFM4', 3: 'XFM4B', 4: 'XFM5', 5: 'XFM5B', 6: 'XFM6', 7: 'XFM6B'}

        complete_livejob = '/mnt/xfm0/data/2ide/{run:s}/{user:s}/livejob_{run:s}_2ide_{user:s}.txt'.format(**{'user':user, 'run':run})
        waiting_livejob = '/mnt/xfm0/data/jobs/livejob_{run:s}_2ide_{user:s}.txt'.format(**{'user':user, 'run':run})
        processing_livejob = '/mnt/xfm0/data/jobs/processing/livejob_{run:s}_2ide_{user:s}.txt'.format(**{'user':user, 'run':run})
        processing_livejob_machine = '/mnt/xfm0/data/jobs/{machine:s}/livejob_{run:s}_2ide_{user:s}.txt'.format(**{'user':user, 'run':run, 'machine': '{:s}'})
        message = 'Processing update error'

        if os.path.exists(complete_livejob):
            message = 'MAPS Processing completed'
            epics.caput('2xfmS1:maps_processing_status.VAL', 0)
        elif os.path.exists(waiting_livejob):
            message = 'MAPS job in queue'
            epics.caput('2xfmS1:maps_processing_status.VAL', 1)
        elif os.path.exists(processing_livejob) or \
            any([os.path.exists(processing_livejob_machine.format(val.lower())) for key, val in machines.iteritems()]):
            message = "MAPS processing in progress"
            epics.caput('2xfmS1:maps_processing_status.VAL', 1)

        epics.caput('2xfmS1:maps_status_1.VAL', message)
        max_string_length = 35
        idle_message = '{:s}: IDLE'
        processing_message = '{:s}: {:s} {:s} {:s}'
        xfm0_jobs = '/mnt/xfm0/data/jobs/'
        for i in range(8):
            if os.path.exists(xfm0_jobs+'status_{:s}_idle.txt'.format(machines[i].lower())):
                epics.caput('2xfmS1:maps_status_{:d}.VAL'.format(i+2), idle_message.format(machines[i]), timeout=0.1)
            elif os.path.exists(xfm0_jobs+'status_{:s}_working.txt'.format(machines[i].lower())):
                try:
                    with open(xfm0_jobs+'status_{:s}_working.txt'.format(machines[i].lower()), 'r') as f:
                        lines = f.readlines()
                        proc_user = lines[1].split('_')[3]
                        if proc_user.find('.')>-1:
                            proc_user = proc_user.split('.')[0]
                        proc_run = lines[1].split('_')[1]
                        proc_beamline = lines[1].split('_')[2]
                        epics.caput('2xfmS1:maps_status_{:d}.VAL'.format(i+2), processing_message.format(machines[i], proc_beamline, proc_run, proc_user)[:max_string_length], timeout=0.1)
                except IOError:
                    print('Could not open {:s}'.format(xfm0_jobs+'status_{:s}_working.txt'.format(machines[i].lower())))
                except IndexError:
                    print('Caught an index error while parsing: status_{:s}_working.txt'.format(machines[i].lower()))
                except:
                    raise

    def maps_reprocess_all(self, pvname, value, **kwargs):

       if value == 1:
            user = epics.caget('2xfmS1:user_string.VAL')
            run = epics.caget('2xfmS1:run_string.VAL')

            if os.path.exists('/mnt/xfm0/data/2ide/{run:s}/{user:s}/output/local_done'.format(**{'user':user, 'run':run})):
                fn_mda_done = '/mnt/xfm0/data/2ide/{run:s}/{user:s}/output/local_done/*.txt'.format(**{'user':user, 'run':run})

                files = glob.glob(fn_mda_done)

                for file in files:
                    shutil.copy(file, '/tmp')
                    os.remove(file)


            self.maps_process_now(pvname=None, value=1)

    def scan_ends_process_now(self, pvname, value, **kwargs):

        if value==0:
            self.maps_process_now(pvname=None, value=1)

    def generate_user_dirs(self, pvname, value, **kwargs):
        if value==1:
            user = epics.caget('2xfmS1:user_string.VAL')
            beamline.setup_user_dirs(user)
        elif value==2:
            user = epics.caget('2xfmS1:user_string.VAL')
            beamline.setup_user_dirs_save_data_location(user)

    def update_user(self, pvname, value, **kwargs):
        if value==1:
            epics.caput('2xfmS1:user_string.VAL', beamline.schedule.get_pi().lower())
            epics.caput('2xfmS1:run_string.VAL', beamline.get_run_name_from_schedule())

    def take_standards(self, pvname, value, **kwargs):
        if value==1:
            old_comment = epics.caget('2xfm:userStringCalc10.CC')
            epics.caput('2xfm:userStringCalc10.CC', 'axo_std')
            self.sample.x.move(0, wait=True)
            self.sample.y.move(-2.55, wait=True)
            self.sample.z.move(0.5, wait=True)

            # Step scan
            epics.caput(self.soft_prefix+'stepfly.VAL', 0)
            epics.caput(self.soft_prefix+'scan_axes_select.VAL',0)
            epics.caput(self.ioc_prefix+'userTran1.P', 1)

            self.scan1.P1WD = -0.06
            self.scan1.NPTS = 61
            self.scan2.P1WD = 0.02
            self.scan2.NPTS = 3

            done = False
            epics.caput('2xfm:scanPause.VAL', 0)
            print('Starting step scan...')
            self.scan2.EXSC = 1
            time.sleep(10.0)
            while not done:
                time.sleep(5.0)
                done = self.scan2.BUSY==0
            print('Step scan done.')

            # Fly Scan
            epics.caput(self.soft_prefix+'stepfly.VAL', 1)
            epics.caput(self.ioc_prefix+'FlySetup:DwellTime.VAL', 30)

            self.FscanH.P1WD = 0.06
            self.FscanH.NPTS = 86
            self.Fscan1.P1WD = 0.02
            self.Fscan1.NPTS = 29

            done = False
            epics.caput('2xfm:FscanPause.VAL', 0)
            print('Starting fly scan...')
            self.Fscan1.EXSC = 1
            time.sleep(10.0)
            while not done:
                time.sleep(5.0)
                done = self.Fscan1.BUSY==0
            print('Step fly done.')

            epics.caput('2xfm:userStringCalc10.CC',old_comment)

    def align_cfg(self, **kwargs):
        if value==1:
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

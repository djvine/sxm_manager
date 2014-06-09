#!/usr/bin/python2
import signal
import sys
import epics
from epics.devices import scan, scaler
from epics.motor import MotorException
import time
import multiprocessing as mp
import threading
import scipy as sp
import ipdb
from settings import xfm as cfg

def in_thread(func):
    def in_thread2(self=None, *args, **kwargs):
        threading.Thread(target=func, args=(self,args,kwargs)).start()
    return in_thread2

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

        # define scan records
        for sc in cfg.scan_records:
            setattr(self, sc, scan.Scan(self.ioc_prefix+sc))

        # define scalers
        for sc in cfg.scalers:
            setattr(self, sc, scaler.Scaler(self.ioc_prefix+sc))

        # define stage stacks
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
        self.callbacks = cfg.callbacks
        self.c_pvs = {}
        for c_pv in cfg.callbacks.keys():
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




    """
    def do_scan(self, *args, **kwargs):

        if args[1]['value'] == 1:

            self.thinking.put(1)
            if self.scand == 2: # 2D scans
                #N1,N2 = self.scan1.NPTS,self.scan2.NPTS
                #self.scan1.reset()
                #self.scan2.reset()
                #self.scan1.NPTS,self.scan2.NPTS = N1, N2
                if self.autoshutter.state.get() == 1:
                    self.scan2.BSPV = '2idb1:9440:1:bo_0.VAL'
                    self.scan2.BSCD = 0
                    self.scan2.ASPV = '2idb1:9440:1:bo_0.VAL'
                    self.scan2.ASCD = 1
                else:
                    self.scan2.BSPV = ''
                    self.scan2.BSCD = 0
                    self.scan2.ASPV = ''
                    self.scan2.ASCD = 0

                self.scan1.P1SM = 0 #Linear
                self.scan2.P1SM = 0 #Linear
                self.scan1.P1AR = 1 # Relative
                self.scan2.P1AR = 1 # Relative
                self.scan1.PASM = 2 # Prior Pos
                self.scan2.PASM = 2 # Prior Pos
                self.scan1.PDLY = 0
                self.scan2.PDLY = 0

                self.scan2.T1PV = self.scan1.PV('EXSC').pvname

            elif self.scand==3: # 3D scans
                #N1,N2,N3 = self.scan1.NPTS,self.scan2.NPTS, self.scan3.NPTS
                #self.scan1.reset()
                # self.scan2.reset()
                #self.scan3.reset()
                #self.scan1.NPTS,self.scan2.NPTS, self.scan3.NPTS = N1,N2,N3

                if self.autoshutter.state.get() == 1:
                    self.scan3.BSPV = '2idb1:9440:1:bo_0.VAL'
                    self.scan3.BSCD = 0
                    self.scan3.ASPV = '2idb1:9440:1:bo_0.VAL'
                    self.scan3.ASCD = 1
                else:
                    self.scan3.BSPV = ''
                    self.scan3.BSCD = 0
                    self.scan3.ASPV = ''
                    self.scan3.ASCD = 0
                    self.scan2.BSPV = ''
                    self.scan2.BSCD = 0
                    self.scan2.ASPV = ''
                    self.scan2.ASCD = 0

                self.scan1.P1SM = 0 #Linear
                self.scan2.P1SM = 0 #Linear
                self.scan3.P1SM = 0 #Linear
                self.scan1.P1AR = 1 # Relative
                self.scan2.P1AR = 1 # Relative
                self.scan3.P1AR = 1 # Relative
                self.scan1.PASM = 2 # Prior Pos
                self.scan2.PASM = 2 # Prior Pos
                self.scan3.PASM = 2 # Prior Pos
                self.scan1.PDLY = 0
                self.scan2.PDLY = 0
                self.scan3.PDLY = 0

            #self.set_scan_type()
            #self.set_scan_axes()
            #self.thinking.put(1)
            self.scan2.T1PV = self.scan1.PV('EXSC').pvname

            # If XANES set energy settling time to 0.1
            if EIO.interface_PVs[iocprefix+sxm_prefix+'scan_axes_select.VAL'].get() == 6:
                self.scan1.PDLY = 0.1

            if self.scand == 2:
                self.scan2.EXSC = 1

            elif self.scand == 3:
                self.scan3.T1PV = self.scan2.PV('EXSC').pvname
                self.scan3.EXSC = 1
            self.thinking.put(0)

    def set_scan_type(self, *args, **kwargs):

        self.thinking.put(1)
        for i in range(4):
            setattr(getattr(self, 'scan1'), 'T%dPV'%(i+1), '')
        for i in range(70):
            setattr(getattr(self, 'scan1'), 'D%0.2dPV'%(i+1), '')

        self.scan1.D01PV = 'S:SRcurrentAI'
        self.scan1.D02PV = '2idb1:scaler1.S2'
        self.scan1.D03PV = '2idb1:scaler1.S3'
        self.scan1.D04PV = '2idb1:scaler1.S4'
        self.scan1.D05PV = '2idb1:scaler1.S5'
        self.scan1.D06PV = '2idb1:scaler1.T'

        try:
            scan_type = args[1]['value']
        except KeyError:
            scan_type = EIO.interface_PVs[iocprefix+sxm_prefix+'scan_type_select.VAL'].get()


        if scan_type == 0: #stxm
            self.scan1.T1PV = '2idb1:scaler1.CNT'

        if scan_type == 1: # ccd
            self.scan1.T1PV = '2idb1:scaler1.CNT'
            self.scan1.T2PV = camprefix+'.AcquireCLBK'

        if scan_type == 2: #dpc
            self.scan1.T1PV = '2idb1:scaler1.CNT'
            self.scan1.T2PV = '2idb1:scanH.EXSC'

            self.scan1.D11PV = '2idb1:IP330_1.VAL'
            self.scan1.D12PV = '2idb1:IP330_2.VAL'
            self.scan1.D13PV = '2idb1:IP330_3.VAL'
            self.scan1.D14PV = '2idb1:IP330_4.VAL'
            self.scan1.D15PV = '2idb1:IP330_5.VAL'
            self.scan1.D16PV = '2idb1:IP330_6.VAL'
            self.scan1.D17PV = '2idb1:IP330_7.VAL'
            self.scan1.D18PV = '2idb1:IP330_8.VAL'
            self.scan1.D19PV = '2idb1:IP330_9.VAL'
            self.scan1.D20PV = '2idb1:IP330_10.VAL'
            self.scan1.D31PV = '2idb1:IP330_11.VAL'

            self.scan1.D21PV = '2idb1:scaler1.S6'
            self.scan1.D22PV = '2idb1:scaler1.S7'
            self.scan1.D23PV = '2idb1:scaler1.S8'
            self.scan1.D24PV = '2idb1:scaler1.S9'
            self.scan1.D25PV = '2idb1:scaler1.S10'
            self.scan1.D26PV = '2idb1:scaler1.S11'
            self.scan1.D27PV = '2idb1:scaler1.S12'
            self.scan1.D28PV = '2idb1:scaler1.S13'
            self.scan1.D29PV = '2idb1:scaler1.S14'
            self.scan1.D30PV = '2idb1:scaler1.S15'

        if scan_type == 3: #xfm
            self.scanH.reset()

            self.scan1.T1PV = '2idb1:scaler1.CNT'
            self.scan1.T2PV = '2idb1:scanH.EXSC'
            #self.scan1.T3PV = camprefix+'Acquire'
            #self.scan1.T2PV = XFD trigger PV

            self.scanH.D01PV = xfdprefix+'mca1.VAL' # MCA Spectrum
            self.scanH.NPTS = 1024
            self.scanH.T1PV = xfdprefix+'mca1EraseStart'

            self.scan1.D07PV = xfdprefix+'mca1.ERTM'
            self.scan1.D08PV = xfdprefix+'mca1.ELTM'
            self.scan1.D09PV = xfdprefix+'mca1.R0'
            self.scan1.D10PV = xfdprefix+'mca1.R1'
            self.scan1.D11PV = xfdprefix+'mca1.R2'
            self.scan1.D12PV = xfdprefix+'mca1.R3'
            self.scan1.D13PV = xfdprefix+'mca1.R4'
            self.scan1.D14PV = xfdprefix+'mca1.R5'
            self.scan1.D15PV = xfdprefix+'mca1.R6'
            self.scan1.D16PV = xfdprefix+'mca1.R7'
            self.scan1.D17PV = xfdprefix+'mca1.R8'
            self.scan1.D18PV = xfdprefix+'mca1.R9'
            #self.scan1.D19PV = xfdprefix+'mca1.FAST_PEAKS'
            #self.scan1.D20PV = xfdprefix+'mca1.SLOW_PEAKS'
            self.scan1.D21PV = '2idb1:scaler1.S6'
            self.scan1.D22PV = '2idb1:scaler1.S7'
            self.scan1.D23PV = '2idb1:scaler1.S8'
            self.scan1.D24PV = '2idb1:scaler1.S9'
            self.scan1.D25PV = '2idb1:scaler1.S10'
            self.scan1.D26PV = '2idb1:scaler1.S11'
            self.scan1.D27PV = '2idb1:scaler1.S12'
            self.scan1.D28PV = '2idb1:scaler1.S13'
            self.scan1.D29PV = '2idb1:scaler1.S14'
            self.scan1.D30PV = '2idb1:scaler1.S15'

        self.thinking.put(0)

    def toggle_autoshutter_state(self, *args, **kwargs):

        if args[1]['value'] == 1:
            self.thinking.put(1)
            self.autoshutter.state.put(1-self.autoshutter.state.get())
            self.thinking.put(0)

    """
    def toggle_stepfly_state(self, pvname, value, **kwargs):
        if value==0:
            self.stepfly = 'step'
        else:
            self.stepfly = 'fly'
            self.select_scan_axes(None, 0) # Sample XY
            self.c_pvs['2xfmS1:scan_axes_select.VAL'].put(0)


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

    def stage_stack_move(self, pvname, value, **kwargs):
        if value>0:
            stack, location = self.move_define_axes[value]

            for axis in getattr(self, stack).axes:
                if axis not in ['top', 'middle', 'theta']:
                    getattr(getattr(self, stack), axis).move(epics.caget(self.soft_prefix+'{:s}_{:s}_{:s}.VAL'.format(stack, axis, location)))

    def stage_stack_define(self, pvname, value, **kwargs):
        if value>0:
            stack, location = self.move_define_axes[value]

            for axis in getattr(self, stack).axes:
                if axis not in ['top', 'middle', 'theta']:
                    epics.caput(self.soft_prefix+'{:s}_{:s}_{:s}.VAL'.format(stack, axis, location), getattr(getattr(self, stack), axis).VAL )

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
    sxm.run()
    print('Ready.')
    ipdb.set_trace()
    task_queue.join()

if __name__ == '__main__':
    mainloop()

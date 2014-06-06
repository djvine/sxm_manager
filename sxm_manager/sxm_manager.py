#!/home/beams/USER2IDB/python/bin/python
import epics
from epics.devices import scan, scaler
from epics.motor import MotorException
import time
import multiprocessing as mp
import threading
import scipy as sp
import ipdb
import settings.djv

class StageStack(epics.Device):

    def __init__(self,**kwargs):

        "keywords should be name=epics_motor_pv"

        for key in kwargs:
            try:
                setattr(self, key, epics.Motor(kwargs[key]))
            except MotorException:
                setattr(self, key, epics.PV(kwargs[key]))

class SXM_Manager(mp.Process):

    def __init__(self, task_queue):
        mp.Process.__init__(self)
        self.task_queue = task_queue

        self.soft_prefix = settings.soft_prefix
        self.ioc_prefix = settings.ioc_prefix
        self.xfd_prefix = settings.xfd_prefix
        self.cam_prefix = settings.cam_prefix

        # define scan records
        for sc in settings.scan_records:
            setattr(self, sc, scan.Scan(self.ioc_prefix+sc))

        # define scalers
        for sc in settings.scalers:
            setattr(self, sc, scaler.Scaler(self.ioc_prefix+sc))

        # define stage stacks
        for stack in setting.stage_stacks.keys():
            setattr(self, stack, StageStack())
            for stage, pvname in settings.strage_stacks[stack].itertitems():
                setattr(getattr(self, stack), stage, pvname)

        # add other PVs
        for pv, value in settings.pvs.iteritems():
            setattr(self, pv, value)

        # Define MEDM/EPICS interfaces
        self.scan_axis_1_name = epics.PV(self.soft_prefix+'scan_axis_name_1')
        self.scan_axis_2_name = epics.PV(self.soft_prefix+'scan_axis_name_2')
        self.abort_scan = epics.PV(self.ioc_prefix+'AbortScans.PROC')
        self.thinking = epics.PV(self.soft_prefix+'thinking.VAL')

        # Define common scan axes
        self.scan_axes = settings.scan_axes

        self.callbacks = settings.callbacks

    def run(self):
        while True:
            next_task = self.task_queue.get()
            if next_task is None:
                print '{:s}: Exiting'.format(self.name)
                self.task_queue.task_done()
                break
            else:
                pvname, value = next_task
                self.callback[pvname](pvname=pvname, value=value)
                self.task_queue.task_done()
        return

    def on_changes(self, pvname=None, value=None, **kw):
        self.task_queue.put([pvname, value])

    @staticmethod
    def heartbeat_wait(task_queue, pvname, value):
       task_queue.put([pvname, value])

    def toggle_heartbeat(self, value=None, **kws):
        pv.put(1-value)
        t = Threading.thread(target=heartbeat_wait, args=(self.task_queue,pvname,1-value))
        t.start()
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

    def set_scan_axes(self, *args, **kwargs):
        self.thinking.put(1)

        for key in self.scan_axes.keys():
            try:
                if self.scan_axes[key].value == args[1]['value']:
                    axes = self.scan_axes[key].pvs
            except KeyError:
                if self.scan_axes[key].value == EIO.interface_PVs[iocprefix+sxm_prefix+'scan_axes_select.VAL'].get():
                    axes = self.scan_axes[key].pvs

        for i, axis_pv in enumerate(axes):
            try:
                setattr(getattr(self, 'scan%d'%(i+1)), 'P1PV', axis_pv.PV('VAL').pvname)
                setattr(getattr(self,  'scan_axis_%d_name'%(i+1)),'value', axis_pv.PV('DESC').value)
            except KeyError:
                self.log.error('Setting scan axes failed due to unknown scan type')
            except AttributeError:

                if axis_pv.pvname == '2idb1:MCL:s1:X_pos':
                    setattr(getattr(self, 'scan%d'%(i+1)), 'P1PV', axis_pv.pvname+'.VAL')
                    setattr(getattr(self,  'scan_axis_%d_name'%(i+1)),'value', 'Fine X')
                elif axis_pv.pvname == '2idb1:MCL:s1:Y_pos':
                    setattr(getattr(self, 'scan%d'%(i+1)), 'P1PV', axis_pv.pvname+'.VAL')
                    setattr(getattr(self,  'scan_axis_%d_name'%(i+1)),'value', 'Fine Y')
                elif axis_pv.pvname == '2idb0:userTran1.A':
                    setattr(getattr(self, 'scan%d'%(i+1)), 'P1PV', axis_pv.pvname)
                    setattr(getattr(self,  'scan_axis_%d_name'%(i+1)),'value', 'Tandem Energy')
        self.scan2.T1PV = self.scan1.PV('EXSC').pvname
        self.scan1.P1CP = 0.
        self.scan2.P1CP = 0.
        self.scand = 2
        if i==2:
            self.scand = 3
            self.scan3.T1PV = self.scan2.PV('EXSC').pvname
            self.scan3.P1CP=0.
        self.thinking.put(0)
        """

def mainloop():
    try:
        task_queue = mp.Joinable_Queue()
        sxm = SXM_Manager(task_queue)
        sxm.start()
    except KeyboardInterrupt:
        task_queue.put(None)
        task_queue.join()
    print('Mainloop Exiting')

if __name__ == '__main__':
    mainloop()

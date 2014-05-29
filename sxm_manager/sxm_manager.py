#!/home/beams/USER2IDB/python/bin/python
import epics
from epics.devices import scan, scaler
from epics.motor import MotorException
import time
import threading
import Pyro4
import scipy as sp
import ipdb
import settings.2xfm

class Message(object):

    def __init__(self, message_pv):
        self.message_pv = message_pv

    def thread_func(self, message):

        epics.caput(self.message_pv, message)
        time.sleep(1)
        epics.caput(self.message_pv, '')


    def put(self, message):
        threading.Thread(target=self.thread_func, args=(message,)).start()

class StageStack(epics.Device):

    def __init__(self,**kwargs):

        "keywords should be name=epics_motor_pv"

        for key in kwargs:
            try:
                setattr(self, key, epics.Motor(kwargs[key]))
            except MotorException:
                setattr(self, key, epics.PV(kwargs[key]))

class Shutter(epics.Device):

    def __init__(self, shutter_pv):
        self._shutter = epics.PV(shutter_pv)
        self.open_state = 1
        self.close_state = 0

    def make_open(self):
        self._shutter.put(self.open_state)

    def make_close(self):
        self._shutter.put(self.close_state)

    def toggle(self):
        self._shutter.put(1-self._shutter.get())

def in_thread(func):
    def in_thread2(self=None, *args, **kwargs):
        threading.Thread(target=func, args=(self,args,kwargs)).start()
    return in_thread2


class EIO(object): # EPICS interface object

    """
    This class provides an object oriented approach to interfacing with EPICS/MEDM.

    The typical workflow involves pressing a button on a MEDM screen and having the daemon
    process the callback and run the appropriate handler.

    Each EIO has:
        * an epics control PV (to which will be associated a callback)
        * a value for which this EIOP instance is selected
        * a handler function

    Example: Setting up a specific scan type

    coarse_xy = EIO( cPV = iocprefix+'scan_type_select', value = 0, name='coarse_xy', handler = set_scan_type )
    """

    interface_PVs = {}
    EIOs = {}

    def __init__(self, **kwargs):

        if kwargs['cPV'] is not None:
            if kwargs['cPV'] not in EIO.interface_PVs.keys():
                print 'New interface PV', kwargs['cPV']
                EIO.interface_PVs[kwargs['cPV']] = epics.PV(kwargs['cPV'], connection_timeout = 0.1,
                                                            callback = kwargs['handler'])
            else:
                l=[]
                for handler_tuple in EIO.interface_PVs.values():
                    try:
                        l.append(handler_tuple.callbacks[0][0]==kwargs['handler'])
                    except:
                        pass
                if not any(l):
                    print 'Found interface PV without callback'
                    EIO.interface_PVs[kwargs['cPV']].callback(kwargs['handler'])
        print 'Adding new EIO', kwargs['name']
        EIO.EIOs[kwargs['name']] = (kwargs['value'], kwargs['cPV'])

        for key in kwargs.keys():
            print 'adding attr to', kwargs['name']
            setattr(self, key, kwargs[key])

class ScanAxes(EIO):

    def __init__(self, **kwargs):

        super(ScanAxes, self).__init__(cPV=iocprefix+sxm_prefix+'scan_axes_select.VAL', **kwargs)

class ScanType(EIO):

    def __init__(self, **kwargs):

        super(ScanType, self).__init__(cPV=iocprefix+sxm_prefix+'scan_type_select.VAL', **kwargs)

class AutoShutter(EIO):

    def __init__(self, **kwargs):

        super(AutoShutter, self).__init__(cPV = iocprefix+sxm_prefix+'autoshutter', **kwargs)

class BeginScan(EIO):

    def __init__(self, **kwargs):

        super(BeginScan, self).__init__(cPV = iocprefix+sxm_prefix+'begin_scan', **kwargs)

class HeartBeat(EIO):
    def __init__(self, **kwargs):

        super(HeartBeat, self).__init__(cPV = iocprefix+sxm_prefix+'heartbeat', **kwargs)

class SXM_Manager(object):

    def __init__(self):

        self.sxm_prefix = settings.soft_prefix
        self.ioc_prefix = settings.ioc_prefix
        self.xfd_prefix = settings.xfd_prefix
        self.cam_prefix = settings.cam_prefix

        # define scan records
        for sc in settings.scan_records:
            setattr(self, sc, scan.Scan(self.ioc_prefix+sc))

        # define scalers
        for sc in settings.scalers:
            setattr(self, sc, scaler.Scaler(self.ioc_prefix+sc))

        # define stages
        if DEBUG:
            self.sample = StageStack(x = iocprefix+'m1', y = iocprefix+'m2', z = iocprefix+'m3')
            self.zp = StageStack(x = iocprefix+'m1', y = iocprefix+'m2', z = iocprefix+'m3')
            self.osa = StageStack(x = iocprefix+'m1', y = iocprefix+'m2', z = iocprefix+'m3')
            self.xfd = StageStack(x = iocprefix+'m1', y = iocprefix+'m2', z = iocprefix+'m3')
            self.vlm = StageStack(x = iocprefix+'m1', y = iocprefix+'m2', z = iocprefix+'m3')
            self.ccd = StageStack(x = iocprefix+'m1', y = iocprefix+'m2', z = iocprefix+'m3')

        else:
            self.sample = StageStack(x = iocprefix+'m13', y = iocprefix+'m14', theta = iocprefix+'m12',
                     fine_x = iocprefix+'MCL:s1:X_pos', fine_y = iocprefix+'MCL:s1:Y_pos')
            self.zp = StageStack(x = iocprefix+'m37', y = iocprefix+'m38', z = iocprefix+'m3')
            self.osa = StageStack(x = iocprefix+'m39', y = iocprefix+'m40', z = iocprefix+'m4')
            self.xfd = StageStack(x = iocprefix+'m7', y = iocprefix+'m8', z = iocprefix+'m9')
            self.vlm = StageStack(x = iocprefix+'m19', y = iocprefix+'m20', z = iocprefix+'m21')
            self.ccd = StageStack(x = iocprefix+'m22', y = iocprefix+'m23', z = iocprefix+'m32')
            self.tandem_energy = epics.PV('2idb0:userTran1.A')
            self.sgm_energy = epics.PV('2idb0:userTran1.B')
            self.U55_energy = epics.PV('2idb0userTran1.I')

        # Define MEDM/EPICS interfaces
        self.message = Message(iocprefix+sxm_prefix+'message')
        self.scan_axis_1_name = epics.PV(iocprefix+sxm_prefix+'scan_axis_1_name')
        self.scan_axis_2_name = epics.PV(iocprefix+sxm_prefix+'scan_axis_2_name')
        self.scan_axis_3_name = epics.PV(iocprefix+sxm_prefix+'scan_axis_3_name')
        self.abort_scan = epics.PV('2idb1:AbortScans.PROC')
        self.thinking = epics.PV(iocprefix+sxm_prefix+'thinking.VAL')
        self.heartbeat = HeartBeat(name = 'heartbeat', value = 0,
                                   handler = self.toggle_heartbeat)
        # Define common scan axes
        self.scan_axes = {}
        self.scan_axes['coarse_xy'] = ScanAxes(name='coarse_xy',    value = 0,
                                  pvs = (self.sample.x,self.sample.y),
                                  handler = self.set_scan_axes)
        self.scan_axes['coarse_xz'] = ScanAxes(name='coarse_xz',    value = 1,
                                  pvs = (self.sample.x, self.zp.z),
                                  handler = self.set_scan_axes)
        self.scan_axes['coarse_yz'] = ScanAxes(name='coarse_yz',    value = 2,
                                  pvs = (self.sample.y, self.zp.z),
                                  handler = self.set_scan_axes)
        self.scan_axes['fine_xy'] = ScanAxes(name='fine_xy',        value = 3,
                                  pvs = (self.sample.fine_x,self.sample.fine_y),
                                  handler = self.set_scan_axes)
        self.scan_axes['focus_xz'] = ScanAxes(name='focus_xz',      value = 4,
                                  pvs = (self.sample.fine_x,self.zp.z),
                                  handler = self.set_scan_axes)
        self.scan_axes['focus_yz'] = ScanAxes(name='focus_yz',      value = 5,
                                  pvs = (self.sample.fine_y,self.zp.z),
                                  handler = self.set_scan_axes)
        self.scan_axes['xanes'] = ScanAxes(name='xanes',            value = 6,
                                  pvs = (self.tandem_energy, self.ccd.x),
                                  handler = self.set_scan_axes)
        self.scan_axes['fine_tomo'] = ScanAxes(name='fine_tomo',    value = 7,
                                  pvs = (self.sample.fine_x, self.sample.fine_y, self.sample.theta),
                                  handler = self.set_scan_axes)
        self.scan_axes['coarse_tomo'] = ScanAxes(name='coarse_tomo', value = 8,
                                  pvs = (self.sample.x, self.sample.y, self.sample.theta),
                                  handler = self.set_scan_axes)
        self.scand = None


        # Define scan types
        self.scan_type = {}
        self.scan_type['stxm'] = ScanType(name='stxm', value = 0, handler = self.set_scan_type)
        self.scan_type['ccd'] = ScanType(name='ccd', value = 1, handler = self.set_scan_type)
        self.scan_type['dpc'] = ScanType(name='dpc', value = 2, handler = self.set_scan_type)
        self.scan_type['xfm'] = ScanType(name='xfm', value = 3, handler = self.set_scan_type)

        # Define autoshuttering
        self.autoshutter = AutoShutter(name='autoshutter', value = 0,
                                       handler = self.toggle_autoshutter_state,
                                       state = epics.PV(iocprefix+sxm_prefix+'autoshutter_state',
                                                           connection_timeout=0.1)
                                       )

        self.begin_scan = BeginScan(name='begin_scan', value = 0,
                                    handler = self.do_scan)


    def close(self):
        SXM_Manager.instances = {}

    def kill_daemon(self):
        del(SXM_Manager.instances[SXM_Manager.instances.keys()[0]])

    @in_thread
    def toggle_heartbeat(self, *args, **kwargs):
        if args[1]['value'] == 0:
            time.sleep(5.0)
            EIO.interface_PVs[iocprefix+sxm_prefix+'heartbeat'].put(1)

    @in_thread
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

    @in_thread
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

    @in_thread
    def toggle_autoshutter_state(self, *args, **kwargs):

        if args[1]['value'] == 1:
            self.thinking.put(1)
            self.autoshutter.state.put(1-self.autoshutter.state.get())
            self.thinking.put(0)

    @in_thread
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

def start_server():
    sxm = SXM_Manager()

    daemon = Pyro4.Daemon()
    ns = Pyro4.locateNS()
    uri = daemon.register(sxm)
    ns.register('sxm', uri)
    print "Ready."
    daemon.requestLoop()

def start_client():
    sxm = Pyro4.Proxy("PYRONAME:sxm")



if __name__ == '__main__':
    #s=SXM_Manager()

    start_server()

  # python -m Pyro4.naming








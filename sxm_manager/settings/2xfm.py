
import settings.common

ioc_prefix = '2xfm:'
soft_prefix = 'SXM:'
cam_prefix = 'MMPAD3x2:cam1:'
xfd_prefix = 'dxpXMAP2xfm3'

scan_records = ['scanH','scan1','scan2','scan3','FscanH','Fscan1','Fscan2','Fscan3']
scalers = []
stage_stacks = {
        'sample': {
            'x': PV,
            'y': PV,
            'z': PV,
            'top': PV,
            'middle': PV,
            'theta': PV,
            }
        },
        'zp20': {
                'x': PV

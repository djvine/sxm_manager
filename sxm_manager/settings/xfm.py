
from sxm_manager.settings import common

soft_prefix = '2xfmS1:'
ioc_prefix = '2xfm:'
cam_prefix = 'MMPAD3x2:cam1:'
xfd_prefix = 'dxpXMAP2xfm3'

# Syntax: {pvname: function_name}
# When pvname changes function_name will be called with pvname's value
callbacks = {
        #{soft_prefix+'scan_axes_select': 'select_scan_axes'},
        #{soft_prefix+'scan_type_select': 'select_scan_type'},
        #{soft_prefix+'autoshutter': 'toggle_autoshutter_state'},
        #{soft_prefix+'heartbeat': 'toggle_heartbeat'},
        #{soft_prefix+'stepfly': 'toggle_stepfly_state'},
        #{soft_prefix+'dwell': 'update_dwell'},
        #{soft_prefix+'begin_scan_1d': 'do_1d_scan'},
        #{soft_prefix+'begin_scan_2d': 'do_2d_scan'},
        #{soft_prefix+'pinhole': 'define_pinhole'},
        soft_prefix+'toggle_lock.VAL': 'toggle_lock_state',
        soft_prefix+'heartbeat.VAL': 'toggle_heartbeat',
        #{soft_prefix+'stage_stack_move': 'move_stage_stack'},
        #{soft_prefix+'stage_stack_define': 'define_stage_stack'},
    }

scan_records = ['scanH','scan1','scan2','scan3','FscanH','Fscan1','Fscan2','Fscan3']
scalers = []
stage_stacks = {
        'filter_wheel': {
            'angle': ioc_prefix+'m15',
            'lock_state': soft_prefix+'filter_wheel_locked.VAL',
            },
        'pinhole': {
            'x': ioc_prefix+'m9',
            'y': ioc_prefix+'m10',
            'lock_state': soft_prefix+'pinhole_locked.VAL',
            },
        'zp20': {
            'x': ioc_prefix+'m30',
            'y': ioc_prefix+'m12',
            'z': ioc_prefix+'m29',
            'lock_state': soft_prefix+'zp20_locked.VAL',
            },
        'zp10': {
            'x': ioc_prefix+'m17',
            'y': ioc_prefix+'m22',
            'z': ioc_prefix+'m20',
            'lock_state': soft_prefix+'zp10_locked.VAL',
            },
        'osa': {
            'x': ioc_prefix+'m18',
            'y': ioc_prefix+'m19',
            'lock_state': soft_prefix+'osa_locked.VAL',
            },
        'sample': {
            'x': ioc_prefix+'m24',
            'y': ioc_prefix+'m13',
            'z': ioc_prefix+'m23',
            'top': ioc_prefix+'m4',
            'middle': ioc_prefix+'m5',
            'theta': ioc_prefix+'m6',
            'lock_state': soft_prefix+'sample_locked.VAL',
            },
        'tx_det': {
            'x': ioc_prefix+'m31',
            'y': ioc_prefix+'m32',
            'lock_state': soft_prefix+'tx_det_locked.VAL',
        }
        }

scan_axes = {
    0: ['sample.x', 'sample.y'],
    1: ['sample.x', 'zp20.z'],
    2: ['sample.y', 'zp20.z'],
    3: ['sample.x', 'zp10.z'],
    4: ['sample.y', 'zp10.z'],
    5: ['tx_det.x', 'tx_det.y'],
    6: ['osa.x', 'osa.y'],
    }

lock_state = {
        1: 'filter_wheel',
        2: 'pinhole',
        3: 'zp20',
        4: 'zp10',
        5: 'osa',
        6: 'sample',
        7: 'tx_det'
        }


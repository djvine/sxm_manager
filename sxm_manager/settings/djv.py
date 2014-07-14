
import settings.common

soft_prefix = 'djvS1:'
ioc_prefix = 'djv:'
cam_prefix = 'MMPAD3x2:cam1:'
xfd_prefix = 'dxpXMAP2xfm3'

# Syntax: {pvname: function_name}
# When pvname changes function_name will be called with pvname's value
callbacks = [
        {soft_prefix+'scan_axes_select': 'select_scan_axes'},
        {soft_prefix+'scan_type_select': 'select_scan_type'},
        {soft_prefix+'autoshutter': 'toggle_autoshutter_state'},
        {soft_prefix+'heartbeat': 'toggle_heartbeat'},
        {soft_prefix+'stepfly': 'toggle_stepfly_state'},
        {soft_prefix+'dwell': 'update_dwell'},
        {soft_prefix+'begin_scan_1d': 'do_1d_scan'},
        {soft_prefix+'begin_scan_2d': 'do_2d_scan'},
        {soft_prefix+'pinhole': 'define_pinhole'},
        {soft_prefix+'toggle_lock': 'toggle_lock_state'},
        {soft_prefix+'stage_stack_move': 'move_stage_stack'},
        {soft_prefix+'stage_stack_define': 'define_stage_stack'},
    ]

scan_records = ['scanH','scan1','scan2','scan3','FscanH','Fscan1','Fscan2','Fscan3']
scalers = []
stage_stacks = {
        'filter_wheel': {
            'angle': ioc_prefix+'m15',
            },
        'pinhole': {
            'x': ioc_prefix+'m1',
            'y': ioc_prefix+'m2',
            },
        'zp20': {
            'x': ioc_prefix+'m1',
            'y': ioc_prefix+'m2',
            'z': ioc_prefix+'m3',
            },
        'zp10': {
            'x': ioc_prefix+'m1',
            'y': ioc_prefix+'m2',
            'z': ioc_prefix+'m3',
            },
        'osa': {
            'x': ioc_prefix+'m1',
            'y': ioc_prefix+'m2',
            },
        'sample': {
            'x': ioc_prefix+'m1',
            'y': ioc_prefix+'m2',
            'z': ioc_prefix+'m3',
            'top': ioc_prefix+'m4',
            'middle': ioc_prefix+'m5',
            'theta': ioc_prefix+'m6',
            },
        'tx_det': {
            'x': ioc_prefix+'m1',
            'y': ioc_prefix+'m2',
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



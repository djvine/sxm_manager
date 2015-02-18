
from sxm_manager.settings import common

soft_prefix = '2xfmS1:'
ioc_prefix = '2xfm:'
cam_prefix = 'MMPAD3x2:cam1:'
xfd_prefix = 'dxpXMAP2xfm3:'

# Syntax: {pvname: function_name}
# When pvname changes function_name will be called with pvname's value
callbacks = {
        soft_prefix+'scan_axes_select.VAL': 'select_scan_axes',
        soft_prefix+'scan_type_select.VAL': 'select_scan_type',
        soft_prefix+'stepfly.VAL': 'toggle_stepfly_state',
        soft_prefix+'dwell.VAL': 'update_dwell',
        ioc_prefix+'userTran1.P': 'update_dwell',
        ioc_prefix+'FlySetup:DwellTime.VAL': 'update_dwell',
        soft_prefix+'begin_scan_1d.VAL': 'do_1d_scan',
        soft_prefix+'begin_scan_2d.VAL': 'do_2d_scan',
        soft_prefix+'pinhole_define.VAL': 'pinhole_define',
        soft_prefix+'pinhole_move.VAL': 'pinhole_move',
        soft_prefix+'toggle_lock.VAL': 'toggle_lock_state',
        soft_prefix+'heartbeat.VAL': 'toggle_heartbeat',
        soft_prefix+'stage_stack_move.VAL': 'stage_stack_move',
        soft_prefix+'stage_stack_define.VAL': 'stage_stack_define',
        soft_prefix+'alignment.VAL': 'alignment_mode',
        soft_prefix+'to_batch.VAL': 'push_to_batch',
	    soft_prefix+'generate_config.VAL': 'generate_config',
	    soft_prefix+'reprocess_all.VAL': 'maps_reprocess_all',
	    soft_prefix+'process_now.VAL': 'maps_process_now',
	    soft_prefix+'setup_user_dirs.VAL': 'generate_user_dirs',
        soft_prefix+'update_user.VAL': 'update_user',
        ioc_prefix+'scan2.EXSC': 'scan_ends_process_now',
        ioc_prefix+'Fscan1.EXSC': 'scan_ends_process_now',
        soft_prefix+'take_standards.VAL': 'take_standards',
        soft_prefix+'zp1_state.VAL': 'zp_stack_state_change',
        soft_prefix+'zp2_state.VAL': 'zp_stack_state_change',
        soft_prefix+'zp3_state.VAL': 'zp_stack_state_change',
        soft_prefix+'zp4_state.VAL': 'zp_stack_state_change',
        soft_prefix+'zp_config_move.VAL': 'zp_stack_in_out',
    }

scan_records = ['scanH','scan1','scan2','scan3','FscanH','Fscan1','Fscan2','Fscan3']

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
        'zp_stack': {
            'x': ioc_prefix+'m30',
            'y': ioc_prefix+'m12',
            'z': ioc_prefix+'m29',
            'lock_state': soft_prefix+'zp_stack_locked.VAL',
            },
        'zp1': {
            'x': ioc_prefix+'m56',
            'y': ioc_prefix+'m57',
            'z': ioc_prefix+'m58',
            'lock_state': soft_prefix+'zp1_locked.VAL',
            },
        'zp2': {
            'x': ioc_prefix+'m59',
            'y': ioc_prefix+'m60',
            'z': ioc_prefix+'m61',
            'lock_state': soft_prefix+'zp2_locked.VAL',
            },
        'zp3': {
            'x': ioc_prefix+'m62',
            'y': ioc_prefix+'m63',
            'z': ioc_prefix+'m64',
            'lock_state': soft_prefix+'zp3_locked.VAL',
            },
        'zp4': {
            'x': ioc_prefix+'m65',
            'y': ioc_prefix+'m66',
            'z': ioc_prefix+'m67',
            'lock_state': soft_prefix+'zp4_locked.VAL',
            },
        'osa': {
            'x': ioc_prefix+'m18',
            'y': ioc_prefix+'m19',
            'lock_state': soft_prefix+'osa_locked.VAL',
            },
        'sample': {
            'x': ioc_prefix+'m200',
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
    1: ['sample.y', 'sample.x'],
    2: ['sample.x', 'zp_stack.z'],
    3: ['sample.y', 'zp_stack.z'],
    4: ['sample.x', 'zp1.z'],
    5: ['sample.y', 'zp1.z'],
    6: ['sample.x', 'zp2.z'],
    7: ['sample.y', 'zp2.z'],
    8: ['sample.x', 'zp3.z'],
    9: ['sample.y', 'zp3.z'],
    10: ['sample.x', 'zp4.z'],
    11: ['sample.y', 'zp4.z'],
    12: ['tx_det.x', 'tx_det.y'],
    13: ['tx_det.y', 'tx_det.x'],
    14: ['osa.x', 'osa.y'],
    }

lock_state = {
        1: 'filter_wheel',
        2: 'pinhole',
        3: 'zp_stack',
        4: 'zp1',
        5: 'zp2',
        6: 'zp3',
        7: 'zp4',
        8: 'osa',
        9: 'sample',
        10: 'tx_det'
        }

move_define_axes = {
         1: ['zp1', '1'],
         2: ['zp1', '2'],
         3: ['zp1', '3'],
         4: ['zp1', '4'],
         5: ['zp1', 'out'],
         6: ['zp2', '1'],
         7: ['zp2', '2'],
         8: ['zp2', '3'],
         9: ['zp2', '4'],
         10: ['zp2', 'out'],
         11: ['zp3', '1'],
         12: ['zp3', '2'],
         13: ['zp3', '3'],
         14: ['zp3', '4'],
         15: ['zp3', 'out'],
         16: ['zp4', '1'],
         17: ['zp4', '2'],
         18: ['zp4', 'out'],
         19: ['zp_stack', '1'],
         20: ['zp_stack', '2'],
         21: ['zp_stack', '3'],
         22: ['zp_stack', '4'],
         23: ['zp_stack', 'out'],
         24: ['osa', 'in'],
         25: ['osa', 'out'],
         26: ['sample', '0'],
         27: ['sample', '1'],
         28: ['sample', '2'],
         29: ['sample', '3'],
         30: ['sample', '4'],
         31: ['tx_det', '1'], # CCD
         32: ['tx_det', '2'], # CFG
         33: ['tx_det', '3'], # Ptycho
        }

active_zp = {
    # Syntax axis: move_define_axis value in, out
    1: [1, 6, 11],
    2: [2, 7, 12],
    3: [3, 8, 13],
    4: [4, 9, 14],
    5: [16],
    6: [17],
    'all_out': [5, 10, 15, 18],
}

# Scan estimate overhead - used to accurately estimate scan time
time_estimate_overhead = {
        #       'mode': [constant per pt, multiplier per pt, constant per line, multiplier per line]
        'step': [9.4406, 1.784, 0.0, 1.1],
        'fly': [0.1, 1.05, 0.0, 1.1]
        }

#!/usr/bin/env python2

# This script automates the submission of perpixel fitting of completed experiments to xfm0


import glob
import os
import datetime as dt
import shutil

maps_job = """
This file will set some MAPS settings mostly do do with fitting.\r
DIRECTORY:Y:\data\\{station:s}\{run:s}\{user:s}\r
A: {A:s}\r
B: {B:s}\r
C: {C:s}\r
D: {D:s}\r
E: {E:s}\r
F: {F:s}\r
VERSION:      1.00000\r
DETECTOR_ELEMENTS:       4\r
MAX_NUMBER_OF_PROCESSORS_TO_USE:      3\r
QUICK_DIRTY: 0\r
XRF_BIN:     0\r
NNLS: 0\r
XANES_SCAN: 0\r
DETECTOR_TO_START_WITH: 0\r
COMPUTER_TO_USE:\r
"""

older_than = 14
more_recent_than = 90

watch_directory = '/mnt/xfm0/data/2ide/*/*/'
jobs_dir = '/mnt/xfm0/data/jobs'

for fn in glob.glob(watch_directory+'livejob*'):

    # check modification time of livejob file
    mtime = dt.datetime.fromtimestamp(os.path.getmtime(fn))
    now = dt.datetime.now()
    if mtime>now-dt.timedelta(days=more_recent_than) and mtime < now-dt.timedelta(days=older_than):
        print(fn)
        livejob_filename = os.path.basename(fn)
        job_filename = livejob_filename.strip('live')
        job = livejob_filename.split('_')[0]
        run = livejob_filename.split('_')[1]
        user = '_'.join(livejob_filename.split('_')[2:])
        processing_vars = {'A':'0', 'B':'0', 'C':'0', 'D':'0', 'E':'0', 'F':'0'}
        processing_vars['user'] = user
        processing_vars['run'] = run
        processing_vars['station'] = station

        # check if dataset is currently being processed
        if os.path.exists(os.path.join(jobs_dir, 'processing', job_filename)):
            print('{:s} is currently being processed'.format(fn))
            continue

        # check if processing is completed
        elif os.path.exists(os.path.join(jobs_dir, 'done', job_filename)):
            # Job is found in done directory
            with open(os.path.join(jobs_dir, 'done', job_filename), 'r') as job_file:
                contents = job_file.readlines()
                for line in contents:
                    for n, element in enumerate(processing_vars.keys()):
                        if line.startswith(element):
                            for c in line:
                                if c.isdigit():
                                    processing_vars[element] = c

            # Has dataset been processed through stage 1?
            if processing_vars['A']=='1' and processing_vars['B']=='1':
                processing_vars['C'] = '1'
                processing_vars['E'] = '1'
                shutil.remove(os.path.join(jobs_dir, 'done', job_filename))
            elif processing_vars['C']=='1':
                continue

        else: # Processing hasn't begun
            processing_vars['A'] = '1'
            processing_vars['B'] = '1'

        with open(os.path.join(jobs_dir, job_filename), 'w') as job_file:
            job_file.writelines(maps_job.format(**processing_vars))


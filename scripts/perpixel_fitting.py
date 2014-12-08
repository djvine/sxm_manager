#!/usr/bin/env python2

# This script automates the submission of perpixel fitting of completed experiments to xfm0


import glob
import os
import datetime as dt
import ipdb
import zipfile
import smtplib
from email.mime.text import MIMEText
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
zip_dir = '/mnt/xfm0/data/{station:s}/{run:s}'
ftp_dir = '/net/ftp/ftp/pub/user2ide/{station:s}/{run:s}'

message = "PERPIXEL FITTING SCHEDULING UPDATE\n\nThe following results pertain to datasets older than {:d} and more recent than {:d}\n\n".format(older_than, more_recent_than)
for fn in glob.glob(watch_directory+'livejob*'):
    print('Starting: {:s}'.format(fn))
    # check modification time of livejob file
    mtime = dt.datetime.fromtimestamp(os.path.getmtime(fn))
    now = dt.datetime.now()
    if mtime>now-dt.timedelta(days=more_recent_than) and mtime < now-dt.timedelta(days=older_than):
        livejob_filename = os.path.basename(fn)
        job_filename = livejob_filename.strip('live')
        job = livejob_filename.split('_')[0]
        run = livejob_filename.split('_')[1]
        station = livejob_filename.split('_')[2]
        user = '_'.join(livejob_filename.strip('.txt').split('_')[3:])
        processing_vars = {'A':'0', 'B':'0', 'C':'0', 'D':'0', 'E':'0', 'F':'0'}
        processing_vars['user'] = user
        processing_vars['run'] = run
        processing_vars['station'] = station
        # Check if a commissioning beamtime
        if user[:4].lower() == 'comm':
            message+="[{run:s} {station:s} {user:s}] Commissioning experiments ignored by default.\n".format(**processing_vars)
            print('Not processing commissioning data.')
            continue

        # check if dataset is currently being processed
        if os.path.exists(os.path.join(jobs_dir, 'processing', job_filename)):
            print('{:s} is currently being processed'.format(fn))
            message+="[{run:s} {station:s} {user:s}] Currently being processed.\n".format(**processing_vars)
            continue
        # check if dataset is currently in queue 
        elif os.path.exists(os.path.join(jobs_dir, job_filename)):
            print('{:s} is currently in queue'.format(fn))
            message+="[{run:s} {station:s} {user:s}] Currently enqueued.\n".format(**processing_vars)
            continue

        # check if processing is completed
        elif os.path.exists(os.path.join(jobs_dir, 'done', job_filename)):
            # Job is found in done directory
            with open(os.path.join(jobs_dir, 'done', job_filename), 'r') as job_file:
                contents = job_file.readlines()
                for line in contents:
                    for element in processing_vars.keys():
                        if line.startswith(element):
                            for c in line:
                                if c.isdigit():
                                    processing_vars[element] = c
            # Has dataset been processed through stage 1?
            if processing_vars['A']=='1' and processing_vars['B']=='1':
                # Is there an axo_std mda file?
                if not os.path.exists('/mnt/xfm0/data/{station:s}/{run:s}/{user:s}/mda/axo_std.mda'):
                    print('Stage 2 processing delayed. Could not find axo_std')
                    message+="[{run:s} {station:s} {user:s}] Stage 2 processing delayed. Could not find axo_std.mda\n".format(**processing_vars)
                    continue

                processing_vars['A'] = '0'
                processing_vars['B'] = '0'
                processing_vars['C'] = '1'
                processing_vars['E'] = '1'
                os.remove(os.path.join(jobs_dir, 'done', job_filename))
                message+="[{run:s} {station:s} {user:s}] Start stage 2 processing.\n".format(**processing_vars)
                print('Start stage 2 processing.')
            elif processing_vars['C']=='1':
                # Processing is completed

                # Has data been zipped?
                if os.path.exists(os.path.join(zip_dir.format(**{'station':station, 'run':run}), '{:s}.zip'.format(user))):
                    # Analysis is completed
                    print('Data analysis completed for this dataset')
                    message+="[{run:s} {station:s} {user:s}] Data analysis complete. Nothing to do.\n".format(**processing_vars)
                else:
                    # Zip the data
                    print('Zipping the data')
                    zf = zipfile.ZipFile(os.path.join(zip_dir.format(**{'station':station, 'run':run}), '{:s}.zip'.format(user)), 'w', zipfile.ZIP_DEFLATED, True)
                    for root, dirs, files in os.walk(os.path.join(zip_dir.format(**{'station':station, 'run':run}), user)):
                        for file in files:
                            zf.write(os.path.join(root, file))
                    zf.close()
                    if not os.path.exists(ftp_dir.format(**processing_vars)):
                        os.makedir(ftp_dir.format(**processing_vars), mode='0777')

                    # Copy to ftp server
                    print('Moving to ftp')
                    shutil.copy(os.path.join(zip_dir.format(**{'station':station, 'run':run}), '{:s}.zip'.format(user)), ftp_dir.format(**processing_vars))
                    message+="[{run:s} {station:s} {user:s}] Created zip file and copied to ftp.\n".format(**processing_vars)
                    continue

        else: # Processing hasn't begun
            processing_vars['A'] = '1'
            processing_vars['B'] = '1'
            message+="[{run:s} {station:s} {user:s}] Start stage 1 processing.\n".format(**processing_vars)
            print('Start stage 1 processing')

        with open(os.path.join(jobs_dir, job_filename), 'w') as job_file:
            job_file.writelines(maps_job.format(**processing_vars))
    else: # Not within time range
        print('Not between {:d} and {:d} days old'.format(more_recent_than, older_than))

msg = MIMEText(message)
sender = 'user2ide@aps.anl.gov'
recipients = ', '.join(['dvine@anl.gov', 'svogt@aps.anl.gov'])
msg['Subject'] = 'Per-Pixel Fitting Update'
msg['From'] = sender
msg['To'] = recipients

s= smtplib.SMTP('localhost')
s.sendmail(sender, recipients, msg.as_string())
s.quit()


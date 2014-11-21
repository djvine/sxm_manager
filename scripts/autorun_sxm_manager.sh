#!/bin/bash
#autorun_sxm_manager.sh
#Ensure that sxm_manager is always running.
export PYEPICS_LIBCA=/APSshare/epics/base-3.14.12.2/lib/linux-x86_64/libca.so
process=start_sxm_manager.sh
makerun=/home/beams/USER2IDE/bin/start_sxm_manager.sh

if ps ax | grep -v grep | grep $process > /dev/null
	then
		echo "SXM Manager already running."
		exit
	else
	echo "Starting SXM Manager."
	$makerun &
	fi
exit

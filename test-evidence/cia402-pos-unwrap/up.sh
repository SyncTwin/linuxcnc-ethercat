#!/bin/bash
# up.sh <cia402 module> <csv|csp> -- fresh RT env; test modules installed under temporary names
set -e
cd ~/unwrap/run
(setsid rtapi_app start > rt_$2.out 2>&1 < /dev/null &)
sleep 1
halcmd loadrt threads name1=servo-thread period1=1000000 fp1=1
halcmd loadusr -W /home/cnc/unwrap/src/src/lcec_conf ethercat-conf.xml
rtapi_app load lcec_unw
rtapi_app load $1 count=1
halcmd -f body_$2.hal
halcmd start

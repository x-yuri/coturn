#!/usr/bin/env bash
set -eu
pkill -USR1 turnserver
sleep 1
tail -n 10000 /var/log/dturn.log \
    | sed -En '/turnports: range_start/,/^[0-9]{4}-[0-9]{2}-[0-9]{2}/ p'

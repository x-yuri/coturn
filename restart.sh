#!/usr/bin/env bash
set -eu
pkill turnserver || true
if [ "${1-}" = remove-log ]; then
    rm /var/log/dturn.log
fi
bin/turnserver -c /etc/turnserver.conf -o -v --simple-log

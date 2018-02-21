#!/usr/bin/env bash
set -eu

run() {
    echo '
        log_user 0
        spawn telnet localhost 5766
        expect >
        send "'$1'\n"
        expect " '$1'\r\n"
        log_user 1
        expect -re {(?n)^> }
    ' | expect | head -n -1
}

echo $(run ps | egrep '\s*Total\s+sessions:' | sed -E 's/[^:]*:\s*//; s/[^0-9]+//')

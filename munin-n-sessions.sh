#!/bin/sh

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

case $1 in
   config)
        cat <<'EOM'
graph_title coturn: n_sessions
graph_vlabel n_sessions
graph_args --lower-limit 0
sessions.label sessions
EOM
        exit 0;;
esac

printf "sessions.value "
echo "$(run ps | egrep '\s*Total\s+sessions:' | sed -E 's/[^:]*:\s*//; s/[^0-9]+//')"

#!/bin/sh

case $1 in
   config)
        cat <<'EOM'
graph_title coturn: n_errors
graph_vlabel n_errors
errors.label errors
errors.type DERIVE
errors.min 0
EOM
        exit 0;;
esac

printf "errors.value "
cat /var/log/turn_n_errors 2>/dev/null || echo 0

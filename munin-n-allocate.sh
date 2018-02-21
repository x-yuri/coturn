#!/bin/sh

case $1 in
   config)
        cat <<'EOM'
graph_title coturn: n_allocate
graph_vlabel n_allocate
allocs.label allocs
allocs.type DERIVE
allocs.min 0
EOM
        exit 0;;
esac

printf "allocs.value "
cat /var/log/turn_n_allocate 2>/dev/null || echo 0

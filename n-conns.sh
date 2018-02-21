#!/bin/sh

case $1 in
   config)
        cat <<'EOM'
graph_title n_conns
graph_vlabel n_conns
graph_args --lower-limit 0
tcp_conns.label tcp_conns
listen_tcp_conns.label listen_tcp_conns
udp_conns.label udp_conns
EOM
        exit 0;;
esac

printf "tcp_conns.value "
ss -tna | wc -l
printf "listen_tcp_conns.value "
ss -tnl | wc -l
printf "udp_conns.value "
ss -una | wc -l

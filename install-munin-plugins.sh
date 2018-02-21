#!/usr/bin/env bash
set -eu
ln -sf /root/build/coturn/munin-n-sessions.sh /etc/munin/plugins/coturn_n_sessions
ln -sf /root/build/coturn/munin-n-errors.sh /etc/munin/plugins/coturn_n_errors
ln -sf /root/build/coturn/munin-n-allocate.sh /etc/munin/plugins/coturn_n_allocate
ln -sf /root/build/coturn/n-conns.sh /etc/munin/plugins/n_conns
echo '[coturn_n_sessions]
user root
[coturn_n_errors]
user root
[coturn_n_allocate]
user root
[n_conns]
user root' > /etc/munin/plugin-conf.d/my
systemctl restart munin-node

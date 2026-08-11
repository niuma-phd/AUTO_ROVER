# Wheeltec controller udev rule

This rule is pinned to the observed WCH USB bridge VID/PID/serial and replaces
the vendor-installed world-writable `wheeltec_controller3.rules`. It does not
authorize protocol use or actuation. The physical adapter still requires its
runtime identity checks and explicit opt-ins.

Install only after verifying that `/dev/ttyACM0` currently reports USB identity
`1a86:55d4`, serial `0002`, and is not held by another process:

```sh
sudo install -o root -g root -m 0644 \
  deploy/system/udev/wheeltec_controller3.rules \
  /etc/udev/rules.d/wheeltec_controller3.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change /sys/class/tty/ttyACM0
sudo udevadm settle
```

Afterward, `/dev/ttyACM0` must be `root:dialout` with mode `0660`. A mismatch is
a stop condition; do not use `chmod 0777` as a workaround.

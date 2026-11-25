#!/usr/bin/env bash
set -euo pipefail

# chronyd may start before an Internet-bearing interface exists.  Its resolver
# then backs off, which can delay source selection for minutes after Wi-Fi has
# actually connected.  NetworkManager invokes this helper on connectivity
# recovery so source resolution and the initial NTP burst happen immediately.

if ! systemctl is-active --quiet chrony.service; then
  exit 0
fi

/usr/bin/chronyc online >/dev/null
/usr/bin/chronyc refresh >/dev/null || true
/usr/bin/chronyc burst 4/4 >/dev/null || true

# ptp4l also retries on its own.  Starting an inactive instance here removes
# up to one RestartSec interval after Chrony becomes qualified.  `systemctl
# start` is a no-op when the service is already running, so this never disrupts
# an established PTP domain.
for _ in $(seq 1 30); do
  tracking="$(/usr/bin/chronyc tracking 2>/dev/null || true)"
  reference_id="$(sed -n 's/^Reference ID[[:space:]]*:[[:space:]]*\([^[:space:]]*\).*/\1/p' <<<"${tracking}")"
  leap_status="$(sed -n 's/^Leap status[[:space:]]*:[[:space:]]*//p' <<<"${tracking}")"
  if [[ -n "${reference_id}" && "${reference_id}" != "00000000" &&
        "${leap_status}" == "Normal" ]]; then
    /usr/bin/systemctl start --no-block lite3-ptp4l-pc.service
    exit 0
  fi
  sleep 1
done

# No permanent failure is latched here. chronyd keeps polling and ptp4l's
# Restart=on-failure loop keeps checking readiness.
exit 0

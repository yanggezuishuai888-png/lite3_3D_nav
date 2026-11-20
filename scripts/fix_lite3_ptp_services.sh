#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this installer as root: sudo $0" >&2
  exit 1
fi

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DROPIN_SOURCE="${SCRIPT_DIR}/systemd"
readonly SYSTEMD_DIR="/etc/systemd/system"

install -D -m 0644 \
  "${DROPIN_SOURCE}/lite3-ptp4l-pc.service.d/20-recovery.conf" \
  "${SYSTEMD_DIR}/lite3-ptp4l-pc.service.d/20-recovery.conf"
install -D -m 0644 \
  "${DROPIN_SOURCE}/lite3-phc2sys-pc.service.d/20-recovery.conf" \
  "${SYSTEMD_DIR}/lite3-phc2sys-pc.service.d/20-recovery.conf"

systemctl daemon-reload
systemctl reset-failed lite3-ptp4l-pc.service lite3-phc2sys-pc.service

# Restart the owner service.  Its Upholds= relationship starts phc2sys after
# ptp4l passes the PHC and Chrony readiness checks.
systemctl restart lite3-ptp4l-pc.service

deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  if systemctl is-active --quiet lite3-ptp4l-pc.service && \
     systemctl is-active --quiet lite3-phc2sys-pc.service; then
    break
  fi
  sleep 1
done

systemctl --no-pager --full status \
  lite3-ptp4l-pc.service lite3-phc2sys-pc.service

if ! systemctl is-active --quiet lite3-ptp4l-pc.service || \
   ! systemctl is-active --quiet lite3-phc2sys-pc.service; then
  echo "Lite3 PTP services did not both become active within 30 seconds." >&2
  echo "Inspect: journalctl -u lite3-ptp4l-pc.service -u lite3-phc2sys-pc.service -n 200" >&2
  exit 1
fi

echo "Lite3 PC PTP master and CLOCK_REALTIME -> PHC discipline are active."

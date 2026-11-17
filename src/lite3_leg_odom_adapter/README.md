# Lite3 leg odometry adapter

This node turns the validated `/lite3/leg_odom2` pose into the motion-guess TF
used by RTAB-Map:

```text
vo -> leg_odom -> base_link
```

It preserves the source timestamp, requires `/lite3/gateway/time_valid`, uses
Best Effort sensor QoS for the gateway odometry, and rejects stale, future,
non-monotonic, malformed or discontinuous samples. A discontinuity latches a
fault so a source reset cannot silently corrupt a running map.

The validated odometry topic is `/lite3/leg_odom_validated`. Its covariance is
still the source covariance and must not be used as a fused external odometry
input until it has been calibrated. RTAB-Map currently uses the TF only as a
visual registration guess.

After investigating a latched fault, restart the mapping session and clear it:

```bash
ros2 service call /leg_odom_adapter/clear_fault std_srvs/srv/Trigger '{}'
```

Standalone launch:

```bash
ros2 launch lite3_leg_odom_adapter leg_odom_adapter.launch.py
```

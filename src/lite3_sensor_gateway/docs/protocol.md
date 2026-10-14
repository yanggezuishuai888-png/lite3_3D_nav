# UDP protocol v1

This application protocol carries explicitly encoded fields, not native C/C++ structures and not ROS CDR. Integers and IEEE-754 binary64 fields use big endian. Raw image bytes retain the declared Image.is_bigendian and row padding. Both peers must run compatible protocol code.

## Datagram header (56 bytes)

| Byte offset | Length | Meaning |
|---|---|---|
| 0 | 4 | Magic 0x4c334757 / ASCII L3GW |
| 4 | 1 | Version = 1 |
| 5 | 1 | Message type |
| 6 | 1 | Source clock: 0 unknown, 1 steady, 2 system |
| 7 | 1 | Flags: 0 raw, 1 zlib-compressed depth (raw or aligned) |
| 8 | 8 | Nonzero random sender session |
| 16 | 8 | Nonzero per-stream sequence, increasing within session |
| 24 | 8 | Sender Unix/system time ns, sampled before packetization |
| 32 | 8 | Source header time ns (0 for unstamped streams) |
| 40 | 4 | Full wire payload length |
| 44 | 4 | zlib CRC32 of full wire payload |
| 48 | 2 | Fragment index, starting at 0 |
| 50 | 2 | Fragment count |
| 52 | 2 | This fragment's payload length |
| 54 | 2 | Reserved = 0 |

Max datagram 1200 bytes; max fragment data 1144. All but last fragment must have exactly 1144 bytes. Max message payload 16 MiB, max 4 concurrent assemblies and 32 MiB allocated per reassembler. Assembly expires 250 ms after the first fragment is received, without retries. Frame loss is acceptable; partial image publication is forbidden. Sequences are assigned when dequeued for sending; replacement in the sender latest-only queue is counted separately in sender logs, not reflected by sequence gaps.

On the telemetry channel this implementation restricts full payload to 1144 bytes, which fits the current IMU, odometry and RealSense calibration. Very long frame IDs or unusually large calibration messages exceeding this bound are rejected. Image fragments only arrive on the image channel. CRC validates accidental corruption, not sender authenticity. IDs 8–11 extend protocol v1; deploy the sender and receiver together for RGB-D. Older receivers reject these new types.

## Type IDs and payloads

Strings and byte arrays have a u32 length followed by that many bytes. Max string length is 4096. ROS Header = signed seconds carried as u32 two's complement, u32 nanoseconds, frame string. Components x/y/z/w and covariances are f64, retaining original units/signs. Finite floats are required; unsupported/non-finite inputs are dropped and counted as invalid at the receiver.

- 1 Heartbeat: length-prefixed string `lite3_sensor_gateway/1`.
- 2 IMU: header, quaternion xyzw, orientation covariance[9], angular xyz, angular covariance[9], acceleration xyz, acceleration covariance[9].
- 3 Odometry: header, child frame string, position xyz, quaternion xyzw, pose covariance[36], linear xyz, angular xyz, twist covariance[36].
- 4 Ultrasound: f64 raw value.
- 5 Handle: linear xyz, angular xyz.
- 6 Depth: header, u32 height, u32 width, encoding string (`16UC1`), u8 bigendian, u32 step, f64 metres/count scale, length-prefixed image data. Dimensions <=8192, step>=2*width, exact step*height data size. If flag=1, whole payload is replaced by a u32 uncompressed byte count followed by zlib-compressed bytes. Decompression rejects declared lengths over 16 MiB and mismatched resulting lengths.
- 7 CameraInfo: header, u32 height/width, distortion model string, u32 D count (<=64), D floats, K[9], R[9], P[12], u32 binning x/y, u32 ROI x/y/height/width, u8 do_rectify.
- 8 Color: header, u32 height/width, encoding string (`rgb8` or `bgr8`), u8 bigendian, u32 step, length-prefixed raw image data. Row padding is preserved. No JPEG or zlib compression is applied.
- 9 AlignedDepth: same payload and optional zlib compression as type 6, with its own sequence and topic.
- 10 ColorInfo: same payload as type 7, for RGB calibration.
- 11 AlignedInfo: same payload as type 7, for depth aligned to RGB.
- 16–22: one byte Bool (0 or 1), respectively imu/odom/odom2/joint/ultrasound/realsense/lidar source status.

The receiver establishes a new session only from a complete, valid heartbeat. It remembers the most recent 128 retired sessions to reject delayed packets after restart. It discards duplicate or older completed sequences for each stream. A new session clears source freshness, cached calibration and unfinished frames, and publishes false statuses until updates arrive. Late joiners recover at the next heartbeat; there is no handshake, ACK or retransmission.

No command types exist. The receiver sends no UDP responses. The sender exposes no UDP command listener.

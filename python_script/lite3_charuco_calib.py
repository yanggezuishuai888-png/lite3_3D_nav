
#!/usr/bin/env python3
# Lite3 + D435i ChArUco extrinsic calibration
# Supports OpenCV 4.6.0 and newer versions.

import argparse
import json
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def make_board(args):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV does not contain the aruco module")

    ar = cv2.aruco

    if not hasattr(ar, args.dictionary):
        raise RuntimeError("Unknown ArUco dictionary: " + args.dictionary)

    dictionary = ar.getPredefinedDictionary(
        getattr(ar, args.dictionary)
    )

    square = args.square_mm / 1000.0
    marker = args.marker_mm / 1000.0

    if not (0 < marker < square):
        raise ValueError("Marker size must be smaller than square size")

    # OpenCV 4.6 old API
    if hasattr(ar, "CharucoBoard_create"):
        board = ar.CharucoBoard_create(
            args.cols, args.rows, square, marker, dictionary
        )

    # Newer OpenCV API
    elif hasattr(ar, "CharucoBoard"):
        board = ar.CharucoBoard(
            (args.cols, args.rows), square, marker, dictionary
        )
        if hasattr(board, "setLegacyPattern"):
            board.setLegacyPattern(False)

    else:
        raise RuntimeError(
            "No ChArUco board API. Check OpenCV installation."
        )

    return board, dictionary


def detect_charuco(gray, board, dictionary, K=None, D=None):
    ar = cv2.aruco

    if hasattr(ar, "CharucoDetector"):
        detector = ar.CharucoDetector(board)
        corners, ids, marker_corners, marker_ids = (
            detector.detectBoard(gray)
        )

    else:
        # OpenCV 4.6.0
        if not hasattr(ar, "interpolateCornersCharuco"):
            raise RuntimeError(
                "Missing interpolateCornersCharuco API"
            )

        params = ar.DetectorParameters_create()

        marker_corners, marker_ids, _ = ar.detectMarkers(
            gray, dictionary, parameters=params
        )

        if marker_ids is None or len(marker_ids) == 0:
            return None, None, 0

        if K is None:
            _, corners, ids = ar.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board
            )
        else:
            _, corners, ids = ar.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board,
                cameraMatrix=K,
                distCoeffs=D
            )

    nmarkers = 0 if marker_ids is None else len(marker_ids)

    if corners is None or ids is None or len(ids) == 0:
        return None, None, nmarkers

    corners = corners.reshape(-1, 2).astype(np.float64)
    ids = ids.reshape(-1).astype(np.int32)

    return corners, ids, nmarkers


def make_transform(Rm, translation):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(translation).reshape(3)
    return T


def invert_transform(T):
    result = np.eye(4)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result


def transform_to_xyz_rpy(T):
    xyz = T[:3, 3].tolist()
    rpy = Rotation.from_matrix(
        T[:3, :3]
    ).as_euler("xyz").tolist()
    return xyz, rpy


def summarize_poses(poses):
    translations = np.stack([
        T[:3, 3] for T in poses
    ])

    rotations = Rotation.from_matrix(
        np.stack([T[:3, :3] for T in poses])
    )

    mean_translation = np.median(translations, axis=0)
    mean_rotation = rotations.mean()

    T_mean = make_transform(
        mean_rotation.as_matrix(),
        mean_translation
    )

    translation_errors = np.linalg.norm(
        translations - mean_translation, axis=1
    ) * 1000.0

    rotation_errors = (
        mean_rotation.inv() * rotations
    ).magnitude() * 180.0 / np.pi

    stats = {
        "translation_mm_p50": float(
            np.percentile(translation_errors, 50)
        ),
        "translation_mm_p95": float(
            np.percentile(translation_errors, 95)
        ),
        "rotation_deg_p50": float(
            np.percentile(rotation_errors, 50)
        ),
        "rotation_deg_p95": float(
            np.percentile(rotation_errors, 95)
        )
    }

    return T_mean, stats


def image_to_gray(msg):
    encoding = msg.encoding.lower()

    channels = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1
    }

    if encoding not in channels:
        raise ValueError(
            "Unsupported image encoding: " + msg.encoding
        )

    count = channels[encoding]

    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if raw.size < msg.step * msg.height:
        raise ValueError("Image buffer too small")

    if msg.step < msg.width * count:
        raise ValueError("Invalid image step")

    image = raw[:msg.step * msg.height]
    image = image.reshape(msg.height, msg.step)
    image = image[:, :msg.width * count]
    image = image.reshape(msg.height, msg.width, count)

    if encoding == "mono8":
        return np.ascontiguousarray(image[:, :, 0])

    conversion = {
        "rgb8": cv2.COLOR_RGB2GRAY,
        "bgr8": cv2.COLOR_BGR2GRAY,
        "rgba8": cv2.COLOR_RGBA2GRAY,
        "bgra8": cv2.COLOR_BGRA2GRAY
    }

    return cv2.cvtColor(image, conversion[encoding])


def transform_from_ros(tf):
    q = tf.transform.rotation
    t = tf.transform.translation

    Rm = Rotation.from_quat([
        q.x, q.y, q.z, q.w
    ]).as_matrix()

    return make_transform(
        Rm, [t.x, t.y, t.z]
    )


def print_pose(title, T):
    xyz, rpy = transform_to_xyz_rpy(T)

    print("\n" + "=" * 55)
    print(title)
    print("=" * 55)

    print(
        "xyz (m): ",
        " ".join(f"{v:+.6f}" for v in xyz)
    )

    print(
        "rpy (rad):",
        " ".join(f"{v:+.6f}" for v in rpy)
    )

    print(
        "rpy (deg):",
        " ".join(f"{v:+.3f}" for v in np.degrees(rpy))
    )

    return {
        "xyz_m": xyz,
        "rpy_rad": rpy,
        "rpy_deg": np.degrees(rpy).tolist()
    }


def self_test(board, dictionary, args):
    if hasattr(board, "generateImage"):
        image = board.generateImage(
            (900, 1050), marginSize=50
        )
    else:
        image = board.draw(
            (900, 1050), marginSize=50
        )

    corners, ids, nmarkers = detect_charuco(
        image, board, dictionary
    )

    expected = (args.cols - 1) * (args.rows - 1)
    detected = 0 if ids is None else len(ids)

    print("OpenCV version:", cv2.__version__)
    print("Detected markers:", nmarkers)
    print("ChArUco corners:", detected, "/", expected)

    if detected < expected - 2:
        raise RuntimeError(
            "Self-test failed: insufficient corners"
        )

    print("PASS: synthetic ChArUco detection successful")


def run_calibration(args, board, dictionary):
    import rclpy
    from sensor_msgs.msg import Image, CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()

    node = rclpy.create_node(
        "lite3_charuco_extrinsic_calibration"
    )

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    state = {
        "K": None,
        "D": None,
        "frame": None,
        "width": None,
        "height": None,
        "poses": [],
        "rms": [],
        "frames": 0,
        "error_printed": False
    }

    board_points = np.asarray(
        board.getChessboardCorners(),
        dtype=np.float64
    ).reshape(-1, 3)

    def on_camera_info(msg):
        if state["K"] is not None:
            return

        if not msg.header.frame_id:
            return

        K = np.array(
            msg.k, dtype=np.float64
        ).reshape(3, 3)

        if K[0, 0] <= 0 or K[1, 1] <= 0:
            return

        if msg.distortion_model not in (
            "plumb_bob", "rational_polynomial", ""
        ):
            print(
                "WARNING: unsupported distortion model:",
                msg.distortion_model
            )
            return

        D = (
            np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            if msg.d else np.zeros((5, 1))
        )

        state["K"] = K
        state["D"] = D
        state["frame"] = msg.header.frame_id.lstrip("/")
        state["width"] = msg.width
        state["height"] = msg.height

        print(
            f"CameraInfo: {msg.width}x{msg.height}, "
            f"frame={state['frame']}"
        )

    def on_image(msg):
        if state["K"] is None:
            return

        if len(state["poses"]) >= args.samples:
            return

        if (
            msg.width != state["width"]
            or msg.height != state["height"]
        ):
            if not state["error_printed"]:
                print(
                    "ERROR: image size differs from CameraInfo"
                )
                state["error_printed"] = True
            return

        if (
            msg.header.frame_id
            and msg.header.frame_id.lstrip("/")
            != state["frame"]
        ):
            if not state["error_printed"]:
                print(
                    "ERROR: Image and CameraInfo frame IDs differ"
                )
                state["error_printed"] = True
            return

        try:
            gray = image_to_gray(msg)

            corners, ids, nmarkers = detect_charuco(
                gray,
                board,
                dictionary,
                state["K"],
                state["D"]
            )

            state["frames"] += 1

            if ids is None or len(ids) < args.min_corners:
                return

            object_points = board_points[ids]

            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                corners,
                state["K"],
                state["D"],
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not ok or float(tvec[2, 0]) <= 0:
                return

            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                state["K"],
                state["D"]
            )

            error = projected.reshape(-1, 2) - corners

            rms = float(
                np.sqrt(
                    np.mean(
                        np.sum(error * error, axis=1)
                    )
                )
            )

            if rms > args.max_rms:
                return

            Rm, _ = cv2.Rodrigues(rvec)

            # T_O_A: board -> camera optical frame
            T_O_A = make_transform(Rm, tvec)

            state["poses"].append(T_O_A)
            state["rms"].append(rms)

            n = len(state["poses"])

            if n == 1 or n % 5 == 0:
                print(
                    f"Accepted {n}/{args.samples}, "
                    f"markers={nmarkers}, "
                    f"corners={len(ids)}, "
                    f"RMS={rms:.3f}px"
                )

        except Exception as exc:
            if not state["error_printed"]:
                print("Image processing error:", repr(exc))
                state["error_printed"] = True

    sub_info = node.create_subscription(
        CameraInfo,
        args.info_topic,
        on_camera_info,
        qos_profile_sensor_data
    )

    sub_image = node.create_subscription(
        Image,
        args.image_topic,
        on_image,
        qos_profile_sensor_data
    )

    print("Waiting for RGB images and CameraInfo...")
    print("Keep robot and calibration board stationary.")

    start = time.monotonic()

    try:
        while (
            rclpy.ok()
            and len(state["poses"]) < args.samples
            and time.monotonic() - start < args.timeout
        ):
            rclpy.spin_once(node, timeout_sec=0.15)

        if len(state["poses"]) < 5:
            print(
                "ERROR: insufficient valid frames:",
                len(state["poses"])
            )
            print(
                "Check topics, board visibility, "
                "printing scale and board parameters."
            )
            return

        T_O_A, stats = summarize_poses(
            state["poses"]
        )

        result = {
            "opencv_version": cv2.__version__,
            "valid_samples": len(state["poses"]),
            "optical_frame": state["frame"],
            "board_rows": args.rows,
            "board_cols": args.cols,
            "square_mm": args.square_mm,
            "marker_mm": args.marker_mm,
            "dictionary": args.dictionary,
            "mean_reprojection_rms_px": float(
                np.mean(state["rms"])
            ),
            "repeatability": stats
        }

        result["optical_to_board"] = print_pose(
            state["frame"] + " -> board (PnP)",
            T_O_A
        )

        print("\nRepeatability (not absolute accuracy):")
        print(json.dumps(stats, indent=2))

        if args.board_xyz is not None:
            # T_B_A: board -> base_link
            R_B_A = Rotation.from_euler(
                "xyz",
                args.board_rpy_deg,
                degrees=True
            ).as_matrix()

            T_B_A = make_transform(
                R_B_A,
                args.board_xyz
            )

            # T_B_O = T_B_A * inverse(T_O_A)
            T_B_O = T_B_A @ invert_transform(T_O_A)

            result["base_to_optical"] = print_pose(
                "base_link -> " + state["frame"],
                T_B_O
            )

            try:
                # T_L_O: optical -> camera_link
                tf = tf_buffer.lookup_transform(
                    args.camera_link_frame,
                    state["frame"],
                    Time()
                )

                T_L_O = transform_from_ros(tf)

                # T_B_L = T_B_O * inverse(T_L_O)
                T_B_L = T_B_O @ invert_transform(T_L_O)

                result["base_to_camera_link"] = print_pose(
                    "base_link -> " + args.camera_link_frame,
                    T_B_L
                )

            except Exception as exc:
                print("\nWARNING: camera internal TF unavailable")
                print(str(exc))
                print(
                    "Only base_link -> optical frame "
                    "has been calculated."
                )
                print(
                    "Do NOT use that result as "
                    "base_link -> camera_link."
                )

        else:
            print(
                "\nPnP-only mode: board pose relative "
                "to base_link was not provided."
            )

        with open(
            args.output, "w", encoding="utf-8"
        ) as f:
            json.dump(
                result,
                f,
                indent=2,
                ensure_ascii=False
            )

        print("\nResult saved to:", args.output)

    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Lite3 D435i ChArUco calibration"
    )

    # Default board: 7 rows x 6 columns, 25/18 mm
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--cols", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--marker-mm", type=float, default=18.0)
    parser.add_argument(
        "--dictionary", default="DICT_4X4_50"
    )

    parser.add_argument(
        "--image-topic",
        default="/lite3/camera/color/image_raw"
    )
    parser.add_argument(
        "--info-topic",
        default="/lite3/camera/color/camera_info"
    )
    parser.add_argument(
        "--camera-link-frame",
        default="camera_link"
    )

    parser.add_argument(
        "--board-xyz",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z")
    )
    parser.add_argument(
        "--board-rpy-deg",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW")
    )

    parser.add_argument(
        "--samples", type=int, default=40
    )
    parser.add_argument(
        "--timeout", type=float, default=90.0
    )
    parser.add_argument(
        "--min-corners", type=int, default=10
    )
    parser.add_argument(
        "--max-rms", type=float, default=2.0
    )
    parser.add_argument(
        "--output",
        default="lite3_charuco_result.json"
    )
    parser.add_argument(
        "--self-test",
        action="store_true"
    )

    args = parser.parse_args()

    if (
        (args.board_xyz is None)
        != (args.board_rpy_deg is None)
    ):
        parser.error(
            "Provide both --board-xyz and "
            "--board-rpy-deg, or neither"
        )

    board, dictionary = make_board(args)

    if args.self_test:
        self_test(board, dictionary, args)
    else:
        run_calibration(args, board, dictionary)


if __name__ == "__main__":
    main()

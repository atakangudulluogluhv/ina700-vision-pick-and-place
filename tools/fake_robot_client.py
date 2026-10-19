"""Stand-in for the ABB robot, for testing the server without the controller.

It mirrors robot/MainModule.mod: connect as a client, let the server speak
first, then answer every PICK message with ACK;<id>;OK or ACK;<id>;FAIL by the
same rules the RAPID module applies (field count, numbers, reach, known place
cavity). Keep the constants below in step with that module.

    python tools/fake_robot_client.py --host 127.0.0.1
"""

import argparse
import socket
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.core.config import Config  # noqa: E402  (path setup must come first)

RECEIVE_BUFFER_BYTES = 4094

# Mirrors of the RAPID constants in robot/MainModule.mod.
FIELD_SEPARATOR = ";"
PICK_FIELD_COUNT = 8
PICK_REACH_MM = 400.0
PICK_Z_MIN_MM = -215.0
PICK_Z_MAX_MM = 100.0
PLACE_CAVITY_SHAPES = ("circle", "star", "hexagon")


def verdict_for(message: str) -> tuple[str, str]:
    """Return (message id, "OK" or "FAIL") exactly as MainModule.mod decides."""
    fields = message.split(FIELD_SEPARATOR)
    if len(fields) != PICK_FIELD_COUNT or fields[0] != "PICK":
        return "0", "FAIL"
    message_id, shape = fields[1], fields[2]
    try:
        x_mm, y_mm, z_mm, _rz_deg = (float(field) for field in fields[4:])
    except ValueError:
        return message_id, "FAIL"
    reachable = (abs(x_mm) <= PICK_REACH_MM and abs(y_mm) <= PICK_REACH_MM
                 and PICK_Z_MIN_MM <= z_mm <= PICK_Z_MAX_MM)
    if not reachable or shape not in PLACE_CAVITY_SHAPES:
        return message_id, "FAIL"
    return message_id, "OK"


def parse_arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fake ABB robot client")
    parser.add_argument("--host", default=None, help="server address (default: comm.host)")
    parser.add_argument("--port", type=int, default=None, help="server port (default: comm.port)")
    parser.add_argument("--fail-every", type=int, default=0,
                        help="answer FAIL on every Nth message, to exercise the retry path")
    parser.add_argument("--motion-sec", type=float, default=0.0,
                        help="seconds to wait before answering, like a real pick and place")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_arguments(argv)
    config = Config.load()
    host = args.host or str(config.get("comm.host"))
    port = args.port or int(config.get("comm.port"))
    send_encoding = str(config.get("comm.receive_encoding"))
    receive_encoding = str(config.get("comm.send_encoding"))

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        print("[robot] connecting to " + host + ":" + str(port))
        client_socket.connect((host, port))
        print("[robot] connected, waiting for the server to speak first")

        message_count = 0
        while True:
            payload = client_socket.recv(RECEIVE_BUFFER_BYTES)
            if not payload:
                print("[robot] the server closed the connection")
                return 0

            message = payload.decode(receive_encoding).strip()
            message_count += 1
            print("[robot] received " + repr(message) + "  (" + str(len(message)) + " chars)")

            message_id, verdict = verdict_for(message)
            time.sleep(args.motion_sec)
            if args.fail_every > 0 and message_count % args.fail_every == 0:
                verdict = "FAIL"
            answer = "ACK;" + message_id + ";" + verdict
            client_socket.sendall(answer.encode(send_encoding))
            print("[robot] answered " + repr(answer))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[robot] stopped")
    except OSError as error:
        print("[robot] connection error: " + str(error))
        sys.exit(1)

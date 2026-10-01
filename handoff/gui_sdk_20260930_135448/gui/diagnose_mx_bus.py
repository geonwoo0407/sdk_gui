"""Read-only MX-28/MX-106 Protocol 2.0 diagnostics; close other serial clients first.

Sends only PING and READ. Never changes torque, goals, EEPROM or baud settings
on a motor. A stopped GUI changes bus traffic, so this is a stationary sample,
not a reproduction of motion-time failures.
"""
import argparse
import collections
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import subprocess
import termios
import time

from dynamixel_sdk import COMM_SUCCESS, PortHandler, Protocol2PacketHandler


FAULTS = {1: "input_voltage", 4: "overheating", 8: "encoder",
          16: "electrical_shock_or_insufficient_power", 32: "overload"}
SUPPORTED_MODELS = {30: "MX-28(2.0)", 321: "MX-106(2.0)"}


def decode_ram(data, model=321):
    if model not in SUPPORTED_MODELS or len(data) != 83:
        raise ValueError("Expected supported MX model and complete RAM block 64..146")
    def value(address, size=1, signed=False):
        return int.from_bytes(bytes(data[address - 64:address - 64 + size]),
                              "little", signed=signed)
    fault = value(70)
    state = {
        "torque": value(64), "status_return_level": value(68),
        "hardware_error": fault,
        "faults": [name for bit, name in FAULTS.items() if fault & bit],
        "position_d_gain": value(80, 2), "position_i_gain": value(82, 2),
        "position_p_gain": value(84, 2),
        "bus_watchdog": value(98, signed=True),
        "goal_pwm_raw": value(100, 2, True),
        "profile_acceleration": value(108, 4),
        "profile_velocity": value(112, 4),
        "goal_position": value(116, 4, True),
        "realtime_tick": value(120, 2), "moving": value(122),
        "present_pwm_raw": value(124, 2, True),
        "present_velocity_raw": value(128, 4, True),
        "present_position": value(132, 4, True),
        "input_voltage_v": round(value(144, 2) * 0.1, 1),
        "temperature_c": value(146),
    }
    # MX-28 address 126 is inferred load, not measured current; address 102
    # is reserved. Do not interpret it using the MX-106 control table.
    if model == 30:
        state["present_load_percent"] = round(value(126, 2, True) * 0.1, 1)
    else:
        state["goal_current_raw"] = value(102, 2, True)
        state["present_current_ma"] = round(value(126, 2, True) * 3.36, 2)
    return state


def decode_config(data):
    if len(data) != 64:
        raise ValueError("Expected complete configuration block 0..63")
    def value(address, size=1, signed=False):
        return int.from_bytes(bytes(data[address:address + size]),
                              "little", signed=signed)
    model = value(0, 2)
    if model not in SUPPORTED_MODELS:
        raise ValueError(f"Unsupported model {model}")
    config = {
        "model": value(0, 2), "firmware": value(6), "id": value(7),
        "baud_index": value(8), "return_delay_us": value(9) * 2,
        "drive_mode": value(10), "operating_mode": value(11),
        "secondary_id": value(12), "protocol": value(13),
        "homing_offset": value(20, 4, True),
        "temperature_limit_c": value(31),
        "max_voltage_v": value(32, 2) * 0.1,
        "min_voltage_v": value(34, 2) * 0.1,
        "pwm_limit": value(36, 2),
        "velocity_limit": value(44, 4),
        "max_position_limit": value(48, 4),
        "min_position_limit": value(52, 4), "shutdown_mask": value(63),
    }
    if model == 321:
        config["current_limit_raw"] = value(38, 2)
    return config


def port_users(port):
    result = subprocess.run(["fuser", port], capture_output=True, text=True)
    if result.returncode not in (0, 1) or (result.returncode == 1 and result.stderr.strip()):
        raise RuntimeError("Cannot check serial port ownership: " + result.stderr)
    return [int(pid) for pid in result.stdout.split() if int(pid) != os.getpid()]


def run(args):
    users = port_users(args.port)
    if users:
        raise RuntimeError(f"Port busy: {args.port}, PID {users}. Close the GUI first.")
    port = PortHandler(args.port)
    packet = Protocol2PacketHandler()
    port.baudrate = args.baud  # Host-side baud only; no motor configuration write.
    rows = []
    stats = {j: collections.Counter() for j in args.ids}
    last = {}
    configs = {}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", buffering=1) as log:
        def emit(row):
            row = {"time": datetime.now().astimezone().isoformat(), **row}
            log.write(json.dumps(row, ensure_ascii=False) + "\n")
            return row

        try:
            if not port.openPort():
                raise RuntimeError("Cannot open port")
            fcntl.ioctl(port.ser.fileno(), termios.TIOCEXCL)
            users = port_users(args.port)
            if users:
                raise RuntimeError(f"Another serial client opened the port: {users}")
            latency_path = Path("/sys/bus/usb-serial/devices") / Path(os.path.realpath(args.port)).name / "latency_timer"
            latency = latency_path.read_text().strip() if latency_path.exists() else None
            emit({"event": "session", "port": args.port, "baud": args.baud,
                  "ids": args.ids, "usb_latency_ms": latency,
                  "read_only": True, "samples_requested": args.samples})

            def read(j, address, length):
                started = time.perf_counter()
                try:
                    data, comm, error = packet.readTxRx(port, j, address, length)
                    valid = comm == COMM_SUCCESS and (error & 0x7F) == 0 and len(data) == length
                    row = emit({"event": "read", "id": j, "address": address,
                                "length": length, "comm": comm,
                                "comm_text": packet.getTxRxResult(comm),
                                "packet_error": error, "valid": valid,
                                "elapsed_ms": round((time.perf_counter() - started) * 1000, 3),
                                "data": list(data)})
                except (OSError, IndexError, TypeError) as exc:
                    port.is_using = False
                    row = emit({"event": "read", "id": j, "address": address,
                                "valid": False, "exception": str(exc)})
                    data, valid = [], False
                stats[j]["reads"] += 1
                stats[j]["valid" if valid else "failed"] += 1
                return data if valid else None

            supported = {}
            for j in args.ids:
                for attempt in range(3):
                    try:
                        model, comm, error = packet.ping(port, j)
                    except (OSError, IndexError, TypeError) as exc:
                        port.is_using = False
                        emit({"event": "ping", "id": j, "exception": str(exc)})
                        continue
                    emit({"event": "ping", "id": j, "model": model,
                          "comm": comm, "packet_error": error, "attempt": attempt + 1})
                    if comm == COMM_SUCCESS and not (error & 0x7F):
                        if model in SUPPORTED_MODELS:
                            supported[j] = model
                            print(f"ID {j}: {SUPPORTED_MODELS[model]}")
                        else:
                            print(f"ID {j}: model {model}; only MX-28(2.0)/MX-106(2.0) are decoded.")
                        break
                else:
                    print(f"ID {j}: no valid ping response")

            # Read fault state before slower configuration reads; Alert (0x80)
            # is logged separately and does not invalidate readable RAM data.
            for sample in range(args.samples):
                started = time.monotonic()
                for j in supported:
                    data = read(j, 64, 83)
                    if data is None:
                        print(f"sample {sample}: ID {j} READ FAILED")
                        continue
                    state = decode_ram(data, supported[j])
                    rows.append(emit({"event": "state", "id": j, "model": supported[j], "sample": sample, **state}))
                    key = (state["torque"], state["hardware_error"], state["bus_watchdog"])
                    if last.get(j) != key or sample == args.samples - 1:
                        effort = (f"load_estimate={state['present_load_percent']}%"
                                  if supported[j] == 30 else
                                  f"current={state['present_current_ma']}mA")
                        print(f"ID {j}: torque={state['torque']} fault=0x{state['hardware_error']:02x} "
                              f"{state['faults']} voltage={state['input_voltage_v']}V "
                              f"temp={state['temperature_c']}C {effort} "
                              f"goal/position={state['goal_position']}/{state['present_position']} "
                              f"watchdog={state['bus_watchdog']}")
                    last[j] = key
                if sample == 0:
                    for j in supported:
                        data = read(j, 0, 64)
                        if data is not None:
                            configs[j] = decode_config(data)
                            emit({"event": "config", "id": j, **configs[j]})
                            print(f"ID {j} config: {json.dumps(configs[j])}")
                if sample + 1 < args.samples:
                    time.sleep(max(0.0, args.interval - (time.monotonic() - started)))
            for j in args.ids:
                states = [r for r in rows if r["id"] == j]
                summary = {"event": "summary", "id": j, **stats[j]}
                if states:
                    summary.update(voltage_min=min(r["input_voltage_v"] for r in states),
                                   voltage_max=max(r["input_voltage_v"] for r in states),
                                   temperature_max=max(r["temperature_c"] for r in states),
                                   hardware_errors=sorted({r["hardware_error"] for r in states}))
                emit(summary)
                print(json.dumps(summary))
        finally:
            if port.is_open:
                port.closePort()
    print(f"Saved: {args.output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=4000000)
    parser.add_argument("--ids", nargs="+", type=int, default=[17, 18, 19, 20, 21, 22])
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--output", type=Path, default=Path("diagnostics") / (datetime.now().strftime("mx_bus_%Y%m%d_%H%M%S_%f") + ".jsonl"))
    args = parser.parse_args()
    if args.samples < 1 or args.interval < 0 or any(not 0 <= j <= 252 for j in args.ids):
        parser.error("samples >= 1, interval >= 0, IDs in 0..252 required")
    args.ids = list(dict.fromkeys(args.ids))
    try:
        run(args)
    except (RuntimeError, OSError) as exc:
        parser.exit(1, str(exc) + "\n")


if __name__ == "__main__":
    main()

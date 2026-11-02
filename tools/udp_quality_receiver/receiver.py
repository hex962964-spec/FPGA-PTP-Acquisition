"""Acquisition V1/V2 UDP metadata receiver; Python standard library only."""
import argparse
import collections
import csv
import datetime
import json
from pathlib import Path
import socket
import struct
import time

QUALITY = {0: "INVALID", 1: "REACQUIRE", 2: "LOCKED", 3: "HOLDOVER"}
FIELDS = ["received_at", "peer_ip", "peer_port", "udp_bytes", "version",
          "time_quality", "sequence", "channel", "sample_count", "timestamp",
          "disposition", "error"]


def parse_packet(payload):
    # Each 16-bit word is sent high-byte first; multiword fields are LOW word first.
    if len(payload) < 20 or len(payload) % 2:
        raise ValueError("truncated_or_odd_length")
    w = struct.unpack_from(">10H", payload)
    if w[0] != 0x55AA:
        raise ValueError("bad_magic")
    if w[1] == 1:
        version, quality = 1, "UNKNOWN"
    elif (w[1] & 255) == 2:
        if w[1] & 0xFC00:
            raise ValueError("unsupported_reserved_flags")
        version, quality = 2, QUALITY[(w[1] >> 8) & 3]
    else:
        raise ValueError("unsupported_version")
    if len(payload) != 20 + 2 * w[5]:
        raise ValueError("sample_count_length_mismatch")
    return dict(version=version, time_quality=quality,
                sequence=w[2] | (w[3] << 16), channel=w[4], sample_count=w[5],
                timestamp=sum(w[6+i] << (16*i) for i in range(4)))


def classify(packet):
    return {"LOCKED": "USE", "HOLDOVER": "RISK", "REACQUIRE": "WAIT",
            "INVALID": "ISOLATE", "UNKNOWN": "UNVERIFIED"}[packet["time_quality"]]


class Recorder:
    def __init__(self, directory, raw=False):
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self.csv_file = (directory / "packets.csv").open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.raw_file = (directory / "raw_payloads.jsonl").open("w", encoding="utf-8") if raw else None
        self.counts = collections.Counter()
        self.last = None

    def record(self, payload, peer):
        row = dict.fromkeys(FIELDS, "")
        row.update(received_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   peer_ip=peer[0], peer_port=peer[1], udp_bytes=len(payload))
        try:
            packet = parse_packet(payload)
            row.update(packet, disposition=classify(packet))
            self.counts[packet["time_quality"]] += 1
        except ValueError as exc:
            row.update(disposition="MALFORMED", error=str(exc))
            self.counts["MALFORMED"] += 1
        self.counts["TOTAL"] += 1
        self.writer.writerow(row)
        if self.raw_file:
            self.raw_file.write(json.dumps(dict(received_at=row["received_at"], peer=list(peer),
                                                payload_hex=payload.hex())) + "\n")
        self.last = row
        return row

    def flush(self):
        self.csv_file.flush()
        if self.raw_file:
            self.raw_file.flush()

    def close(self):
        self.flush()
        self.csv_file.close()
        if self.raw_file:
            self.raw_file.close()
        (self.directory / "summary.json").write_text(json.dumps(dict(
            kind="host_received_udp_observations", counts=dict(self.counts), last=self.last,
            note="Missing packets/quiet periods do not establish HOLDOVER or INVALID; tags are not an independent accuracy measurement."
        ), ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Receive FPGA acquisition UDP V1/V2; write per-packet quality CSV.")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--source", default="192.168.0.14", help="Expected DUT IP, empty string allows all sources")
    parser.add_argument("--seconds", type=float, default=0, help="0 runs until Ctrl+C")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "captures")
    parser.add_argument("--raw", action="store_true", help="Also save every UDP payload as hex JSONL (can grow rapidly)")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535 or args.seconds < 0:
        parser.error("port must be 1..65535 and seconds must be nonnegative")
    args.output.mkdir(parents=True, exist_ok=True)
    directory = args.output / ("capture_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # Do not enable SO_REUSEADDR: a second receiver must fail visibly.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind((args.bind, args.port))
        sock.settimeout(0.25)
        recorder = Recorder(directory, args.raw)
        print("Listening:", sock.getsockname(), "expected DUT:", args.source or "any", flush=True)
        print("Output:", directory, "  Ctrl+C stops and saves summary.", flush=True)
        started = report = time.monotonic()
        ignored = 0
        try:
            while args.seconds == 0 or time.monotonic() - started < args.seconds:
                try:
                    data, peer = sock.recvfrom(65535)
                    if not args.source or peer[0] == args.source:
                        recorder.record(data, peer)
                    else:
                        ignored += 1
                except socket.timeout:
                    pass
                now = time.monotonic()
                if now - report >= 1:
                    report = now
                    recorder.flush()
                    last = recorder.last
                    detail = ("last=" + str(last["time_quality"]) + " seq=" + str(last["sequence"]) +
                              " ts=" + str(last["timestamp"]) + " action=" + str(last["disposition"])) if last else "NO PACKETS"
                    print(dict(recorder.counts), detail, "other_source=" + str(ignored), flush=True)
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            recorder.close()
            print("Saved:", directory, flush=True)


if __name__ == "__main__":
    main()

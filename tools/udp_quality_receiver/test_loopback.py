"""Real UDP loopback integration; sends synthetic packets, never board traffic."""
import csv
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

root=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory() as temp:
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    proc=subprocess.Popen([sys.executable,'-B',str(root/'receiver.py'),'--bind','127.0.0.1',
        '--source','127.0.0.1','--port',str(port),'--seconds','2','--output',temp,'--raw'],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        first=proc.stdout.readline()
        assert 'Listening:' in first,first
        fixture=bytes.fromhex('55aa 0202 5678 1234 0007 0003 7788 5566 3344 1122 0000 0001 0002')
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            for flags in [0x202,0x302,0x002,0x102,0x202,1]:
                payload=fixture[:2]+flags.to_bytes(2,'big')+fixture[4:]
                sender.sendto(payload,('127.0.0.1',port));time.sleep(.02)
            sender.sendto(b'bad',('127.0.0.1',port))
        stdout,stderr=proc.communicate(timeout=8)
        assert proc.returncode==0,stderr
        capture=next(Path(temp).glob('capture_*'))
        summary=json.loads((capture/'summary.json').read_text(encoding='utf-8'))
        assert summary['counts']==dict(LOCKED=2,HOLDOVER=1,INVALID=1,REACQUIRE=1,UNKNOWN=1,MALFORMED=1,TOTAL=7),summary
        with (capture/'packets.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
        assert len(rows)==7
        assert rows[0]['timestamp']==str(0x1122334455667788)
        assert rows[2]['disposition']=='ISOLATE'
        assert len((capture/'raw_payloads.jsonl').read_text().splitlines())==7
        print('PASS actual localhost UDP receive, source filter, V1/V2 quality classification, CSV, raw journal and summary; synthetic data only')
    finally:
        if proc.poll() is None:proc.kill();proc.wait()

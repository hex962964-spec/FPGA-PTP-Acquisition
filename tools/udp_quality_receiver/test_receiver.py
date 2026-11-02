import csv
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

spec=importlib.util.spec_from_file_location("receiver",Path(__file__).with_name("receiver.py"))
r=importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)

def packet(flags=0x202, count=3, seq=0x12345678, stamp=0x1122334455667788):
    words=[0x55AA,flags,seq&65535,seq>>16,7,count]+[(stamp>>(16*i))&65535 for i in range(4)]
    return struct.pack(">10H",*words)+struct.pack(">"+"H"*count,*range(count))

class ProtocolTests(unittest.TestCase):
    def test_all_quality_states_and_policy(self):
        for n,name in r.QUALITY.items():
            p=r.parse_packet(packet((n<<8)|2))
            self.assertEqual(p['time_quality'],name)
            self.assertEqual(r.classify(p),['ISOLATE','WAIT','USE','RISK'][n])
    def test_wire_word_order_and_64bit_precision(self):
        p=r.parse_packet(packet(stamp=0xFEDCBA9876543210))
        self.assertEqual(p['sequence'],0x12345678)
        self.assertEqual(p['timestamp'],0xFEDCBA9876543210)
        self.assertEqual((p['sample_count'],p['channel']),(3,7))
    def test_v1_is_unknown_not_invalid_or_locked(self):
        p=r.parse_packet(packet(1))
        self.assertEqual((p['version'],p['time_quality'],r.classify(p)),(1,'UNKNOWN','UNVERIFIED'))
    def test_explicit_wire_fixture(self):
        p=r.parse_packet(bytes.fromhex('55aa 0202 5678 1234 0007 0003 7788 5566 3344 1122 0000 0001 0002'))
        self.assertEqual(p,dict(version=2,time_quality='LOCKED',sequence=0x12345678,
                               channel=7,sample_count=3,timestamp=0x1122334455667788))
    def test_empty_payload_packet(self):
        self.assertEqual(r.parse_packet(packet(count=0))['sample_count'],0)
    def test_malformed(self):
        cases=[b'',packet()[:-1],packet()[:-2],packet()+b'xx',b'xx'+packet()[2:],packet(3),packet(0x402),packet(0x101)]
        for data in cases:
            with self.subTest(data=data[:4]):
                with self.assertRaises(ValueError):r.parse_packet(data)
    def test_journal_retains_invalid_but_isolates_it(self):
        with tempfile.TemporaryDirectory() as temp:
            rec=r.Recorder(Path(temp)/'capture',raw=True)
            for flags in [0x202,0x302,0x002,0x102,0x202,1]:rec.record(packet(flags),('192.168.0.14',1234))
            rec.record(b'bad',('192.168.0.14',1234))
            rec.close()
            with (Path(temp)/'capture/packets.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),7)
            self.assertEqual([x['disposition'] for x in rows],['USE','RISK','ISOLATE','WAIT','USE','UNVERIFIED','MALFORMED'])
            self.assertEqual(rec.counts['TOTAL'],7)
            self.assertEqual(rec.counts['LOCKED'],2)
            self.assertTrue((Path(temp)/'capture/summary.json').exists())

if __name__=='__main__':unittest.main(verbosity=2)

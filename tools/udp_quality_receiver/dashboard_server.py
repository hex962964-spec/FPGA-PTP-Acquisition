"""Local HTTP dashboard bridge for acquisition UDP; no third-party packages."""
import argparse
import collections
import csv
import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import threading
import time
from urllib.parse import parse_qs, urlparse
import webbrowser
from urllib.request import urlopen
from receiver import Recorder


class PacketStats:
    def __init__(self):
        self.total = 0
        self.counts = collections.Counter()
        self.recent = collections.deque(maxlen=2000)
        self.attention = collections.deque(maxlen=2000)
        self.events = collections.deque(maxlen=200)
        self.timeline = collections.deque(maxlen=1800)
        self.previous = {}
        self.first_at = self.last_at = None
        self.gaps = self.sequence_resets = self.sequence_missing = self.duplicates = self.timestamp_backwards = 0
        self.max_gap = 0.0
        self.last_quality = None

    def observe(self, row):
        row = dict(row)
        # JavaScript numbers cannot represent arbitrary uint64 values exactly.
        for key in ('timestamp',):
            if row.get(key, '') != '': row[key] = str(row[key])
        stamp = datetime.datetime.fromisoformat(row['received_at'].replace('Z', '+00:00')).timestamp()
        quality = row.get('time_quality') or 'MALFORMED'
        self.total += 1
        self.counts[quality] += 1
        if self.first_at is None: self.first_at = stamp
        if self.last_at is not None:
            gap = stamp - self.last_at
            self.max_gap = max(self.max_gap, gap)
            if gap >= 1:
                self.gaps += 1
                self.events.append(dict(at=stamp, kind='gap', text=f'恢复收包；此前 {gap:.3f} 秒未收到数据（不能据此判断时间质量）'))
        if quality != self.last_quality:
            self.events.append(dict(at=stamp, kind='quality', text=f'包内质量：{self.last_quality or "开始"} → {quality}'))
            self.last_quality = quality
        if row.get('sequence', '') != '' and row.get('timestamp', '') != '':
            key = (row.get('peer_ip'), str(row.get('channel')))
            seq, timestamp = int(row['sequence']), int(row['timestamp'])
            prev = self.previous.get(key)
            if prev:
                delta = (seq - prev[0]) & 0xFFFFFFFF
                if delta == 0: self.duplicates += 1
                elif delta < 0x80000000: self.sequence_missing += delta - 1
                else:
                    self.sequence_resets += 1
                    self.events.append(dict(at=stamp, kind='sequence', text=f'序号回退／重启：{prev[0]} → {seq}；不等于已确认整板复位'))
                if timestamp <= prev[1]: self.timestamp_backwards += 1
            self.previous[key] = (seq, timestamp)
        second = int(stamp)
        if self.timeline and second > self.timeline[-1]['second']:
            for missing in range(max(self.timeline[-1]['second']+1, second-1799), second):
                self.timeline.append(dict(second=missing, packets=0))
        if not self.timeline or self.timeline[-1]['second'] != second:
            self.timeline.append(dict(second=second, packets=0))
        self.timeline[-1]['packets'] += 1
        self.last_at = stamp
        self.recent.append(row)
        if quality != "LOCKED": self.attention.append(row)

    def snapshot(self):
        return dict(total=self.total, counts=dict(self.counts), last=self.recent[-1] if self.recent else None,
                    recent=list(reversed(self.recent)), attention_recent=list(reversed(self.attention)),
                    cache_limit=2000, events=list(reversed(self.events)), timeline=list(self.timeline),
                    first_at=self.first_at, last_at=self.last_at, duration_s=(self.last_at-self.first_at) if self.total else 0,
                    gap_count=self.gaps, max_gap_s=self.max_gap, sequence_resets=self.sequence_resets,
                    sequence_missing=self.sequence_missing, duplicates=self.duplicates,
                    timestamp_backwards=self.timestamp_backwards)


class CaptureManager:
    def __init__(self, captures, bind='192.168.0.3', port=6000, source='192.168.0.14'):
        self.captures, self.bind, self.port, self.source = captures, bind, port, source
        self.lock = threading.RLock()
        self.stats = PacketStats()
        self.stop_event = threading.Event()
        self.thread = None
        self.directory = None
        self.error = ''
        self.running = False
        self.ignored = 0

    def start(self):
        with self.lock:
            if self.running: return
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
                sock.bind((self.bind, self.port))
                sock.settimeout(.2)
                directory = self.captures / ('capture_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
                self.captures.mkdir(parents=True, exist_ok=True)
                recorder = Recorder(directory)
            except Exception:
                sock.close()
                raise
            self.directory, self.stats = directory, PacketStats()
            self.error, self.ignored = '', 0
            self.stop_event.clear()
            self.running = True
            self.thread = threading.Thread(target=self._receive, args=(sock, recorder), daemon=True)
            self.thread.start()

    def _receive(self, sock, recorder):
        flushed = time.monotonic()
        try:
            while not self.stop_event.is_set():
                try: data, peer = sock.recvfrom(65535)
                except socket.timeout: continue
                if self.source and peer[0] != self.source:
                    with self.lock: self.ignored += 1
                    continue
                with self.lock:
                    row = recorder.record(data, peer)
                    self.stats.observe(row)
                    if time.monotonic()-flushed >= 1:
                        recorder.flush()
                        flushed = time.monotonic()
        except Exception as exc:
            with self.lock: self.error = str(exc)
        finally:
            sock.close()
            with self.lock:
                try: recorder.close()
                except Exception as exc: self.error = str(exc)
                self.running = False

    def stop(self):
        self.stop_event.set()
        thread = self.thread
        if thread: thread.join(timeout=5)
        if thread and thread.is_alive(): raise RuntimeError('接收线程尚未停止，请稍后重试')

    def snapshot(self):
        with self.lock:
            result = self.stats.snapshot()
            result.update(running=self.running, error=self.error, capture=self.directory.name if self.directory else None,
                          bind=self.bind, port=self.port, source=self.source, ignored=self.ignored,
                          idle_s=max(0, time.time()-self.stats.last_at) if self.stats.last_at else None,
                          now=time.time(), mode='live')
            return result

    def archive_path(self, name):
        if not name or Path(name).name != name or not name.startswith('capture_'): raise ValueError('无效记录名称')
        path = (self.captures/name).resolve()
        if path.parent != self.captures.resolve() or not (path/'packets.csv').is_file(): raise ValueError('记录不存在')
        return path

    def archives(self):
        if not self.captures.exists(): return []
        return [p.name for p in sorted(self.captures.glob('capture_*'), reverse=True)
                if p.is_dir() and (p/'packets.csv').is_file()][:200]

    def read_archive(self, name):
        path = self.archive_path(name)
        with self.lock:
            if self.running and path == self.directory: raise ValueError('请先停止保存，或选择已结束的历史记录')
        stats = PacketStats()
        with (path/'packets.csv').open(encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream): stats.observe(row)
        result = stats.snapshot()
        result.update(mode='archive', capture=name, running=False, idle_s=None, error='')
        summary = path/'summary.json'
        if summary.exists():
            try: result['summary_matches'] = json.loads(summary.read_text(encoding='utf-8-sig'))['counts']['TOTAL'] == stats.total
            except (ValueError, KeyError): result['summary_matches'] = False
        return result


class DashboardHTTPServer(ThreadingHTTPServer):
    # Windows SO_REUSEADDR can allow two listeners on the same address. A local
    # dashboard must own its endpoint exclusively, otherwise state is ambiguous.
    allow_reuse_address = False
    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def make_handler(manager, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def _allowed(self):
            allowed = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
            return self.headers.get('Host') in allowed
        def _json(self, value, status=200):
            raw = json.dumps(value, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        def do_GET(self):
            if not self._allowed(): return self._json({'error':'不允许的主机'},403)
            url = urlparse(self.path)
            query = parse_qs(url.query)
            try:
                if url.path == '/':
                    raw = page.read_bytes()
                    self.send_response(200)
                    self.send_header('Content-Type','text/html; charset=utf-8')
                    self.send_header('Content-Length',str(len(raw)))
                    self.end_headers(); self.wfile.write(raw)
                elif url.path == '/api/ping': self._json(dict(app='fpga_ptp_udp_dashboard',page=str(page.resolve())))
                elif url.path == '/api/state': self._json(manager.snapshot())
                elif url.path == '/api/captures': self._json(manager.archives())
                elif url.path == '/api/archive': self._json(manager.read_archive(query.get('name',[''])[0]))
                elif url.path == '/api/download':
                    path = manager.archive_path(query.get('name',[''])[0])
                    with manager.lock:
                        if manager.running and path == manager.directory: raise ValueError('请先停止保存再导出当前记录')
                    file = path/'packets.csv'
                    self.send_response(200)
                    self.send_header('Content-Type','text/csv; charset=utf-8')
                    self.send_header('Content-Disposition','attachment; filename="packets.csv"')
                    self.send_header('Content-Length',str(file.stat().st_size))
                    self.end_headers()
                    with file.open('rb') as stream:
                        while True:
                            block = stream.read(65536)
                            if not block: break
                            self.wfile.write(block)
                else: self._json({'error':'路径不存在'},404)
            except (ValueError, OSError) as exc: self._json({'error':str(exc)},400)
        def do_POST(self):
            origin = self.headers.get('Origin')
            if not self._allowed() or (origin and origin not in (f'http://127.0.0.1:{self.server.server_port}', f'http://localhost:{self.server.server_port}')):
                return self._json({'error':'仅允许本机同源操作'},403)
            try:
                if self.path == '/api/start': manager.start()
                elif self.path == '/api/stop': manager.stop()
                else: return self._json({'error':'路径不存在'},404)
                self._json(manager.snapshot())
            except (OSError, RuntimeError) as exc: self._json({'error':str(exc)},400)
    return Handler


def main():
    parser = argparse.ArgumentParser(description='FPGA串口+UDP整合上位机（本机服务）')
    parser.add_argument('--page',type=Path,default=Path(__file__).resolve().parents[2]/'上位机v1.5.html')
    parser.add_argument('--captures',type=Path,default=Path(__file__).resolve().parent/'captures')
    parser.add_argument('--bind',default='192.168.0.3')
    parser.add_argument('--udp-port',type=int,default=6000)
    parser.add_argument('--http-port',type=int,default=8765)
    parser.add_argument('--open',action='store_true')
    args=parser.parse_args()
    if not args.page.is_file(): parser.error('网页不存在：'+str(args.page))
    manager=CaptureManager(args.captures,args.bind,args.udp_port)
    url=f'http://127.0.0.1:{args.http_port}/'
    try:
        server=DashboardHTTPServer(('127.0.0.1',args.http_port),make_handler(manager,args.page))
    except OSError:
        try:
            with urlopen(url+'api/ping',timeout=2) as response: existing=json.load(response)
            if existing.get('app') != 'fpga_ptp_udp_dashboard' or existing.get('page') != str(args.page.resolve()): raise ValueError('另一服务或另一备份版本占用此端口')
        except Exception:
            raise RuntimeError('本机网页端口被占用；请关闭另一服务，或用 --http-port 指定其他端口') from None
        print('整合上位机服务已运行：'+url,flush=True)
        if args.open: webbrowser.open(url)
        return
    server.daemon_threads=True
    url=f'http://127.0.0.1:{server.server_port}/'
    print('整合上位机：'+url,flush=True)
    print('在网页点击“开始UDP记录”；关闭网页不会停止接收，请先点击“停止并保存”。Ctrl+C关闭服务。',flush=True)
    if args.open: webbrowser.open(url)
    try: server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt: pass
    finally: manager.stop(); server.server_close()


if __name__ == '__main__': main()

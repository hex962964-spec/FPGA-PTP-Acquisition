"""Independent presentation preview: serves a page and GET-only UDP snapshots."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
from urllib.request import urlopen
from urllib.error import URLError,HTTPError
import webbrowser

class PreviewServer(ThreadingHTTPServer):
    allow_reuse_address=False
    daemon_threads=True
    def server_bind(self):
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        super().server_bind()

def make_handler(page,upstream_port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def send_json(self,data,status=200):
            self.send_bytes(json.dumps(data,ensure_ascii=False).encode('utf-8'),'application/json; charset=utf-8',status)
        def send_bytes(self,body,mime,status=200):
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
        def allowed(self):return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')
        def do_GET(self):
            if not self.allowed():return self.send_json({'error':'本预览仅允许本机访问'},403)
            if self.path=='/':return self.send_bytes(page.read_bytes(),'text/html; charset=utf-8')
            if self.path=='/api/ping':return self.send_json({'app':'ptp_presentation_preview','page':str(page.resolve()),'read_only':True})
            if self.path=='/api/state':
                try:
                    with urlopen(f'http://127.0.0.1:{upstream_port}/api/state',timeout=5) as response:raw=response.read()
                    data=json.loads(raw);data['preview_read_only']=True
                    return self.send_json(data)
                except (URLError,TimeoutError,ValueError,OSError):return self.send_json({'error':'正式UDP服务不可用；可以先查看样式演示场景'},503)
            return self.send_json({'error':'预览不提供此操作'},404)
        def do_POST(self):return self.send_json({'error':'只读预览不启停正式采集'},403)
    return Handler

def main():
    parser=argparse.ArgumentParser(description='比赛展示预览，不改正式页面，不占串口或UDP端口')
    parser.add_argument('--page',type=Path,default=Path(__file__).resolve().parents[2]/'上位机演示预览.html')
    parser.add_argument('--port',type=int,default=8767)
    parser.add_argument('--upstream-port',type=int,default=8765)
    parser.add_argument('--open',action='store_true');args=parser.parse_args()
    if not args.page.is_file():parser.error('预览HTML不存在')
    url=f'http://127.0.0.1:{args.port}/'
    try:server=PreviewServer(('127.0.0.1',args.port),make_handler(args.page,args.upstream_port))
    except OSError:
        try:
            with urlopen(url+'api/ping',timeout=2) as response:data=json.load(response)
            if data.get('app')!='ptp_presentation_preview' or data.get('page')!=str(args.page.resolve()):raise ValueError()
        except Exception:raise RuntimeError('预览端口被其他服务占用') from None
        if args.open:webbrowser.open(url)
        print('PREVIEW_ALREADY_RUNNING='+url,flush=True);return
    url=f'http://127.0.0.1:{server.server_port}/';print('PREVIEW_URL='+url,flush=True)
    if args.open:webbrowser.open(url)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()

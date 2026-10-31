"""Приёмник кадров для записи ролика взрыв-схемы: python tools/explode/record.py <папка> [порт]
Страница http://localhost:8770/?record=1 шлёт PNG-кадры POST /frame?i=N, в конце POST /done."""
import os, sys
from http.server import HTTPServer, BaseHTTPRequestHandler
OUT = sys.argv[1]; PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8771
os.makedirs(OUT, exist_ok=True)
class H(BaseHTTPRequestHandler):
    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*'); self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS'); self.send_header('Access-Control-Allow-Headers', '*')
    def do_OPTIONS(self): self.send_response(204); self._cors(); self.end_headers()
    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0)); data = self.rfile.read(n)
        if self.path.startswith('/frame'):
            i = int(self.path.split('i=')[1]); open(os.path.join(OUT, f'f{i:05d}.png'), 'wb').write(data)
        elif self.path.startswith('/done'):
            open(os.path.join(OUT, 'DONE'), 'w').write('ok')
        self.send_response(200); self._cors(); self.end_headers()
    def log_message(self, *a): pass
HTTPServer(('127.0.0.1', PORT), H).serve_forever()

#!/usr/bin/env python3
"""本地启动 PolyTrack 网页版：python3 serve.py [端口]"""
import http.server, socketserver, sys, os, webbrowser
os.chdir(os.path.dirname(os.path.abspath(__file__)))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
class H(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      '.wasm': 'application/wasm', '.js': 'text/javascript',
                      '.glb': 'model/gltf-binary', '.track': 'application/octet-stream'}
socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("127.0.0.1", PORT), H) as httpd:
    url = f"http://localhost:{PORT}/"
    print("PolyTrack 已启动：", url, "（Ctrl+C 退出）")
    webbrowser.open(url)
    try: httpd.serve_forever()
    except KeyboardInterrupt: pass

#!/usr/bin/env python3
from http.server import HTTPServer, SimpleHTTPRequestHandler, test
import sys

class CORSRequestHandler (SimpleHTTPRequestHandler):
    def end_headers (self):
        self.send_header('Cross-Origin-Embedder-Policy', 'require-corp')
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        #self.send_header('Access-Control-Allow-Origin', '*')
        SimpleHTTPRequestHandler.end_headers(self)

# ensure proper mime types are set for all used file-types
# (see issue #2 https://github.com/fraunhoferhhi/vvdecWebPlayer/issues/2#issuecomment-1049816809)
CORSRequestHandler.extensions_map['.js']    = 'application/javascript'
CORSRequestHandler.extensions_map['.json']  = 'application/json'
CORSRequestHandler.extensions_map['.wasm']  = 'application/wasm'
CORSRequestHandler.extensions_map['.css']   = 'text/css'
CORSRequestHandler.extensions_map['.svg']   = 'image/svg+xml'
CORSRequestHandler.extensions_map['.png']   = 'image/png'
CORSRequestHandler.extensions_map['.woff2'] = 'font/woff2'


if __name__ == '__main__':
    test(CORSRequestHandler, HTTPServer, bind='127.0.0.1', port=int(sys.argv[1]) if len(sys.argv) > 1 else 8000)

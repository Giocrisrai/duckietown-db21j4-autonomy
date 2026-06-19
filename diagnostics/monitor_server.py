"""Servidor de monitoreo de salud por RED (HTTP JSON, stdlib, sin dependencias).

Permite conectarse al robot por la red y ver la salud de los componentes EN VIVO
mientras corre. Sirve get_payload() como JSON en cualquier ruta (p. ej. /health).
"""
import json


def health_payload(monitor, extra=None):
    """Payload JSON-serializable con el estado de salud para exponer por red."""
    payload = {
        "overall": monitor.overall_status(),
        "components": dict(monitor._status),
        "updates": monitor.updates,
    }
    if extra:
        payload.update(extra)
    return payload


def run_monitor_server(get_payload, host="0.0.0.0", port=8080):
    """Levanta un servidor HTTP que sirve get_payload() como JSON. Robot-side."""
    from http.server import BaseHTTPRequestHandler, HTTPServer

    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps(get_payload()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    HTTPServer((host, port), _Handler).serve_forever()

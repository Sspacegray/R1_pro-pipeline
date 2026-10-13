"""PID-scoped, acknowledged controls for resident perception workers."""
import json
import os
import time
from pathlib import Path

ROOT = Path('/home/pc10/.cache/r1pro_resident_control')

def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(value), encoding='utf-8')
    temp.replace(path)

class ResidentControl:
    def __init__(self):
        self.pid = os.getpid()
        self.started = time.time_ns()
        self.last_nonce = ''
        self.path = ROOT / str(self.pid)

    def poll(self):
        try:
            value = json.loads(self.path.with_suffix('.command').read_text())
        except (OSError, ValueError):
            return None
        nonce = str(value.get('nonce', ''))
        if not nonce or nonce == self.last_nonce or int(value.get('issued_ns', 0)) < self.started:
            return None
        self.last_nonce = nonce
        return value

    def ack(self, command, **fields):
        write_json(self.path.with_suffix('.ack'), dict(nonce=command['nonce'], pid=self.pid, **fields))

def submit_worker(process, action, payload=None):
    if process is None or process.poll() is not None:
        raise RuntimeError('resident worker is not running')
    path = ROOT / str(process.pid)
    nonce = str(time.time_ns())
    write_json(path.with_suffix('.command'), dict(action=action, payload=payload or {}, nonce=nonce, issued_ns=time.time_ns()))
    return nonce

def wait_worker(process, nonce, action, timeout_s=5.0, cancelled=None):
    path = ROOT / str(process.pid)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if cancelled and cancelled():
            raise RuntimeError('resident command cancelled')
        if process.poll() is not None:
            raise RuntimeError('resident worker exited')
        try:
            ack = json.loads(path.with_suffix('.ack').read_text())
            if ack.get('nonce') == nonce:
                if not ack.get('ok'):
                    raise RuntimeError(ack.get('error') or 'resident command failed')
                return ack
        except (OSError, ValueError):
            pass
        time.sleep(0.05)
    raise RuntimeError('resident ' + action + ' acknowledgement timeout')

def command_worker(process, action, payload=None, timeout_s=5.0, cancelled=None):
    nonce = submit_worker(process, action, payload)
    return wait_worker(process, nonce, action, timeout_s, cancelled)

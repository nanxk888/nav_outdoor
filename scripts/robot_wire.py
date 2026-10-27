"""Our paired UDP protocol, NOT wire-compatible with the S1 binary receiver."""
import hashlib
import hmac
import json
import math
import time


def encode(value, key):
    body = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return json.dumps({'body': body.decode(), 'mac': hmac.new(key, body, hashlib.sha256).hexdigest()}).encode()


def decode(packet, key):
    if len(packet) > 4096:
        raise ValueError('oversize')
    envelope = json.loads(packet)
    if not isinstance(envelope, dict) or not isinstance(envelope.get('body'), str) or not isinstance(envelope.get('mac'), str):
        raise ValueError('invalid envelope')
    body = envelope['body'].encode()
    if not hmac.compare_digest(envelope['mac'], hmac.new(key, body, hashlib.sha256).hexdigest()):
        raise ValueError('bad signature')
    value = json.loads(body)
    if not isinstance(value, dict):
        raise ValueError('invalid body')
    stamp = value['time']
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or abs(time.time()-stamp) > .25:
        raise ValueError('stale packet or clocks not synchronized')
    return value


def velocity(v, w):
    if any(type(x) not in (float, int) or not math.isfinite(x) for x in (v, w)):
        raise ValueError('invalid velocity')
    if not 0 <= v <= 1. or abs(w) > .5:
        raise ValueError('outside commissioning limits')
    return v, w

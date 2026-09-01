"""PolicyServer wire protocol — the ONLY contact surface between the framework
and any policy backbone.

Request : {"state": f32[state_dim], "images": {name: u8 HWC}, "task": str}
Response: f32[chunk, action_dim]   (denormalised action chunk)

Framing: 8-byte big-endian length prefix + pickle.  Any model that can serve
this protocol plugs into the whole stack (takeover clients, eval pipelines)
with zero downstream changes.
"""
import pickle
import struct


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        c = conn.recv(n - len(buf))
        if not c:
            return None
        buf += c
    return buf


def recv_msg(conn):
    hdr = recv_exact(conn, 8)
    if hdr is None:
        return None
    (ln,) = struct.unpack(">Q", hdr)
    return pickle.loads(recv_exact(conn, ln))


def send_msg(conn, obj):
    p = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    conn.sendall(struct.pack(">Q", len(p)) + p)

"""
SCARA Pendant Server
Bridges the HTML pendant (WebSocket) to the ESP32 (UDP)

Install: pip install fastapi uvicorn websockets
Run:     uvicorn server:app --host 0.0.0.0 --port 8000 --reload
"""

import asyncio
import json
import socket
import time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ── Config ────────────────────────────────────────────────────────────
ESP32_IP   = "192.168.124.195"   # ← change to your ESP32's IP 192.168.84.195
ESP32_PORT = 5005               # UDP port on ESP32
LISTEN_PORT = 5006              # UDP port this server listens on

# ── Shared state ──────────────────────────────────────────────────────
state = {
    "joints":           {"j1": 0.0, "j2": 0.0, "j3": 150.0, "j4": 0.0},
    "joint_velocities": {"j1": 0.0, "j2": 0.0, "j3": 0.0,   "j4": 0.0},
    "estop": False,
    "mode": "joint",
    "connected": False,
}

# active WebSocket clients
clients: list[WebSocket] = []

# UDP socket (non-blocking)
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.setblocking(False)
udp_sock.bind(("0.0.0.0", LISTEN_PORT))

# ── UDP helpers ───────────────────────────────────────────────────────

def send_to_robot(cmd: dict):
    """Send a JSON command to the ESP32 over UDP."""
    try:
        data = json.dumps(cmd).encode()
        udp_sock.sendto(data, (ESP32_IP, ESP32_PORT))
    except Exception as e:
        print(f"[UDP send error] {e}")

async def udp_receiver():
    """Background task: receive state updates from ESP32 and push to all WS clients."""
    loop = asyncio.get_event_loop()
    while True:
        try:
            data, _ = udp_sock.recvfrom(1024)
            msg = json.loads(data.decode())
            # update shared state
            if "j1" in msg:
                state["joints"]["j1"] = msg.get("j1", 0.0)
                state["joints"]["j2"] = msg.get("j2", 0.0)
                state["joints"]["j3"] = msg.get("j3", 0.0)
                state["joints"]["j4"] = msg.get("j4", 0.0)
            if "j1v" in msg:
                state["joint_velocities"]["j1"] = msg.get("j1v", 0.0)
                state["joint_velocities"]["j2"] = msg.get("j2v", 0.0)
                state["joint_velocities"]["j3"] = msg.get("j3v", 0.0)
                state["joint_velocities"]["j4"] = msg.get("j4v", 0.0)
            state["connected"] = True
            # broadcast to all pendant clients
            payload = json.dumps(state)
            for ws in clients.copy():
                try:
                    await ws.send_text(payload)
                except:
                    clients.remove(ws)
        except BlockingIOError:
            await asyncio.sleep(0.01)
        except Exception as e:
            print(f"[UDP recv error] {e}")
            await asyncio.sleep(0.1)

# ── Startup ───────────────────────────────────────────────────────────

@app.on_event("startup")
async def startup():
    asyncio.create_task(udp_receiver())
    print(f"[Server] Listening for ESP32 on UDP :{LISTEN_PORT}")
    print(f"[Server] Sending commands to ESP32 at {ESP32_IP}:{ESP32_PORT}")

# ── REST endpoints (used by pendant) ─────────────────────────────────

@app.get("/state")
async def get_state():
    return state

@app.post("/move_tcp")
async def move_tcp(body: dict):
    send_to_robot({"cmd": "move_tcp",
                   "x": body.get("x", 0),
                   "y": body.get("y", 0),
                   "z": body.get("z", 0),
                   "alpha": body.get("alpha", 0)})
    return {"ok": True, "state": state}

@app.post("/move_joint")
async def move_joint(body: dict):
    send_to_robot({"cmd": "move_joint",
                   "j1": body.get("j1", state["joints"]["j1"]),
                   "j2": body.get("j2", state["joints"]["j2"]),
                   "j3": body.get("j3", state["joints"]["j3"]),
                   "j4": body.get("j4", state["joints"]["j4"])})
    return {"ok": True, "state": state}

@app.post("/jog")
async def jog(body: dict):
    send_to_robot({"cmd": "jog",
                   "axis":      body.get("axis"),
                   "direction": body.get("direction", 1),
                   "active":    body.get("active", False),
                   "step":      body.get("step", 5)})
    return {"ok": True}

@app.post("/estop")
async def estop():
    state["estop"] = True
    send_to_robot({"cmd": "estop"})
    return {"ok": True, "state": state}

@app.post("/estop/reset")
async def estop_reset():
    state["estop"] = False
    send_to_robot({"cmd": "estop_reset"})
    return {"ok": True, "state": state}

@app.post("/home")
async def home():
    send_to_robot({"cmd": "home"})
    return {"ok": True, "state": state}

@app.post("/mode")
async def set_mode(body: dict):
    state["mode"] = body.get("mode", "joint")
    send_to_robot({"cmd": "mode", "mode": state["mode"]})
    return {"ok": True}

# ── WebSocket (live updates to pendant) ──────────────────────────────

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    print(f"[WS] Client connected ({len(clients)} total)")
    try:
        # send current state immediately on connect
        await ws.send_text(json.dumps(state))
        while True:
            # keep connection alive, pendant sends nothing but we need to detect close
            await ws.receive_text()
    except WebSocketDisconnect:
        clients.remove(ws)
        print(f"[WS] Client disconnected ({len(clients)} total)")
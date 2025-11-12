# gcs.py
import asyncio
import socket
import os
import time
from datetime import datetime
from pymavlink import mavutil
from aiohttp import web

# === CONFIGURATION ===
GCS_UDP_PORT = 14550       # Port to receive telemetry from MAVProxy
DRONE_IP = "YOUR_DRONE_IP" # Drone IP
DRONE_MAVLINK_PORT = 14550
WEB_PORT = 3000

os.makedirs("logs", exist_ok=True)
LOG_FILE = f"logs/flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.tlog"

# === TELEMETRY STATE ===
telemetry = {
    "roll": 0, "pitch": 0, "yaw": 0,
    "lat": 0, "lon": 0, "alt": 0,
    "spd": 0, "vsi": 0,
    "volt": 0, "curr": 0,
    "gps": 0, "satellites": 0, "hdop": 0,
    "mode": 0, "armed": False,
    "vibe": 0,
    "connected": False
}

websocket_clients = set()
last_heartbeat = 0
drone_system_id = 1
drone_component_id = 1

tlog_file = open(LOG_FILE, "wb")

# === MAVLINK RECEIVER USING parse_buffer() ===
async def mavlink_receiver():
    global last_heartbeat, drone_system_id, drone_component_id, telemetry
    print(f"📡 Listening for MAVLink telemetry on UDP:{GCS_UDP_PORT}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", GCS_UDP_PORT))
    sock.setblocking(False)
    loop = asyncio.get_event_loop()
    
    # MAVLink parser
    mparser = mavutil.mavlink.MAVLink(None)

    while True:
        try:
            data, _ = await loop.sock_recvfrom(sock, 2048)
        except Exception:
            await asyncio.sleep(0.1)
            continue

        # Log raw telemetry
        tlog_file.write(data)
        tlog_file.flush()

        # Parse all complete MAVLink messages in the packet
        try:
            msgs = mparser.parse_buffer(data)
            for msg in msgs:
                telemetry["connected"] = True
                if drone_system_id == 1 and msg.get_srcSystem() != 0:
                    drone_system_id = msg.get_srcSystem()
                    drone_component_id = msg.get_srcComponent()
                if msg.get_type() == "HEARTBEAT":
                    last_heartbeat = time.time()
                    telemetry.update({
                        "mode": msg.custom_mode,
                        "armed": bool(msg.base_mode & 128)
                    })
                process_message(msg)
        except Exception as e:
            print(f"[Parse] {e}")

        if time.time() - last_heartbeat > 5:
            telemetry["connected"] = False

def process_message(msg):
    t = msg.get_type()
    if t == "ATTITUDE":
        telemetry.update({
            "roll": round(msg.roll * 180 / 3.14159, 1),
            "pitch": round(msg.pitch * 180 / 3.14159, 1),
            "yaw": round(msg.yaw * 180 / 3.14159, 1)
        })
    elif t == "GLOBAL_POSITION_INT":
        telemetry.update({
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.relative_alt / 1000.0
        })
    elif t == "VFR_HUD":
        telemetry.update({
            "spd": round(msg.groundspeed, 1),
            "vsi": round(msg.climb, 1)
        })
    elif t == "BATTERY_STATUS":
        volt = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0
        curr = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
        telemetry.update({
            "volt": round(volt, 2),
            "curr": round(curr, 2)
        })
    elif t == "GPS_RAW_INT":
        telemetry.update({
            "gps": msg.fix_type,
            "satellites": msg.satellites_visible,
            "hdop": getattr(msg, "eph", 0) / 100.0 if getattr(msg, "eph", 0) != 65535 else 0
        })
    elif t == "VIBRATION":
        vibe = (msg.vibration_x + msg.vibration_y + msg.vibration_z) / 3.0
        telemetry.update({"vibe": round(vibe, 3)})

# === WEBSOCKET BROADCAST ===
async def broadcast_telemetry():
    if not websocket_clients:
        return
    dead = set()
    for ws in websocket_clients:
        try:
            await ws.send_json(telemetry)
        except:
            dead.add(ws)
    websocket_clients.difference_update(dead)

async def periodic_broadcast():
    while True:
        await broadcast_telemetry()
        await asyncio.sleep(0.1)  # 10 Hz

# === GCS HEARTBEAT ===
async def send_heartbeat():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        try:
            hb_mav = mavutil.mavlink_connection("udpout:127.0.0.1:14557")
            hb_mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            buf = hb_mav.mav.buf
            if buf:
                sock.sendto(buf, (DRONE_IP, DRONE_MAVLINK_PORT))
        except Exception as e:
            print(f"[HB] {e}")
        await asyncio.sleep(1)

# === COMMANDS ===
async def send_command(cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        cmd_mav = mavutil.mavlink_connection("udpout:127.0.0.1:14558")
        cmd_mav.mav.command_long_send(
            drone_system_id,
            drone_component_id,
            cmd,
            0, p1, p2, p3, p4, p5, p6, p7
        )
        buf = cmd_mav.mav.buf
        if buf:
            sock.sendto(buf, (DRONE_IP, DRONE_MAVLINK_PORT))
    except Exception as e:
        print(f"[CMD] {e}")

async def rtl_handler(request):
    await send_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
    return web.json_response({"status": "RTL sent"})

async def set_home_handler(request):
    await send_command(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 1)
    return web.json_response({"status": "Home set"})

# === WEB SERVER ===
async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    websocket_clients.add(ws)
    try:
        async for msg in ws:
            pass
    finally:
        websocket_clients.discard(ws)
    return ws

async def index_handler(request):
    return web.FileResponse('./public/index.html')

# === MAIN ENTRY ===
async def main():
    app = web.Application()
    app.router.add_get('/ws', websocket_handler)
    app.router.add_post('/rtl', rtl_handler)
    app.router.add_post('/set-home', set_home_handler)
    app.router.add_get('/', index_handler)
    app.router.add_static('/', path='./public')

    # Start tasks
    asyncio.create_task(mavlink_receiver())
    asyncio.create_task(send_heartbeat())
    asyncio.create_task(periodic_broadcast())
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', WEB_PORT)
    await site.start()

    print(f"✅ GCS ready!")
    print(f"🌐 Web UI: http://localhost:{WEB_PORT}")
    print(f"📡 Telemetry UDP port: {GCS_UDP_PORT}")
    print(f"📹 Video stream: http://{DRONE_IP}:8080/?action=stream")
    print(f"\n📝 DRONE SETUP (MAVProxy):")
    print(f"mavproxy.py --master=/dev/ttyAMA0 --baudrate=57600 --out=udp:172.25.187.110:14555 --streamrate=10")
    
    await asyncio.Event().wait()  # Keep running

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        tlog_file.close()
        print("📁 Flight log saved.")


# FPV-GS
Ground Station for FPV with Ardupilot or PX4

FPV Ground Station (GCS) Development Update
Excited to share progress on my latest project — an FPV Ground Station (GCS) built for ArduPilot and PX4 platforms, integrating real-time telemetry over GSM with a modern FPV video overlay interface.
This system brings together performance, clarity, and connectivity with:
* Dynamic HUD — moving horizon, compass, and heading arrow for accurate attitude awareness.
* Live telemetry streaming through WebSocket with status monitoring and UTM coordinates.
 * FPV overlay with transparent UI for a clean flight experience.
 * Map tracking with real-time drone position visualization.

The GCS currently runs fully stable with MAVLink and GSM telemetry, ensuring robust situational data in any environment.
 Next up — integration for radio telemetry, TBS Crossfire, and Betaflight firmware, extending compatibility across both professional UAV and FPV ecosystems.
This project aims to merge open-source flight control, AI-assisted situational awareness, and intuitive UX design into one cohesive FPV-GCS platform — accessible from any browser, anywhere in the field.

Clone this repository
```bash
git clone https://github.com/palacita135/FPV-GS.git
cd FPV-GS
python3 gcs.py
```

You need configure the gsc.py with your drone IP
```bash
# === CONFIGURATION ===
GCS_UDP_PORT = 14550       # Port to receive telemetry from MAVProxy
DRONE_IP = "YOUR_DRONE_IP" # Drone IP
DRONE_MAVLINK_PORT = 14550
WEB_PORT = 3000
```

At /public/index.html
```bash
Line 58
<img id="videoOverlay" src="http://DRONE_IP:8080/?action=stream"

Line 90
Cesium.Ion.defaultAccessToken='CESIUM_API_KEY';
```

HAPPY FLYING

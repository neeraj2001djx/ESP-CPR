import websocket
import json

ESP_IP = "192.168.31.71"   # ESP32 IP
PORT = 81

last_event_status = "NONE"

def on_message(ws, message):
    global last_event_status

    data = json.loads(message)
    depth = data["depth"]
    status = data["status"]

    # -------- CONTINUOUS OUTPUT --------
    # print(f"[LIVE] Depth: {depth:.2f} cm | Status: {status}")

    # -------- EVENT OUTPUT (ONLY ON LEFT/RIGHT/CENTER) --------
    if status != "NONE" and status != last_event_status:
        print(f"*** EVENT ***  Status: {status} | Depth: {depth:.2f} cm")

    last_event_status = status

def on_open(ws):
    print("Connected to ESP32 WebSocket")

def on_close(ws):
    print("Disconnected")

ws = websocket.WebSocketApp(
    f"ws://{ESP_IP}:{PORT}",
    on_message=on_message,
    on_open=on_open,
    on_close=on_close
)

ws.run_forever()

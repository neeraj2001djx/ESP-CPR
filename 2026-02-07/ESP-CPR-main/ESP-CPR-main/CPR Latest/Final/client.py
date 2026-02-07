import websocket
import json
import time

ESP_IP = "192.168.214.26"   # Your ESP32 IP
PORT = 81

last_event_status = "NONE"
last_count = 0


# =========================
# MESSAGE RECEIVED
# =========================
def on_message(ws, message):
    global last_event_status
    global last_count

    try:
        # Handle multiple JSON objects stuck together
        packets = message.replace('}{', '}|{').split('|')

        for pkt in packets:
            pkt = pkt.strip()

            if not pkt:
                continue

            if not pkt.startswith("{") or not pkt.endswith("}"):
                continue

            data = json.loads(pkt)

            # 🔥 MATCH ESP JSON FORMAT
            pressure = float(data.get("pressure", 0))
            status = str(data.get("status", "NONE"))
            count = int(data.get("count", 0))

            # ===== CONTINUOUS LIVE OUTPUT =====
            print(f"[LIVE] Pressure: {pressure:.2f} | Status: {status} | Count: {count}")

            # ===== EVENT OUTPUT (only when status changes) =====
            if status != "NONE" and status != last_event_status:
                print(f"\n*** EVENT *** Status: {status} | Pressure: {pressure:.2f} | Count: {count}\n")

            last_event_status = status
            last_count = count

    except Exception as e:
        print("RAW MESSAGE:", message)
        print("Error parsing message:", e)


# =========================
# CONNECTION OPEN
# =========================
def on_open(ws):
    print("Connected to ESP32 WebSocket")


# =========================
# CONNECTION CLOSE
# =========================
def on_close(ws, close_status_code, close_msg):
    print("Disconnected from ESP32")
    print("Reconnecting in 3 seconds...")
    time.sleep(3)
    start_client()


# =========================
# CONNECTION ERROR
# =========================
def on_error(ws, error):
    print("WebSocket Error:", error)


# =========================
# START CLIENT
# =========================
def start_client():
    ws = websocket.WebSocketApp(
        f"ws://{ESP_IP}:{PORT}",
        on_message=on_message,
        on_open=on_open,
        on_close=on_close,
        on_error=on_error
    )
    ws.run_forever()


# Run
start_client()

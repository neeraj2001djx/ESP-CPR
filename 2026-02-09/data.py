import websocket
import json
import time

ESP_IP = "192.168.31.163"
PORT = 82

def on_message(ws, message):
    try:
        data = json.loads(message)

        pressure = data.get("pressure")
        count = data.get("count")
        status = data.get("status")

        print(f"pressure={pressure}, count={count}, status={status}")

    except Exception as e:
        print("Invalid JSON:", message, e)

def on_error(ws, error):
    print("WebSocket error:", error)

def on_close(ws, close_status_code, close_msg):
    print("WebSocket closed")

def on_open(ws):
    print("Connected to ESP32 WebSocket")

if __name__ == "__main__":
    ws_url = f"ws://{ESP_IP}:{PORT}"
    print("Connecting to", ws_url)

    ws = websocket.WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )

    ws.run_forever()
import socket
import threading

# ==== UPDATE THIS to match your ESP32’s IP on the same WiFi ====
ESP32_IP   = "192.168.166.100"
ESP32_PORT = 5000

VALID_CODES = ("AUTHORIZED_CARD", "UNAUTHORIZED_CARD", "PHYSICAL_ALARM")

def send_status(status: str, timeout: float = 2.0) -> bool:
    """
    Connect to ESP32 at (ESP32_IP, ESP32_PORT), send one of the valid commands.
    Returns True if send succeeded, False on failure.
    """
    code = status.strip().upper()
    if code not in VALID_CODES:
        raise ValueError(f"send_status: status must be one of {VALID_CODES}")

    try:
        with socket.create_connection((ESP32_IP, ESP32_PORT), timeout=timeout) as sock:
            message = code + "\n"
            sock.sendall(message.encode("utf-8"))
        return True
    except Exception as e:
        print(f"[pi_sender] Failed to send '{code}': {e}")
        return False

def send_status_async(status: str, timeout: float = 2.0):
    """
    Sends status to ESP32 asynchronously to avoid blocking main thread.
    """
    threading.Thread(target=send_status, args=(status, timeout), daemon=True).start()

if __name__ == "__main__":
    # Quick sanity‐check:
    print("Sending UNAUTHORIZED_CARD →")
    send_status("UNAUTHORIZED_CARD")
    import time; time.sleep(3)
    print("Sending AUTHORIZED_CARD →")
    send_status("AUTHORIZED_CARD")
    time.sleep(3)
    print("Sending PHYSICAL_ALARM →")
    send_status("PHYSICAL_ALARM")
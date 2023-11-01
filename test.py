import requests
import json

telem = {
    "lon": 41,
    "lat": 3232,
    "alt": 0,
    "pitch": 0,
    "yaw": 0,
    "roll": 0
}
json_veri = json.dumps(telem)

server_url = "127.0.0.1:5000"

qr_packet = requests.post(
                "http://127.0.0.1:5000/telem_post_2", json=json_veri)
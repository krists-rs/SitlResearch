import requests

paths = [
    "/",
    "/video",
    "/stream",
    "/mjpeg",
    "/frame",
    "/camera",
    "/image",
    "/snapshot",
    "/live",
    "/feed",
    "/api",
    "/api/video",
    "/api/camera",
    "/api/stream",
    "/cam",
    "/cam/1",
    "/v1/video",
    "/v1/stream",
]

base = "url://127.0.0.1:8080"

for p in paths:
    url = base + p
    try:
        r = requests.get(url, timeout=3, stream=True)
        ctype = r.headers.get("Content-Type", "")
        print(f"{r.status_code:3}  {p:15}  {ctype}")
    except Exception as e:
        print(f"ERR  {p:15}  {e}")
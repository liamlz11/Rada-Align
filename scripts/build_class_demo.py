"""Build one offline HTML file of the guided tour, for presenting in class.

Runs the guided tour once in a headless Chrome/Edge against the live demo server and
records every simulation message and tour event (?record&trace). The recording, three.js
and the font are packed into a single HTML file that replays the tour step by step with
"next" / "back" buttons (or space / arrows / a presentation clicker). No Python and no
server are needed to present it; open the file in Chrome or Edge.

Start the demo first, then:
    python -m rada_align.psd                       (port 8001)
    python scripts/build_class_demo.py --out docs/media/RADA_Align_class_demo.html

SIMULATION: the file replays the simulator with assumed parameters, not hardware.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import gzip
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import websockets

ROOT = Path(__file__).resolve().parents[1]
STATIC = ROOT / "rada_align" / "psd" / "static"
BROWSERS = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    "google-chrome", "chromium", "chromium-browser", "microsoft-edge",
]


def find_browser() -> str:
    for b in BROWSERS:
        if os.path.isfile(b) or shutil.which(b):
            return b
    raise SystemExit("no Chrome or Edge found")


async def capture(url: str, timeout_s: float, width: int, height: int) -> list:
    port = 9334
    profile = tempfile.mkdtemp(prefix="rada_class_")
    proc = subprocess.Popen([
        find_browser(), "--headless=new", f"--remote-debugging-port={port}", f"--user-data-dir={profile}",
        f"--window-size={width},{height}", "--mute-audio", "--no-first-run",
        "--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--disable-background-timer-throttling",
        "--disable-renderer-backgrounding", "about:blank"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(100):
            try:
                tabs = json.load(urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list", timeout=1))
                page = next(t for t in tabs if t.get("type") == "page")
                break
            except Exception:
                time.sleep(0.2)
        else:
            raise SystemExit("browser did not start")
        async with websockets.connect(page["webSocketDebuggerUrl"], max_size=256 * 2**20) as ws:
            n = 0

            async def call(method: str, params: dict | None = None):
                nonlocal n
                n += 1
                me = n
                await ws.send(json.dumps({"id": me, "method": method, "params": params or {}}))
                while True:
                    msg = json.loads(await ws.recv())
                    if msg.get("id") == me:
                        if "error" in msg:
                            raise RuntimeError(msg["error"])
                        return msg["result"]

            async def js(expr: str):
                r = await call("Runtime.evaluate", {"expression": expr, "returnByValue": True, "awaitPromise": True})
                return r.get("result", {}).get("value")

            await call("Emulation.setDeviceMetricsOverride",
                       {"width": width, "height": height, "deviceScaleFactor": 1, "mobile": False})
            await call("Page.navigate", {"url": url})
            for _ in range(150):
                if await js("document.readyState === 'complete' && !!document.getElementById('tour')"):
                    break
                await asyncio.sleep(0.2)
            await asyncio.sleep(2.0)
            await js("document.getElementById('tour').click(), true")
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                await asyncio.sleep(2.0)
                if await js("!!window.__tourDone"):
                    break
                el = time.time() - t0
                if int(el) % 20 < 2:
                    print(f"  recording the tour... {el:.0f} s, {await js('window.__trace.length')} events", flush=True)
            else:
                raise SystemExit("timeout: the tour did not finish")
            if await js("window.__trace.some(e => e[1]==='c' && e[2] && e[2][0]==='עדיין לא חזר')"):
                print("WARNING: the run hit 'not back yet' (a disturbance did not recover in time)")
            total = await js("window.__trace.length")
            events = []
            for i in range(0, total, 400):
                events += json.loads(await js(f"JSON.stringify(window.__trace.slice({i},{i + 400}))"))
            return events
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except Exception:
            proc.kill()
        shutil.rmtree(profile, ignore_errors=True)


def thin(events: list, hz: float) -> list:
    """Keep simulation messages at most `hz` per second (the log lines of dropped messages
    move to the next kept one); every tour event is kept."""
    out, carry, last_t = [], [], -1e9
    min_dt = 1000.0 / hz
    for i, e in enumerate(events):
        if e[1] != "m":
            out.append(e)
            continue
        m = e[2]
        nxt = events[i + 1][1] if i + 1 < len(events) else None
        keep = e[0] - last_t >= min_dt or nxt in ("w", "c", "v", "s")
        if not keep:
            carry += m.get("events") or []
            continue
        if carry:
            m["events"] = carry + (m.get("events") or [])
            carry = []
        out.append(e)
        last_t = e[0]
    if carry:   # log lines of trailing dropped messages go to the last kept one
        last = next((e for e in reversed(out) if e[1] == "m"), None)
        if last is not None:
            last[2]["events"] = (last[2].get("events") or []) + carry
    return out


def b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode()


def build_html(events: list, out: Path) -> None:
    html = (STATIC / "demo.html").read_text(encoding="utf-8")
    # font: inline the woff2 files
    css = (STATIC / "vendor" / "fonts" / "fonts.css").read_text(encoding="utf-8")
    css = re.sub(r"url\(([^)]+\.woff2)\)",
                 lambda m: f"url(data:font/woff2;base64,{b64(STATIC / 'vendor' / 'fonts' / m.group(1).strip(chr(39) + chr(34)).split('/')[-1])})",
                 css)
    html = html.replace('<link href="/vendor/fonts/fonts.css" rel="stylesheet">', f"<style>{css}</style>")
    # three.js and OrbitControls as data: modules through the import map
    three = "data:text/javascript;base64," + b64(STATIC / "vendor" / "three" / "three.module.js")
    orbit = "data:text/javascript;base64," + b64(STATIC / "vendor" / "three" / "addons" / "controls" / "OrbitControls.js")
    imap = re.search(r'<script type="importmap">.*?</script>', html, re.S).group(0)
    html = html.replace(imap, '<script type="importmap">' + json.dumps({"imports": {
        "three": three, "three/addons/controls/OrbitControls.js": orbit}}) + "</script>")
    payload = base64.b64encode(gzip.compress(json.dumps(events, separators=(",", ":")).encode(), 9)).decode()
    html = html.replace('<script type="module">',
                        f'<script>window.__REPLAY_GZ__ = "{payload}";</script>\n<script type="module">', 1)
    html = re.sub(r"<title>.*?</title>", "<title>RADA-Align · הדגמה מודרכת</title>", html, count=1, flags=re.S)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--url", default="http://127.0.0.1:8001/")
    ap.add_argument("--out", default="docs/media/RADA_Align_class_demo.html")
    ap.add_argument("--hz", type=float, default=12.0, help="simulation messages kept per second")
    ap.add_argument("--timeout", type=float, default=900.0)
    a = ap.parse_args()
    sep = "&" if "?" in a.url else "?"
    url = f"{a.url}{sep}record=1&trace=1&gap=0&read=0.3"
    events = asyncio.run(capture(url, a.timeout, 1920, 1080))
    kept = thin(events, a.hz)
    build_html(kept, Path(a.out))
    stops = sum(1 for e in kept if e[1] == "w")
    print(f"wrote {a.out}: {Path(a.out).stat().st_size / 1e6:.1f} MB, {stops} steps, "
          f"{sum(1 for e in kept if e[1] == 'm')} of {sum(1 for e in events if e[1] == 'm')} messages kept")


if __name__ == "__main__":
    main()

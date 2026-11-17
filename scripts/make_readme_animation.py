"""Make the README animation (animated WebP) from the class file of the guided tour.

Opens docs/media/RADA_Align_class_demo.html in a headless Chrome/Edge, presses "next"
through every step (each step plays as recorded, then the caption is held for --hold
seconds), records the screen through the DevTools screencast and writes an animated
WebP with Pillow. No extra dependencies.

    python scripts/make_readme_animation.py --out docs/media/guided_tour.webp

SIMULATION: the animation shows the simulator with assumed parameters, not hardware.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import importlib.util
import io
import json
import shutil
import subprocess
import tempfile
import time
import urllib.request
from pathlib import Path

import websockets
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("build_class_demo", ROOT / "scripts" / "build_class_demo.py")
_bcd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_bcd)


async def grab(url: str, width: int, height: int, hold: float, last_hold: float) -> list[tuple[float, bytes]]:
    port = 9335
    profile = tempfile.mkdtemp(prefix="rada_anim_")
    proc = subprocess.Popen([
        _bcd.find_browser(), "--headless=new", f"--remote-debugging-port={port}", f"--user-data-dir={profile}",
        f"--window-size={width},{height}", "--hide-scrollbars", "--mute-audio", "--no-first-run",
        "--allow-file-access-from-files", "--enable-unsafe-swiftshader", "--use-angle=swiftshader",
        "--disable-background-timer-throttling", "--disable-renderer-backgrounding", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    frames: list[tuple[float, bytes]] = []
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
        async with websockets.connect(page["webSocketDebuggerUrl"], max_size=64 * 2**20) as ws:
            n = 0
            pending: dict[int, asyncio.Future] = {}

            async def call(method, params=None):
                nonlocal n
                n += 1
                fut = asyncio.get_running_loop().create_future()
                pending[n] = fut
                await ws.send(json.dumps({"id": n, "method": method, "params": params or {}}))
                return await fut

            async def reader():
                nonlocal n
                async for raw in ws:
                    msg = json.loads(raw)
                    if "id" in msg and msg["id"] in pending:
                        pending.pop(msg["id"]).set_result(msg.get("result", {}))
                    elif msg.get("method") == "Page.screencastFrame":
                        p = msg["params"]
                        frames.append((p["metadata"]["timestamp"], base64.b64decode(p["data"])))
                        n += 1
                        await ws.send(json.dumps({"id": n, "method": "Page.screencastFrameAck",
                                                  "params": {"sessionId": p["sessionId"]}}))

            task = asyncio.create_task(reader())

            async def js(expr):
                r = await call("Runtime.evaluate", {"expression": expr, "returnByValue": True})
                return r.get("result", {}).get("value")

            await call("Emulation.setDeviceMetricsOverride",
                       {"width": width, "height": height, "deviceScaleFactor": 1, "mobile": False})
            await call("Page.navigate", {"url": url})
            for _ in range(150):
                if await js("!!document.getElementById('rNext') && /שלב 1 /.test(document.getElementById('replayProg').textContent)"):
                    break
                await asyncio.sleep(0.2)
            await asyncio.sleep(1.5)
            await call("Page.startScreencast", {"format": "jpeg", "quality": 85, "maxWidth": width,
                                                "maxHeight": height, "everyNthFrame": 1})
            total = int(await js("+document.getElementById('replayProg').textContent.match(/מתוך (\\d+)/)[1]"))
            await asyncio.sleep(hold)                      # step 1 is on screen at load
            for k in range(2, total + 1):
                await js("document.getElementById('rNext').click()")
                for _ in range(600):                        # until the step has played
                    await asyncio.sleep(0.1)
                    if await js(f"document.getElementById('replayProg').textContent.startsWith('שלב {k} ')"):
                        break
                await asyncio.sleep(last_hold if k == total else hold)
                print(f"  step {k}/{total}, {len(frames)} frames", flush=True)
            await call("Page.stopScreencast")
            frames.append((time.time(), frames[-1][1]))     # close the last hold
            task.cancel()
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except Exception:
            proc.kill()
        shutil.rmtree(profile, ignore_errors=True)
    return frames


def write_webp(frames: list[tuple[float, bytes]], out: Path, fps: float, size: tuple[int, int], quality: int) -> int:
    """Resample the screencast (sent only on change) to a fixed rate, merge repeats."""
    t0, t1 = frames[0][0], frames[-1][0]
    step = 1.0 / fps
    imgs, durs, j = [], [], 0
    t = t0
    last_src = None
    while t <= t1:
        while j + 1 < len(frames) and frames[j + 1][0] <= t:
            j += 1
        src = frames[j][1]
        if src is last_src and durs:
            durs[-1] += step * 1000
        else:
            imgs.append(Image.open(io.BytesIO(src)).convert("RGB").resize(size, Image.LANCZOS))
            durs.append(step * 1000)
            last_src = src
        t += step
    out.parent.mkdir(parents=True, exist_ok=True)
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=[int(d) for d in durs],
                 loop=0, quality=quality, method=4)
    return len(imgs)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--html", default=str(ROOT / "docs" / "media" / "RADA_Align_class_demo.html"))
    ap.add_argument("--out", default="docs/media/guided_tour.webp")
    ap.add_argument("--hold", type=float, default=4.0, help="seconds each caption stays after its step")
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out-width", type=int, default=960)
    ap.add_argument("--quality", type=int, default=60)
    a = ap.parse_args()
    url = Path(a.html).resolve().as_uri()
    frames = asyncio.run(grab(url, a.width, a.height, a.hold, a.hold + 2))
    size = (a.out_width, round(a.out_width * a.height / a.width))
    n = write_webp(frames, Path(a.out), a.fps, size, a.quality)
    dur = frames[-1][0] - frames[0][0]
    print(f"wrote {a.out}: {Path(a.out).stat().st_size / 1e6:.1f} MB, {n} distinct frames, {dur:.0f} s, {size[0]}x{size[1]}")


if __name__ == "__main__":
    main()

"""The class file builder (scripts/build_class_demo.py): message thinning keeps every tour
event and every log line."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "build_class_demo", Path(__file__).resolve().parents[1] / "scripts" / "build_class_demo.py")
bcd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bcd)


def test_thin_keeps_tour_events_and_log_lines():
    ev = []
    for i in range(100):   # 100 messages, 10 ms apart, one log line each
        ev.append([i * 10, "m", {"t": i * 0.01, "events": [[i * 0.01, f"line {i}"]]}])
        if i == 50:
            ev.append([i * 10 + 1, "c", ["title", "text", ""]])
            ev.append([i * 10 + 2, "w", 1])
    out = bcd.thin(ev, hz=10)
    assert [e for e in out if e[1] != "m"] == [e for e in ev if e[1] != "m"]
    kept = [e for e in out if e[1] == "m"]
    assert len(kept) < 30
    lines = [x[1] for e in kept for x in e[2]["events"]]
    assert sorted(lines, key=lambda x: int(x.split()[1])) == [f"line {i}" for i in range(100)]

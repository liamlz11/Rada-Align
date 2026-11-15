import pytest
from fastapi.testclient import TestClient

from rada_align.config import load_config, with_only_sources
from rada_align.estimator import ERROR_AXES
from rada_align.ui.server import create_app
from rada_align.ui.session import Session

QUIET = with_only_sources(load_config(), ())


@pytest.fixture()
def rig():
    session = Session(QUIET, seed=1, distance_m=2.0)
    with TestClient(create_app(session, autostart=False)) as client:
        yield session, client


def _steps(session, seconds):
    message = None
    for _ in range(round(seconds / session.dt)):
        message = session.step_once()
    return message


def test_page_is_served_in_hebrew_rtl(rig):
    _, client = rig
    response = client.get("/")
    assert response.status_code == 200
    assert 'lang="he" dir="rtl"' in response.text
    for control in ("btnStart", "btnStop", "btnRealign", "btnHold", "btnReport"):
        assert f'id="{control}"' in response.text
    assert "cdn" not in response.text.lower()  # must work offline at the demo


def test_websocket_streams_the_documented_schema(rig):
    session, client = rig
    _steps(session, 0.5)
    with client.websocket_connect("/ws") as ws:
        message = ws.receive_json()
        assert message["state"] == "IDLE" and message["sim"] is True
        assert set(message["errors"]) == set(ERROR_AXES)
        for key in ("aligned", "aligned_with_bias", "hold", "actuators", "events", "truth", "tolerance_deg"):
            assert key in message

        assert client.post("/api/start").json() == {"ok": True}
        _steps(session, 1.0)
        states = set()
        for _ in range(10):
            states.add(ws.receive_json()["state"])
        assert states <= {"ACQUIRE", "ALIGN"} and states


def test_commands_drive_the_controller(rig):
    session, client = rig
    client.post("/api/start")
    assert _steps(session, 40.0)["state"] == "TRACK"
    assert client.get("/api/state").json()["aligned"] is True

    client.post("/api/hold", json={"hold": True})
    assert _steps(session, 0.5)["hold"] is True
    client.post("/api/hold", json={"hold": False})
    assert _steps(session, 0.5)["hold"] is False

    client.post("/api/stop")
    assert _steps(session, 0.2)["state"] == "IDLE"
    client.post("/api/realign")
    assert _steps(session, 0.2)["state"] == "ACQUIRE"

    before = session.latest()["setup_id"]
    client.post("/api/new_setup")
    message = _steps(session, 0.2)
    assert message["setup_id"] == before + 1 and message["state"] == "IDLE"
    assert client.post("/api/hold", json={}).status_code == 422


def test_report_has_before_after_and_events(rig):
    session, client = rig
    assert client.get("/api/report").json()["passed"] is False
    client.post("/api/start")
    _steps(session, 40.0)
    report = client.get("/api/report").json()
    assert report["passed"] is True and report["sim"] is True
    assert set(report["before"]) == set(ERROR_AXES)
    for axis in ERROR_AXES:
        assert report["after"][axis]["score_deg"] < report["tolerance_deg"]
    texts = [text for _, text in report["events"]]
    assert "ALIGNED" in texts and any(t.startswith("move ") for t in texts)

    # A pass that is no longer the current state must not be reported as passed.
    client.post("/api/stop")
    _steps(session, 0.2)
    stale = client.get("/api/report").json()
    assert stale["after"] is not None and stale["passed"] is False


def test_background_loop_runs_in_real_time():
    session = Session(QUIET, seed=1)
    with TestClient(create_app(session)) as client:
        client.post("/api/start")
        with client.websocket_connect("/ws") as ws:
            first = ws.receive_json()
            later = [ws.receive_json() for _ in range(5)][-1]
        assert later["seq"] > first["seq"]
    assert session._thread is None  # stopped on shutdown

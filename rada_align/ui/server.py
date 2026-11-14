"""FastAPI app for the operator dashboard."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel

from .session import Session

STATIC = Path(__file__).parent / "static"


class HoldRequest(BaseModel):
    hold: bool


def create_app(session: Session, autostart: bool = True) -> FastAPI:
    """`autostart=False` leaves the session to be stepped by the caller (tests)."""

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if autostart:
            session.run_in_background()
        yield
        session.shutdown()

    app = FastAPI(title="RADA Align", lifespan=lifespan)

    @app.get("/")
    def index():
        return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/state")
    def state():
        return session.latest() or {}

    @app.get("/api/report")
    def report():
        return session.report()

    @app.post("/api/start")
    def start():
        session.start()
        return {"ok": True}

    @app.post("/api/stop")
    def stop():
        session.stop()
        return {"ok": True}

    @app.post("/api/realign")
    def realign():
        session.realign()
        return {"ok": True}

    @app.post("/api/hold")
    def hold(request: HoldRequest):
        session.set_hold(request.hold)
        return {"ok": True, "hold": request.hold}

    @app.post("/api/new_setup")
    def new_setup():
        session.new_setup()
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        seq = max(session.seq - 1, 0)  # start with the latest message, not the backlog
        try:
            while True:
                for message in session.since(seq):
                    await websocket.send_json(message)
                    seq = message["seq"]
                await asyncio.sleep(session.dt / 2)
        except (WebSocketDisconnect, RuntimeError):
            pass

    return app

"""FastAPI app for the final-design demo (simulation)."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .session import PsdSession

STATIC = Path(__file__).parent / "static"


class Command(BaseModel):
    name: str
    value: float | str | bool | None = None


def create_app(session: PsdSession, autostart: bool = True) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if autostart:
            session.run_in_background()
        yield
        session.shutdown()

    app = FastAPI(title="RADA Align - final design demo", lifespan=lifespan)
    # three.js and the font, bundled so the page works offline (static/vendor/README.md)
    app.mount("/vendor", StaticFiles(directory=STATIC / "vendor"), name="vendor")

    @app.get("/")
    def index():
        return FileResponse(STATIC / "demo.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/state")
    def state():
        return session.latest() or {}

    @app.post("/api/command")
    def command(cmd: Command):
        try:
            return session.command(cmd.name, cmd.value)
        except KeyError:
            raise HTTPException(400, f"unknown command {cmd.name!r}")

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        seq = max(session.seq - 1, 0)
        try:
            while True:
                for message in session.since(seq):
                    await websocket.send_json(message)
                    seq = message["seq"]
                await asyncio.sleep(0.02)
        except WebSocketDisconnect:
            pass

    return app

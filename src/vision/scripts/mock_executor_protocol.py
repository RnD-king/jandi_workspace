"""Motor-free model of forward_walk's ROS command admission/status protocol.

This module intentionally has no ROS dependency, so its state machine can be
unit-tested without ROS 2 or the robot. Status values match CommandStatus.msg.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

ACK = 1
DONE = 2
READY = 3
LINE = 1
CAMERA_COMMANDS = frozenset({1, 2, 3})
# Matches forward_walk's READY-capable walks (legacy 11/12/13 and 21..26).
READY_ACTIONS = frozenset({11, 12, 13, 21, 22, 23, 24, 25, 26})


@dataclass
class Body:
    command_id: int
    mission: int
    action: int
    yaw: int
    acked: bool = False
    ready: bool = False


@dataclass
class Camera:
    command_id: int
    request: int
    acked: bool = False


@dataclass(frozen=True)
class StatusEvent:
    channel: str  # 'body' or 'camera'
    command_id: int
    status: int


class MockExecutorProtocol:
    """Single body action, one queued LINE action, independent camera action."""

    def __init__(self, history_size: int = 64):
        self.active: Body | None = None
        self.queued: Body | None = None
        self.camera: Camera | None = None
        self.completed_body = deque(maxlen=history_size)
        self.completed_camera = deque(maxlen=history_size)
        self._body_ids: set[int] = set()
        self._camera_ids: set[int] = set()
        self._events: list[StatusEvent] = []

    def take_events(self) -> list[StatusEvent]:
        events, self._events = self._events, []
        return events

    def _send(self, channel: str, cid: int, status: int) -> None:
        self._events.append(StatusEvent(channel, cid, status))

    @staticmethod
    def _remember(history: deque, ids: set[int], cid: int) -> None:
        if len(history) == history.maxlen:
            ids.discard(history[0])
        history.append(cid)
        ids.add(cid)

    def receive_body(self, cid: int, mission: int, action: int, yaw: int,
                     auto_ack: bool = True) -> str:
        if cid <= 0:
            return 'BODY rejected: zero action_id'
        if cid in self._body_ids:
            self._send('body', cid, DONE)
            return f'BODY completed duplicate id={cid}: replay DONE'
        for item in (self.active, self.queued):
            if item is not None and cid == item.command_id:
                if (mission, action, yaw) != (item.mission, item.action, item.yaw):
                    return f'BODY rejected: id={cid} reused with different command'
                if item.acked:
                    self._send('body', cid, ACK)
                    return f'BODY active/queued duplicate id={cid}: replay ACK'
                return f'BODY duplicate id={cid}: awaiting manual ACK'
        if mission not in (1, 2, 3, 4) or action <= 0:
            return f'BODY rejected id={cid}: invalid mission/action'
        item = Body(cid, mission, action, yaw)
        if self.active is None:
            self.active = item
            label = 'active'
        elif (self.active.ready and self.queued is None and
              self.active.mission == LINE and mission == LINE):
            self.queued = item
            label = 'queued'
        else:
            return f'BODY rejected id={cid}: busy, no READY or not LINE→LINE'
        if auto_ack:
            item.acked = True
            self._send('body', cid, ACK)
        return f'BODY {label} id={cid} mission={mission} action={action} yaw={yaw}'

    def ack_body(self, queued: bool = False) -> str:
        item = self.queued if queued else self.active
        if item is None:
            return 'BODY ACK ignored: no ' + ('queued' if queued else 'active') + ' command'
        item.acked = True
        self._send('body', item.command_id, ACK)
        return f'BODY {"queued" if queued else "active"} ACK id={item.command_id}'

    def ready_body(self) -> str:
        item = self.active
        if item is None:
            return 'BODY READY ignored: no active command'
        if not item.acked:
            return 'BODY READY blocked: ACK first'
        if item.action not in READY_ACTIONS:
            return f'BODY READY blocked: action={item.action} is not READY-capable'
        if item.ready:
            return 'BODY READY already sent'
        item.ready = True
        self._send('body', item.command_id, READY)
        return f'BODY READY id={item.command_id}'

    def done_body(self) -> str:
        item = self.active
        if item is None:
            return 'BODY DONE ignored: no active command'
        if not item.acked:
            return 'BODY DONE blocked: ACK first'
        if self.queued is not None and not self.queued.acked:
            return 'BODY DONE blocked: send queued ACK first'
        self._send('body', item.command_id, DONE)
        self._remember(self.completed_body, self._body_ids, item.command_id)
        self.active, self.queued = self.queued, None
        return f'BODY DONE id={item.command_id}; queued action promoted' if self.active else f'BODY DONE id={item.command_id}'

    def receive_camera(self, cid: int, request: int, auto_ack: bool = True,
                       auto_done: bool = False) -> str:
        if cid <= 0 or request not in CAMERA_COMMANDS:
            return f'CAMERA rejected id={cid}: invalid id/request'
        if cid in self._camera_ids:
            self._send('camera', cid, DONE)
            return f'CAMERA completed duplicate id={cid}: replay DONE'
        if self.camera and self.camera.command_id == cid:
            if self.camera.request != request:
                return f'CAMERA rejected: id={cid} reused with different request'
            if self.camera.acked:
                self._send('camera', cid, ACK)
                return f'CAMERA active duplicate id={cid}: replay ACK'
            return f'CAMERA duplicate id={cid}: awaiting manual ACK'
        if self.camera is not None:
            return f'CAMERA rejected id={cid}: busy'
        self.camera = Camera(cid, request)
        if auto_ack or auto_done:
            self.ack_camera()
        if auto_done:
            self.done_camera()
        return f'CAMERA accepted id={cid} request={request}'

    def ack_camera(self) -> str:
        if self.camera is None:
            return 'CAMERA ACK ignored: no pending request'
        self.camera.acked = True
        self._send('camera', self.camera.command_id, ACK)
        return f'CAMERA ACK id={self.camera.command_id}'

    def done_camera(self) -> str:
        if self.camera is None:
            return 'CAMERA DONE ignored: no pending request'
        if not self.camera.acked:
            return 'CAMERA DONE blocked: ACK first'
        item = self.camera
        self._send('camera', item.command_id, DONE)
        self._remember(self.completed_camera, self._camera_ids, item.command_id)
        self.camera = None
        return f'CAMERA DONE id={item.command_id}'

    def reset(self) -> str:
        """Mock-only state clear. Deliberately emits NO fake DONE/ACK."""
        self.active = self.queued = self.camera = None
        self.completed_body.clear()
        self.completed_camera.clear()
        self._body_ids.clear()
        self._camera_ids.clear()
        self._events.clear()
        return 'MOCK RESET: internal state cleared; no ROS status emitted'

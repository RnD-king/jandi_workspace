#!/usr/bin/env python3
"""Manual ROS 2 executor UI for motor-free vision tuning.

Publishes only vision/msg/CommandStatus. NEVER publishes body/camera commands,
opens Dynamixel devices, or drives actuators. Do not run a physical executor
on the same command topics while using this mock.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import tkinter as tk
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from vision.msg import ActionCommand, CameraCommand, CommandStatus

from mock_executor_protocol import MockExecutorProtocol, ACK, DONE, READY

ACTIONS = {
    1: 'DEFAULT_POSITION', 2: 'DEFAULT_POSE_MODE', 3: 'STEP_FORWARD_HALF',
    4: 'STEP_BACK', 5: 'LEFT_SIDE_STEP', 6: 'RIGHT_SIDE_STEP',
    7: 'TURN_LEFT', 8: 'TURN_RIGHT', 9: 'WALK_MODE',
    10: 'STEP_FORWARD_ONE', 11: 'STEP_FORWARD_LEFT_THREE',
    12: 'STEP_FORWARD_RIGHT_THREE', 13: 'STEP_FORWARD_FIVE',
    14: 'PICK_BALL', 15: 'RECATCH', 16: 'HUDDLE', 17: 'SHOOT',
    18: 'TURN_LEFT_AND_STEP', 19: 'TURN_RIGHT_AND_STEP',
    20: 'CONTACT_WALK', 21: 'STEP_FORWARD_TWO', 22: 'STEP_FORWARD_THREE',
    23: 'STEP_FORWARD_FOUR', 24: 'STEP_FORWARD_LEFT_TWO',
    25: 'STEP_FORWARD_RIGHT_TWO', 26: 'STEP_FORWARD_RIGHT_ONE',
    27: 'STEP_FORWARD_LEFT_ONE', 28: 'STEP_FORWARD_RIGHT_ONE_HUDDLE',
    29: 'PICK_BALL_RE', 30: 'PICK_BALL_RE_STAND', 31: 'TURN_LEFT_IN_PLACE',
    32: 'LINE_LEFT_UNIT', 33: 'LINE_STRAIGHT_UNIT', 34: 'LINE_RIGHT_UNIT',
}
MISSIONS = {1: 'LINE', 2: 'BALL', 3: 'GOAL', 4: 'HURDLE'}
CAMERAS = {1: 'DOWN', 2: 'FORWARD', 3: 'GOAL'}
STATUS_NAMES = {ACK: 'ACK', READY: 'READY', DONE: 'DONE'}


def service_mapping(name: str) -> dict[str, str]:
    name = name.strip('/')
    if name == 'line_p2p_tuning_node':
        return {'START': f'/{name}/start_line_tuning_trial'}
    if name == 'perception_tuning_node':
        return {key.upper(): '/' + key for key in
                ('start', 'preview', 'commit', 'stop_repeat', 'reset')}
    if name in ('ball_tuning_node', 'hurdle_tuning_node', 'goal_tuning_node'):
        return {key.upper(): f'/{name}/{key}' for key in
                ('start', 'preview', 'commit', 'stop_repeat', 'reset')}
    return {}


class MockExecutorUI:
    def __init__(self, args):
        self.args = args
        self.protocol = MockExecutorProtocol()
        self.closed = False
        self.futures = []
        rclpy.init()
        self.node = rclpy.create_node('jandi_mock_executor_ui')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.action_pub = self.node.create_publisher(
            CommandStatus, args.action_status, qos)
        self.camera_pub = self.node.create_publisher(
            CommandStatus, args.camera_status, qos)
        self.action_sub = self.node.create_subscription(
            ActionCommand, args.action_cmd, self.on_action, qos)
        self.camera_sub = self.node.create_subscription(
            CameraCommand, args.camera_cmd, self.on_camera, qos)
        self.root = tk.Tk()
        self.root.title('Jandi Mock Executor | NO MOTOR')
        self.root.geometry('930x620')
        self.auto_body_ack = tk.BooleanVar(value=True)
        self.auto_camera_ack = tk.BooleanVar(value=True)
        self.auto_camera_done = tk.BooleanVar(value=False)
        self.body_text = tk.StringVar()
        self.queued_text = tk.StringVar()
        self.camera_text = tk.StringVar()
        self._make_widgets()
        self._update_labels()
        self.log('MOCK MODE: No motor commands. Stop real forward_walk before testing.')
        self.log(f'BODY {args.action_cmd} -> {args.action_status}')
        self.log(f'CAMERA {args.camera_cmd} -> {args.camera_status}')
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(30, self.spin)

    def _make_widgets(self):
        frame = ttk.Frame(self.root, padding=12)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='NO MOTOR / SIMULATED FEEDBACK — stop real forward_walk!',
                  foreground='#ae271b', font=('Sans', 11, 'bold')).pack(anchor='w')
        for var in (self.body_text, self.queued_text, self.camera_text):
            ttk.Label(frame, textvariable=var).pack(anchor='w', pady=3)
        options = ttk.Frame(frame)
        options.pack(fill='x', pady=8)
        for label, variable in (
            ('Auto body ACK', self.auto_body_ack),
            ('Auto camera ACK', self.auto_camera_ack),
            ('Auto camera DONE', self.auto_camera_done),
        ):
            ttk.Checkbutton(options, text=label, variable=variable).pack(
                side='left', padx=(0, 15))
        self._button_row(frame, 'BODY', [
            ('ACK', lambda: self._apply(self.protocol.ack_body)),
            ('QUEUED ACK', lambda: self._apply(self.protocol.ack_body, queued=True)),
            ('READY', lambda: self._apply(self.protocol.ready_body)),
            ('DONE', lambda: self._apply(self.protocol.done_body)),
        ])
        self._button_row(frame, 'CAMERA', [
            ('ACK', lambda: self._apply(self.protocol.ack_camera)),
            ('DONE', lambda: self._apply(self.protocol.done_camera)),
        ])
        self._button_row(frame, 'MOCK', [
            ('RESET MOCK STATE', self.reset_mock),
        ])
        services = service_mapping(self.args.tuning_node)
        if services:
            self._button_row(frame, 'TUNING', [
                (label, lambda path=path: self.call_service(path))
                for label, path in services.items()
            ])
        ttk.Label(frame, text='Incoming commands / outgoing statuses / service results').pack(
            anchor='w', pady=(10, 4))
        self.logbox = ScrolledText(frame, height=19, wrap='word', state='disabled')
        self.logbox.pack(fill='both', expand=True)

    @staticmethod
    def _button_row(parent, label, buttons):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=3)
        ttk.Label(row, text=label, width=10).pack(side='left')
        for text, callback in buttons:
            ttk.Button(row, text=text, command=callback).pack(side='left', padx=3)

    def log(self, message):
        self.logbox.configure(state='normal')
        self.logbox.insert('end', f'[{datetime.now():%H:%M:%S}] {message}\n')
        self.logbox.see('end')
        self.logbox.configure(state='disabled')

    def _update_labels(self):
        body = self.protocol.active
        queued = self.protocol.queued
        camera = self.protocol.camera
        self.body_text.set(
            f'BODY ACTIVE id={body.command_id} {MISSIONS.get(body.mission, body.mission)}/'
            f'{ACTIONS.get(body.action, body.action)} yaw={body.yaw} '
            f'ACK={body.acked} READY={body.ready}' if body else 'BODY ACTIVE: IDLE')
        self.queued_text.set(
            f'BODY QUEUED id={queued.command_id} {MISSIONS.get(queued.mission, queued.mission)}/'
            f'{ACTIONS.get(queued.action, queued.action)} ACK={queued.acked}'
            if queued else 'BODY QUEUED: NONE')
        self.camera_text.set(
            f'CAMERA id={camera.command_id} {CAMERAS.get(camera.request, camera.request)} '
            f'ACK={camera.acked}' if camera else 'CAMERA: IDLE')

    def _flush_events(self):
        for event in self.protocol.take_events():
            pub = self.action_pub if event.channel == 'body' else self.camera_pub
            msg = CommandStatus()
            msg.command_id = event.command_id
            msg.status = event.status
            pub.publish(msg)
            self.log(f'{event.channel.upper()} -> {STATUS_NAMES[event.status]} id={event.command_id}')
        self._update_labels()

    def _apply(self, fn, *args, **kwargs):
        notice = fn(*args, **kwargs)
        if notice:
            self.log(notice)
        self._flush_events()

    def on_action(self, msg):
        self._apply(self.protocol.receive_body, int(msg.action_id), int(msg.mission),
                    int(msg.action), int(msg.target_yaw_deg), self.auto_body_ack.get())

    def on_camera(self, msg):
        self._apply(self.protocol.receive_camera, int(msg.command_id),
                    int(msg.request), self.auto_camera_ack.get(),
                    self.auto_camera_done.get())

    def reset_mock(self):
        if not messagebox.askyesno(
            'Reset mock only',
            'Clear mock state WITHOUT sending ACK/DONE?\n'
            'Vision may remain waiting for an outstanding action.\n'
            'Reset the vision FSM separately if necessary.',
            parent=self.root,
        ):
            return
        self._apply(self.protocol.reset)

    def call_service(self, name):
        client = self.node.create_client(Trigger, name)
        if not client.service_is_ready():
            self.log(f'SERVICE {name}: unavailable')
            self.node.destroy_client(client)
            return
        future = client.call_async(Trigger.Request())
        self.futures.append((client, future, name))
        self.log(f'SERVICE -> {name}')

    def spin(self):
        if self.closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            for item in list(self.futures):
                client, future, name = item
                if not future.done():
                    continue
                try:
                    response = future.result()
                    self.log(f'SERVICE {name}: success={response.success} | {response.message}')
                except Exception as error:
                    self.log(f'SERVICE {name}: ERROR {error}')
                self.node.destroy_client(client)
                self.futures.remove(item)
        except Exception as error:
            self.log(f'ROS ERROR: {error}')
        self.root.after(30, self.spin)

    def close(self):
        self.closed = True
        for client, _, _ in self.futures:
            self.node.destroy_client(client)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tuning-node', default='perception_tuning_node',
                        choices=['perception_tuning_node', 'line_p2p_tuning_node',
                                 'ball_tuning_node', 'hurdle_tuning_node',
                                 'goal_tuning_node'])
    parser.add_argument('--action-cmd', default='/jandi_vision/action_cmd')
    parser.add_argument('--action-status', default='/jandi_vision/action_status')
    parser.add_argument('--camera-cmd', default='/jandi_vision/camera_cmd')
    parser.add_argument('--camera-status', default='/jandi_vision/camera_status')
    args = parser.parse_args()
    MockExecutorUI(args).run()


if __name__ == '__main__':
    main()

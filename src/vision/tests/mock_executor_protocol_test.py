#!/usr/bin/env python3
"""Protocol regression tests without ROS or motor hardware."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from mock_executor_protocol import ACK, DONE, READY, MockExecutorProtocol, StatusEvent


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.p = MockExecutorProtocol(history_size=2)

    def events(self):
        return self.p.take_events()

    def test_ack_ready_done_duplicate_replay(self):
        self.p.receive_body(10, 1, 13, 0)
        self.assertEqual(self.events(), [StatusEvent('body', 10, ACK)])
        self.p.receive_body(10, 1, 13, 0)
        self.assertEqual(self.events(), [StatusEvent('body', 10, ACK)])
        self.p.ready_body()
        self.assertEqual(self.events(), [StatusEvent('body', 10, READY)])
        self.p.done_body()
        self.assertEqual(self.events(), [StatusEvent('body', 10, DONE)])
        self.p.receive_body(10, 1, 13, 0)
        self.assertEqual(self.events(), [StatusEvent('body', 10, DONE)])

    def test_one_slot_queue_only_line_after_ready(self):
        self.p.receive_body(1, 1, 13, 0)
        self.events()
        self.p.receive_body(2, 1, 13, 0)
        self.assertIsNone(self.p.queued)
        self.assertEqual(self.events(), [])
        self.p.ready_body()
        self.events()
        self.p.receive_body(2, 2, 13, 0)
        self.assertIsNone(self.p.queued)
        self.p.receive_body(2, 1, 13, 0)
        self.assertEqual(self.p.queued.command_id, 2)
        self.assertEqual(self.events(), [StatusEvent('body', 2, ACK)])
        self.p.receive_body(3, 1, 13, 0)
        self.assertEqual(self.events(), [])
        self.p.done_body()
        self.assertEqual(self.p.active.command_id, 2)
        self.assertEqual(self.events(), [StatusEvent('body', 1, DONE)])

    def test_manual_acks(self):
        self.p.receive_body(1, 1, 13, 0, auto_ack=False)
        self.assertEqual(self.p.ready_body(), 'BODY READY blocked: ACK first')
        self.assertEqual(self.events(), [])
        self.p.ack_body()
        self.p.ready_body()
        self.events()
        self.p.receive_body(2, 1, 13, 0, auto_ack=False)
        self.assertIsNotNone(self.p.queued)
        self.assertEqual(self.p.done_body(), 'BODY DONE blocked: send queued ACK first')
        self.p.ack_body(queued=True)
        self.p.done_body()
        self.assertEqual([x.status for x in self.events()], [ACK, DONE])

    def test_ready_rejected_for_non_walk(self):
        self.p.receive_body(8, 3, 17, 0)
        self.events()
        self.assertIn('not READY-capable', self.p.ready_body())
        self.assertEqual(self.events(), [])
        self.p.done_body()
        self.assertEqual(self.events(), [StatusEvent('body', 8, DONE)])

    def test_camera_dedup_independent(self):
        self.p.receive_body(1, 1, 13, 0)
        self.p.receive_camera(1, 2)
        self.assertEqual(self.events(), [StatusEvent('body', 1, ACK), StatusEvent('camera', 1, ACK)])
        self.p.receive_camera(1, 2)
        self.assertEqual(self.events(), [StatusEvent('camera', 1, ACK)])
        self.p.done_camera()
        self.assertEqual(self.events(), [StatusEvent('camera', 1, DONE)])
        self.p.receive_camera(1, 2)
        self.assertEqual(self.events(), [StatusEvent('camera', 1, DONE)])

    def test_reset_never_fakes_done(self):
        self.p.receive_body(1, 1, 13, 0)
        self.events()
        self.p.reset()
        self.assertEqual(self.events(), [])
        self.assertIsNone(self.p.active)

    def test_completed_history_bounded(self):
        for cid in (1, 2, 3):
            self.p.receive_body(cid, 3, 17, 0)
            self.events()
            self.p.done_body()
            self.events()
        self.assertEqual(set(self.p.completed_body), {2, 3})
        self.p.receive_body(1, 3, 17, 0)
        self.assertEqual(self.events(), [StatusEvent('body', 1, ACK)])


if __name__ == '__main__':
    unittest.main()

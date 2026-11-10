#!/usr/bin/python3
"""Selection, explicit Enter, log completeness, paging; no robot processes."""
from pathlib import Path
import json
import os
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import jandi
from jandi_logs import FILTERS, LogClassifier, LogJournal, LogView
from jandi_runtime import Plan


class OptionsLogsTest(unittest.TestCase):
    def test_real_terminal_option_toggles_enter_and_escape(self):
        for keys, expected in ((b"123\r",b"RESULT=[true, true, false]"),
                               (b"\x1b",b"RESULT=null")):
            master,slave=pty.openpty()
            driver=f"import sys,json; sys.path.insert(0,{str(ROOT/'scripts')!r}); import jandi; print('RESULT='+json.dumps(jandi.choose_options()))"
            proc=subprocess.Popen([sys.executable,"-c",driver],stdin=slave,stdout=slave,stderr=slave)
            os.close(slave)
            output=b""
            try:
                deadline=time.monotonic()+3
                while b"HTTP" not in output:
                    self.assertLess(time.monotonic(),deadline,"option screen did not appear")
                    if select.select([master],[],[],.1)[0]: output+=os.read(master,65536)
                self.assertIsNone(proc.poll())  # Enter has not been pressed.
                os.write(master,keys)
                while expected not in output and time.monotonic()<deadline:
                    if select.select([master],[],[],.1)[0]:
                        try: output+=os.read(master,65536)
                        except OSError: break
                proc.wait(timeout=3)
                self.assertEqual(proc.returncode,0,output)
                self.assertIn(expected,output)
            finally:
                if proc.poll() is None:
                    proc.kill();proc.wait()
                os.close(master)

    def test_services_require_enter_once_and_cancel_has_no_request(self):
        for number, name in zip("4567", ("start", "pause", "resume", "reset")):
            page, request = jandi.key_request("main", number)
            self.assertIsNone(request)
            for key in ("", "x", number):
                self.assertIsNone(jandi.key_request(page, key)[1])
            self.assertEqual(jandi.key_request(page, "\r"), ("main", {"op": "service", "name": name}))
            self.assertIsNone(jandi.key_request("main", "\r")[1])
            for key in ("0", "\x1b"):
                self.assertEqual(jandi.key_request(page, key), ("main", None))

    def test_log_submenus_never_request_control(self):
        self.assertEqual(jandi.key_request("main", "8"), ("logs", None))
        for number in FILTERS:
            page, request = jandi.key_request("logs", number)
            self.assertEqual(page, "logs:" + number)
            self.assertIsNone(request)
            for key in "123456789pnf":
                self.assertIsNone(jandi.key_request(page, key)[1])
            self.assertEqual(jandi.key_request(page, "0"), ("logs", None))

    def test_selected_options_override_yaml_without_changing_camera_or_motor(self):
        for show in (False, True):
            for terminal in (False, True):
                for stream in (False, True):
                    plan=Plan.load(ROOT, show_debug_view=show,
                                   terminal_node=terminal, stream_debug_view=stream)
                    command=plan.commands()["vision"]
                    for key, value in (("show_debug_view",show),("terminal_node",terminal),("stream_debug_view",stream)):
                        self.assertIn(f"{key}:={str(value).lower()}",command)
                    self.assertNotIn("inference_hz", " ".join(command))
                    self.assertEqual(plan.commands()["camera"], ["ros2","launch","realsense2_camera","rs_launch.py"])

    def test_reconnect_without_mode_skips_selection_and_start(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(jandi,"rpc",return_value={"plan":{}}), \
             patch.object(jandi,"console") as console, \
             patch.object(jandi,"choose_options") as choose, \
             patch.object(jandi,"background") as background:
            jandi.main(["--state-dir",directory])
            console.assert_called_once()
            choose.assert_not_called()
            background.assert_not_called()

    def test_cancel_selection_starts_nothing(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(jandi,"rpc",side_effect=FileNotFoundError), \
             patch.object(jandi,"choose_options",return_value=None), \
             patch.object(jandi,"background") as background:
            jandi.main(["--state-dir",directory])
            background.assert_not_called()

    def test_full_journal_traceback_paging_and_partial_last_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"all.jsonl"
            journal=LogJournal(path)
            classifier=LogClassifier()
            messages=["[INFO] camera ready", "Traceback (most recent call last):",
                      '  File "driver.py", line 1', "ValueError: invalid option",
                      "[WARN] sensor warming", "[FATAL] motor lost", "[DEBUG] details"]
            for text in messages:
                journal.write("camera",classifier.classify(text),text)
            for index in range(100):
                journal.write("vision","INFO",str(index))
            huge="x"*5000
            journal.write("manager","ERROR",huge)
            full=LogView(directory,"1")
            records,_=full.lines()
            self.assertEqual(len(full.matches),108)
            self.assertEqual(records[-1]["text"],huge)
            alerts,_=LogView(directory,"2").lines(100)
            self.assertEqual(FILTERS["2"][1], {"WARN", "ERROR", "FATAL"})
            self.assertEqual(FILTERS["2"][2], "전체 경고·오류")
            self.assertEqual([r["text"] for r in alerts],messages[1:6]+[huge])
            warnings,_=LogView(directory,"3").lines(100)
            self.assertIn(messages[4],[r["text"] for r in warnings])
            full.move("p")
            earlier,_=full.lines()
            self.assertNotEqual(earlier[-1]["text"],huge)
            full.move("f")
            self.assertEqual(full.lines()[0][-1]["text"],huge)
            with path.open("ab") as stream: stream.write(b'{"time":')
            full.poll()
            self.assertEqual(len(full.matches),108)


if __name__ == "__main__":
    unittest.main()

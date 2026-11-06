#!/usr/bin/python3
"""Inspect real launch actions/install rules without launching any hardware."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("jandi_launch", ROOT / "launch/jandi.launch.py")
launch_file = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)


class LaunchTest(unittest.TestCase):
    def test_launch_bootstraps_same_detached_runtime_and_no_legacy_nodes(self):
        with patch.object(launch_file, "get_package_prefix", return_value="/target/install/vision"):
            description = launch_file.generate_launch_description()
        arguments = [x for x in description.entities if isinstance(x, DeclareLaunchArgument)]
        self.assertEqual([x.name for x in arguments], ["mode", "workspace", "core_prefix"])
        self.assertEqual(arguments[0].choices, ["practice", "IRC"])
        processes = [x for x in description.entities if isinstance(x, ExecuteProcess)]
        self.assertEqual(len(processes), 1)
        for mode in ("practice", "IRC"):
            context = LaunchContext()
            context.launch_configurations.update(mode=mode, workspace="/home/rnd/jandi", core_prefix="/home/rnd/vision_core/install")
            command = [perform_substitutions(context, part) for part in processes[0].cmd]
            self.assertEqual(command[:3], ["/target/install/vision/lib/vision/jandi", "--background", mode])
            self.assertNotIn("robot_bringup", " ".join(command))

    def test_install_rules_and_declared_runtime_dependencies(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        for filename in ("scripts/jandi.py", "scripts/jandi_runtime.py", "scripts/jandi_ros.py",
                         "scripts/install_jandi_command.sh", "JANDI_RUNTIME.md", "DIRECTORY launch/"):
            self.assertIn(filename, cmake)
        deps = {n.text for n in ET.parse(ROOT / "package.xml").findall("exec_depend")}
        self.assertTrue({"rclpy", "launch", "launch_ros", "ros2launch", "ros2run", "ament_index_python",
                         "python3-yaml", "realsense2_camera"} <= deps)

    def test_entrypoint_installer_quotes_paths_and_never_launches_at_install_time(self):
        with tempfile.TemporaryDirectory(prefix="jandi install ") as temp:
            root = Path(temp)
            workspace = root / "workspace with spaces"
            (workspace / "install").mkdir(parents=True)
            (workspace / "install/setup.bash").write_text("export JANDI_INSTALL_TEST=1\n")
            bin_dir = root / "bin"
            core = root / "core with spaces"
            args = ["bash", str(ROOT / "scripts/install_jandi_command.sh"), str(workspace), str(core), str(bin_dir)]
            subprocess.run(args, check=True, capture_output=True)
            wrapper = bin_dir / "jandi"
            subprocess.run(["bash", "-n", str(wrapper)], check=True)
            self.assertFalse((root / "invoked").exists())
            ros2 = bin_dir / "ros2"
            ros2.write_text('#!/bin/bash\nprintf "%s\\n" "$ROS_DOMAIN_ID" "$VISION_CORE_ROOT" "$JANDI_INSTALL_TEST" "$@" > "$JANDI_TEST_OUTPUT"\n')
            ros2.chmod(0o755)
            env = dict(os.environ, PATH=str(bin_dir) + ":" + os.environ["PATH"],
                       JANDI_TEST_OUTPUT=str(root / "invoked"))
            subprocess.run([str(wrapper), "IRC"], env=env, check=True, capture_output=True)
            values = (root / "invoked").read_text().splitlines()
            self.assertEqual(values[:3], ["22", str(core), "1"])
            self.assertEqual(values[3:7], ["run", "vision", "jandi", "IRC"])
            wrapper.write_text("#!/bin/bash\necho unrelated\n")
            self.assertNotEqual(subprocess.run(args, capture_output=True).returncode, 0)


if __name__ == "__main__":
    unittest.main()

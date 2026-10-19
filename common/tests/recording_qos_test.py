"""Offline checks for the flat native rosbag2 preset (no ROS dependency)."""
from pathlib import Path
import re
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'ros2_ws/src/prism_ros_driver/config/recording_qos.yaml'


class RecordingQosTest(unittest.TestCase):
    def test_all_topics_and_bounded_policies(self):
        expected = {
            '/prism/imu0/data': 2048,
            '/prism/imu1/data': 2048,
            '/prism/lidar/imu': 2048,
            '/prism/lidar/points': 64,
            '/prism/camera/metadata': 256,
            '/diagnostics': 256,
        }
        expected.update({f'/prism/camera{i}/image/compressed': 64 for i in range(4)})
        expected.update({f'/prism/{name}': 256 for name in (
            'gnss/timing', 'gnss/reception', 'gnss/observations', 'gnss/raw',
            'rtk/cors_rtcm', 'gnss/receiver', 'gnss/fix', 'rtk/receiver',
            'rtk/fix', 'rtk/status', 'gnss/rover_rtcm',
        )})
        parsed = {}
        current = None
        for line in CONFIG.read_text().splitlines():
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            if line.startswith('/'):
                self.assertTrue(line.endswith(':'))
                current = line[:-1]
                self.assertNotIn(current, parsed)
                parsed[current] = {}
            else:
                self.assertIsNotNone(current)
                self.assertTrue(line.startswith('  '))
                key, value = line.strip().split(':', 1)
                self.assertNotIn(key, parsed[current])
                parsed[current][key] = value.strip()
        self.assertEqual(set(parsed), set(expected))
        self.assertEqual(len(parsed), 21)
        for topic, depth in expected.items():
            self.assertEqual(parsed[topic], {
                'history': 'keep_last', 'depth': str(depth),
                'reliability': 'reliable', 'durability': 'volatile',
            }, topic)

    def test_installed_with_config_directory(self):
        cmake = (ROOT / 'ros2_ws/src/prism_ros_driver/CMakeLists.txt').read_text()
        self.assertRegex(cmake, r'install\(DIRECTORY launch config\s+DESTINATION share/\$\{PROJECT_NAME\}\)')

    def test_readme_native_commands(self):
        readme = (ROOT / 'README.md').read_text()
        self.assertIn('(ros2_ws/src/prism_ros_driver/config/recording_qos.yaml)', readme)
        commands = [block for block in re.findall(r'```bash\n(.*?)```', readme, re.S)
                    if block.startswith('ros2 bag record')]
        self.assertEqual(len(commands), 3)
        for command in commands:
            self.assertIn('--qos-profile-overrides-path', command)
            self.assertIn('$(ros2 pkg prefix --share prism_ros_driver)/config/recording_qos.yaml', command)
            self.assertIn('--max-cache-size 134217728', command)
            subprocess.run(['bash', '-n'], input=command, text=True, check=True)
        self.assertTrue(any('--start-paused' in command for command in commands))
        self.assertTrue(any('/prism/lidar/points /prism/imu0/data' in command for command in commands))


if __name__ == '__main__':
    unittest.main()

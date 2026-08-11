import importlib.util
import math
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = (
    ROOT
    / "src/apps/known_map_navigation/auto_rover_known_map_bringup"
    / "scripts/fake_fast_livo_source.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("fake_fast_livo_source", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeFastLivoSourceMathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def test_integrates_forward_speed_in_rear_axle_heading(self):
        pose = self.module.Pose2(1.0, 2.0, math.pi / 2.0)
        updated = self.module.integrate_pose(pose, 0.5, 0.0, 2.0)
        self.assertAlmostEqual(1.0, updated.x, places=12)
        self.assertAlmostEqual(3.0, updated.y, places=12)
        self.assertAlmostEqual(math.pi / 2.0, updated.yaw, places=12)

    def test_midpoint_integration_and_yaw_normalization(self):
        pose = self.module.Pose2(0.0, 0.0, math.pi - 0.05)
        updated = self.module.integrate_pose(pose, 1.0, 0.2, 1.0)
        self.assertLess(updated.yaw, -math.pi + 0.2)
        self.assertAlmostEqual(-math.cos(0.05), updated.x, places=6)

    def test_non_identity_source_to_rear_transform_is_inverted(self):
        rear = self.module.Pose2(10.0, 20.0, math.pi / 2.0)
        source = self.module.source_pose_from_rear(rear, -0.20, 0.0, 0.0)
        self.assertAlmostEqual(10.0, source.x, places=12)
        self.assertAlmostEqual(20.20, source.y, places=12)
        self.assertAlmostEqual(math.pi / 2.0, source.yaw, places=12)

        reconstructed_rear = self.module.compose_pose(
            source, self.module.Pose2(-0.20, 0.0, 0.0)
        )
        self.assertAlmostEqual(rear.x, reconstructed_rear.x, places=12)
        self.assertAlmostEqual(rear.y, reconstructed_rear.y, places=12)
        self.assertAlmostEqual(rear.yaw, reconstructed_rear.yaw, places=12)


if __name__ == "__main__":
    unittest.main()

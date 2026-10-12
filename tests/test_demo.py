import math
import unittest

from src.run_demo import forward_kinematics, inverse_kinematics


class KinematicsTests(unittest.TestCase):
    def test_inverse_forward_round_trip(self):
        for target in ((1.0, 0.5), (0.7, 0.9), (1.2, 0.2)):
            angles = inverse_kinematics(*target)
            actual = forward_kinematics(*angles)
            self.assertAlmostEqual(actual[0], target[0], places=12)
            self.assertAlmostEqual(actual[1], target[1], places=12)

    def test_solution_is_finite(self):
        angles = inverse_kinematics(0.9, 0.6)
        self.assertTrue(all(math.isfinite(angle) for angle in angles))


if __name__ == "__main__":
    unittest.main()

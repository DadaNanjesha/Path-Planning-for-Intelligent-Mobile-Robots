import unittest
from D_Star.d_star import DStarAlgorithm
import networkx as nx


class TestDStarAlgorithm(unittest.TestCase):
    def test_init(self):
        x_obs = [1, 2, 3]
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        dstar = DStarAlgorithm(x_obs, y_obs, resol, radius)

        self.assertEqual(dstar.resol, resol)
        self.assertEqual(dstar.radius, radius)
        self.assertIsInstance(dstar.graph, nx.Graph)
        expected_obstacles = {(1, 4), (1, 5), (2, 4), (2, 5), (2, 6), (3, 5), (3, 6)}
        self.assertEqual(dstar.obstacles, expected_obstacles)

    def test_build_graph(self):
        x_obs = [1, 2, 3]
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        dstar = DStarAlgorithm(x_obs, y_obs, resol, radius)
        dstar.build_graph(x_obs, y_obs)

        self.assertGreater(dstar.graph.number_of_nodes(), 0)

    def test_motion_mod(self):
        x_obs = [1, 2, 3]
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        dstar = DStarAlgorithm(x_obs, y_obs, resol, radius)

        self.assertIsNotNone(dstar.motion)

    def test_empty_obstacles(self):
        x_obs = [1]
        y_obs = [0]
        resol = 0.5
        radius = 1.0

        dstar = DStarAlgorithm(x_obs, y_obs, resol, radius)

        expected_obstacles = {(1, 0)}
        self.assertEqual(dstar.obstacles, expected_obstacles)

    def test_single_obstacle(self):
        x_obs = [1]
        y_obs = [4]
        resol = 0.5
        radius = 1.0

        dstar = DStarAlgorithm(x_obs, y_obs, resol, radius)

        expected_obstacles = {(1, 4)}
        self.assertEqual(dstar.obstacles, expected_obstacles)

    def test_invalid_input(self):
        x_obs = "invalid"
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        with self.assertRaises(TypeError):
            DStarAlgorithm(x_obs, y_obs, resol, radius)


if __name__ == "__main__":
    unittest.main()

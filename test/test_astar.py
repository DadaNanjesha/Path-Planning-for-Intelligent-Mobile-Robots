import unittest
from A_Star.a_star_algorithm import AStarAlgorithm
import networkx as nx


class TestAStarAlgorithm(unittest.TestCase):
    def test_init(self):
        """
        Tests that the AStarAlgorithm object is properly initialized.
        It checks that the resolution, radius, graph, and obstacles are set correctly.
        """
        x_obs = [1, 2, 3]
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        astar = AStarAlgorithm(x_obs, y_obs, resol, radius)

        self.assertEqual(astar.resol, resol)
        self.assertEqual(astar.radius, radius)
        self.assertIsInstance(astar.graph, nx.Graph)
        expected_obstacles = {(1, 4), (1, 5), (2, 4), (2, 5), (2, 6), (3, 5), (3, 6)}
        self.assertEqual(astar.obstacles, expected_obstacles)

    def test_build_graph(self):
        """
        Tests that the build_graph method builds a graph with more than 0 nodes.
        """
        x_obs = [1, 2, 3]
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        astar = AStarAlgorithm(x_obs, y_obs, resol, radius)
        astar.build_graph(x_obs, y_obs)

        self.assertGreater(astar.graph.number_of_nodes(), 0)

    def test_motion_mod(self):
        """
        Tests that the motion_mod method returns a non-None value by
        verifying that the 'motion' attribute of the AStarAlgorithm
        instance is properly initialized.
        """

        x_obs = [1, 2, 3]
        y_obs = [4, 5, 6]
        resol = 0.5
        radius = 1.0

        astar = AStarAlgorithm(x_obs, y_obs, resol, radius)

        self.assertIsNotNone(astar.motion)


if __name__ == "__main__":
    unittest.main()

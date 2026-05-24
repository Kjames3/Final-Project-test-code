#!/usr/bin/env python3
import unittest
import sys
import os

# Bootstrap paths
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.utils.bayes_opt import solve_linear, BayesianOptimizer


class TestBayesianOptimization(unittest.TestCase):

    def test_linear_solver(self):
        """Verify that solve_linear correctly resolves standard linear equations."""
        # System:
        #  3*x + 2*y = 8
        #  1*x + 4*y = 6
        # Solution: x = 2.0, y = 1.0
        A = [
            [3.0, 2.0],
            [1.0, 4.0]
        ]
        b = [8.0, 6.0]
        
        x = solve_linear(A, b)
        self.assertAlmostEqual(x[0], 2.0, places=5)
        self.assertAlmostEqual(x[1], 1.0, places=5)

    def test_optimizer_convergence(self):
        """Verify the BayesianOptimizer converges on a 1-D Ks cost minimum (matches the live autotuner)."""
        import random
        random.seed(42)  # deterministic — the optimizer seeds exploration from random

        # Mock balancing target minimum: ks = 9.0
        target_ks = 9.0

        def evaluate_mock_cost(params):
            # Parabolic cost bowl over the single torque->PWM scalar
            return (params["ks"] - target_ks) ** 2

        # 1-D search bounds, matching AUTOTUNE_BOUNDS in the dashboard
        bounds = {"ks": (3.0, 20.0)}

        optimizer = BayesianOptimizer(bounds, length_scale=0.3, noise=1e-4, kappa=1.5)

        # Test a sequence of 12 iterations
        print("\nRunning Bayesian Optimization test iterations...")
        for iteration in range(12):
            suggestion = optimizer.suggest()
            cost = evaluate_mock_cost(suggestion)
            optimizer.register(suggestion, cost)
            print(f"  Iteration {iteration + 1:2d} | Suggested: Ks = {suggestion['ks']:5.2f} | Cost: {cost:6.2f}")

        # Find the best registered set
        best_idx = optimizer.y.index(min(optimizer.y))
        best_params = optimizer.X_raw[best_idx]
        best_cost = optimizer.y[best_idx]

        print(f"\n✓ Optimization Complete!")
        print(f"  Target Minimum : Ks = {target_ks:.2f}")
        print(f"  Found Minimum  : Ks = {best_params['ks']:.2f} (Cost: {best_cost:.4f})")

        # Assert convergence is close to the minimum
        self.assertTrue(abs(best_params["ks"] - target_ks) < 1.5, "Ks did not converge close to target.")


if __name__ == '__main__':
    unittest.main()

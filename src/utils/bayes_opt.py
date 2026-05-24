"""
Bayesian Optimization Core — Zero-dependency, pure-Python optimal search engine.
Uses Gaussian Process regression with an RBF kernel and a Gauss-Jordan linear solver
to suggest optimal hyperparameters (e.g. LQR gains) that minimize a performance cost.
"""

import math
import random
from typing import Dict, List, Tuple


def solve_linear(A: List[List[float]], b: List[float]) -> List[float]:
    """
    Solve the linear system A * x = b using Gaussian elimination with partial pivoting.
    Extremely robust and fast for n <= 20.
    """
    n = len(A)
    # Copy matrices to avoid mutating inputs
    M = [row[:] for row in A]
    y = list(b)

    for i in range(n):
        # Partial pivoting: find largest entry in column i
        pivot_row = i
        for r in range(i + 1, n):
            if abs(M[r][i]) > abs(M[pivot_row][i]):
                pivot_row = r

        # Swap rows
        M[i], M[pivot_row] = M[pivot_row], M[i]
        y[i], y[pivot_row] = y[pivot_row], y[i]

        pivot = M[i][i]
        if abs(pivot) < 1e-9:
            # Handle near-singular matrix with small regularization jitter
            pivot = 1e-9 if pivot >= 0 else -1e-9
            M[i][i] = pivot

        # Forward elimination
        for r in range(i + 1, n):
            factor = M[r][i] / pivot
            M[r][i] = 0.0
            for c in range(i + 1, n):
                M[r][c] -= factor * M[i][c]
            y[r] -= factor * y[i]

    # Back substitution
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        total = y[i]
        for c in range(i + 1, n):
            total -= M[i][c] * x[c]
        x[i] = total / M[i][i]

    return x


class BayesianOptimizer:
    """
    Lightweight, pure-Python Bayesian Optimizer.
    Finds parameters (bounds) that minimize an objective function (e.g., balance cost).
    """

    def __init__(
        self,
        bounds: Dict[str, Tuple[float, float]],
        length_scale: float = 0.3,
        noise: float = 1e-3,
        kappa: float = 1.96,
    ):
        """
        Args:
            bounds: Dict of name: (min, max) range tuples.
            length_scale: RBF kernel bandwidth (in normalized coordinates).
            noise: GP observation noise variance (regularization factor).
            kappa: UCB/LCB exploration trade-off factor (higher = more exploration).
        """
        self.bounds = bounds
        self.keys = sorted(bounds.keys())
        self.length_scale = length_scale
        self.noise = noise
        self.kappa = kappa

        self.X_raw: List[Dict[str, float]] = []  # Tested points (physical scale)
        self.X_norm: List[Tuple[float, ...]] = []  # Tested points (normalized 0-1 scale)
        self.y: List[float] = []  # Measured costs

    def _normalize(self, x_raw: Dict[str, float]) -> Tuple[float, ...]:
        """Normalize a coordinate dictionary to a (0.0 - 1.0) hypercube tuple."""
        normalized = []
        for k in self.keys:
            low, high = self.bounds[k]
            span = high - low if high > low else 1.0
            val = (x_raw[k] - low) / span
            normalized.append(max(0.0, min(1.0, val)))
        return tuple(normalized)

    def _denormalize(self, x_norm: Tuple[float, ...]) -> Dict[str, float]:
        """Convert a normalized coordinate tuple back to a physical scale dictionary."""
        denormalized = {}
        for idx, k in enumerate(self.keys):
            low, high = self.bounds[k]
            val = low + x_norm[idx] * (high - low)
            denormalized[k] = val
        return denormalized

    def _rbf_kernel(self, x1: Tuple[float, ...], x2: Tuple[float, ...]) -> float:
        """Compute the Radial Basis Function covariance between two normalized tuples."""
        dist_sq = sum((a - b) ** 2 for a, b in zip(x1, x2))
        return math.exp(-dist_sq / (2.0 * (self.length_scale ** 2)))

    def register(self, x: Dict[str, float], cost: float) -> None:
        """Register a new evaluated coordinate and its resulting cost."""
        self.X_raw.append(x)
        self.X_norm.append(self._normalize(x))
        self.y.append(cost)

    def suggest(self) -> Dict[str, float]:
        """
        Suggests the next optimal coordinate set to test.
        Uses the Lower Confidence Bound (LCB) acquisition function.
        """
        # 1. Fallback to exploration: first 3 trials are randomly sampled
        if len(self.y) < 3:
            rand_norm = tuple(random.random() for _ in self.keys)
            return self._denormalize(rand_norm)

        # 2. Build the GP covariance matrix K (n x n)
        n = len(self.y)
        K = [[0.0] * n for _ in range(n)]
        for r in range(n):
            for c in range(n):
                cov = self._rbf_kernel(self.X_norm[r], self.X_norm[c])
                if r == c:
                    cov += self.noise  # Add observation noise variance
                K[r][c] = cov

        # 3. Solve the GP mean weight vector: K_inv * y
        # In our case, solve K * alpha = y
        try:
            alpha = solve_linear(K, self.y)
        except Exception:
            # Fallback to random search on numerical failure
            rand_norm = tuple(random.random() for _ in self.keys)
            return self._denormalize(rand_norm)

        # 4. Generate candidate grid search over parameter space (10x10 or 12x12 grid)
        # We perform exhaustive grid search to find the minimum of the acquisition function
        grid_dim = 12
        candidates: List[Tuple[float, ...]] = []
        
        # Symmetrical grid generation for 2D bounds
        if len(self.keys) == 2:
            for i in range(grid_dim):
                for j in range(grid_dim):
                    candidates.append((i / (grid_dim - 1), j / (grid_dim - 1)))
        else:
            # Slower generic random candidate pool for multi-dimensional bounds
            for _ in range(150):
                candidates.append(tuple(random.random() for _ in self.keys))

        best_candidate: Optional[Tuple[float, ...]] = None
        best_lcb = float("inf")

        # 5. Evaluate acquisition function (LCB) for all candidates
        for x_cand in candidates:
            # k_star = covariance between candidate and all tested points
            k_star = [self._rbf_kernel(x_cand, xi) for xi in self.X_norm]

            # Compute predicted mean: k_star^T * alpha
            pred_mean = sum(ki * ai for ki, ai in zip(k_star, alpha))

            # Compute predicted variance: 1.0 - k_star^T * K_inv * k_star
            # Solve K * w = k_star to obtain w = K_inv * k_star
            try:
                w = solve_linear(K, k_star)
                pred_var = 1.0 - sum(ki * wi for ki, wi in zip(k_star, w))
                pred_std = math.sqrt(max(0.0, pred_var))
            except Exception:
                pred_std = 0.2  # conservative fallback standard deviation

            # LCB Acquisition Function (Minimize cost)
            lcb = pred_mean - self.kappa * pred_std

            if lcb < best_lcb:
                best_lcb = lcb
                best_candidate = x_cand

        if best_candidate is None:
            best_candidate = tuple(random.random() for _ in self.keys)

        return self._denormalize(best_candidate)

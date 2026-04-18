import numpy as np
import time

import os
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

N = 2048
FLOP = 2 * N * N * N

if __name__ == "__main__":
    A = np.random.randn(N, N).astype(np.float32)
    B = np.random.randn(N, N).astype(np.float32)

    for i in range(100):
        st = time.monotonic()
        C = A @ B
        et = time.monotonic()
        print(f"{N} GFLOP/S: {FLOP / (et - st) / 1e9:.2f}")
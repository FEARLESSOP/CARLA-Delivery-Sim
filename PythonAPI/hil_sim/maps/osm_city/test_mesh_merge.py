"""Mesh.merge after triangles were dropped (landmark drop_over): kept triangles must stay on
their own vertices. Regression for the 2026-11-21 bug (indices made local by ii.min()).
三角形削除後の Mesh.merge で残った三角形が別の頂点へずれないことの確認。

Run: maps/.venv/bin/python osm_city/test_mesh_merge.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh import Mesh  # noqa: E402


def main():
    m = Mesh()
    for x0 in (0.0, 5.0):
        P = [(x0, 0, 0), (x0 + 1, 0, 0), (x0 + 1, 1, 0), (x0, 1, 0)]
        m.tris("a", P, (0, 0, 1), [(0, 0)] * 4, [(0, 1, 2), (0, 2, 3)])
    part = m.parts["a"]
    V, N, UV, I = (np.concatenate(part[k]) for k in range(4))
    part[0], part[1], part[2], part[3] = [V], [N], [UV], [I[2:]]   # drop the first quad
    out = Mesh()
    out.merge(m)
    Vo, Io = np.concatenate(out.parts["a"][0]), np.concatenate(out.parts["a"][3])
    xs = Vo[Io].reshape(-1, 3)[:, 0]
    ok = xs.min() == 5.0 and xs.max() == 6.0 and len(Io) == 2
    print("PASS" if ok else "FAIL", "kept triangles x range %.1f-%.1f" % (xs.min(), xs.max()))
    if not ok:
        sys.exit(1)
    print("ALL PASS")


if __name__ == "__main__":
    main()

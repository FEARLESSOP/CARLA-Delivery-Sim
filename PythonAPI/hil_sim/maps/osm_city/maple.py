"""Japanese maple (イロハモミジ, Acer palmatum) trees: a branched trunk and leaf
cluster cards, with drawn textures for the seasons.

The recipe was learned from a SketchUp maple the user gave as a reference
(2026-11-04, ~/Downloads/oita kosen/maple-tree; studied only, nothing copied):
  - one trunk to a fork at ~25-30 % of the height, 4-6 limbs rising 35-60 deg,
    two orders of side branches; bark is a small share of the triangles but
    reads as the tree's structure through the gaps
  - the crown is leaf cards only: ~1600 double-sided 0.36 m cards, each one
    texture of a few leaves with an alpha cut-out, in an ellipsoidal shell
    (card centres r 1.0-2.5 m, z 3.2-6.8 m for an 8.2 m tree), oriented at
    random with a bias outward and up (|n.z| median 0.5)
Here: fewer, larger cards (0.75 m, ~11 small leaves each) for the budget,
~2.5 k triangles a tree; the leaves are a separate mesh so CARLA can swap
their texture at runtime (seasons, apply_color_texture_to_objects) and a
post-import step can give them CARLA's foliage master material (masked,
two-sided, sways with the weather's wind).

Coordinates: the map frame (x east, y north, z up, metres).

イロハモミジ: 幹・枝と葉のカード、季節ごとの描画テクスチャ。
"""

import math

import numpy as np

LEAF_MAT = "maple_leaves"          # summer texture in the FBX; seasons swap it live / 夏の葉（季節は実行時に差し替え）
BARK_MAT = "maple_bark"
CARD = 0.75                        # leaf card side (m) / 葉カードの一辺
SEASONS = ("spring", "summer", "autumn_early", "autumn", "winter")


def _tube(m, mat, p0, p1, r0, r1, sides):
    """A tapered tube from p0 to p1, faces out, open ends. / 先細りの管（外向き）。"""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    ln = np.linalg.norm(ax)
    if ln < 1e-6:
        return
    ax /= ln
    ref = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(ax, ref)
    u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    for i in range(sides):
        a0, a1 = 2 * math.pi * i / sides, 2 * math.pi * (i + 1) / sides
        d0, d1 = u * math.cos(a0) + v * math.sin(a0), u * math.cos(a1) + v * math.sin(a1)
        q = [p0 + d0 * r0, p0 + d1 * r0, p1 + d1 * r1, p1 + d0 * r1]
        n = np.cross(q[1] - q[0], q[3] - q[0])
        if float(n @ (d0 + d1)) < 0:          # wind it outward / 外向きに
            q = [q[1], q[0], q[3], q[2]]
        m.quad(mat, *q, (i / sides, 0), ((i + 1) / sides, 0), ((i + 1) / sides, ln / 1.5), (i / sides, ln / 1.5))


def _card(m, mat, c, n, up, size):
    """A double-sided square card centred at c, normal n: two quads 6 mm apart,
    facing opposite ways (back-to-back coplanar faces render black in Cycles).
    両面の葉カード（6 mm 離した 2 枚）。"""
    n = n / np.linalg.norm(n)
    t = np.cross(up, n)
    if np.linalg.norm(t) < 1e-6:
        t = np.cross(np.array([1.0, 0.0, 0.0]), n)
    t /= np.linalg.norm(t)
    b = np.cross(n, t)
    h = size / 2
    for s in (1.0, -1.0):
        o = c + n * 0.003 * s
        q = [o - t * h - b * h, o + t * h - b * h, o + t * h + b * h, o - t * h + b * h]
        uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
        if s < 0:
            q = [q[1], q[0], q[3], q[2]]
            uv = [uv[1], uv[0], uv[3], uv[2]]
        m.quad(mat, *q, *uv)


def tree(bark, leaves, x, y, seed, height=None, crown=None, z0=-0.2):
    """One maple at (x, y): trunk and branches into ``bark``, leaf cards into
    ``leaves`` (separate meshes, same frame). ``crown`` caps the crown radius.
    Returns (height, crown radius). / 1 本のモミジ。"""
    rs = np.random.default_rng(abs(int(seed)) & 0xFFFFFFFF)   # map coordinates go negative / 座標は負にもなる
    H = float(height if height is not None else rs.uniform(5.5, 7.5))
    R = 0.45 * H if crown is None else min(0.45 * H, crown)
    hf = H * rs.uniform(0.24, 0.30)                     # fork / 分岐高
    r0 = 0.05 + 0.012 * H
    lean = rs.normal(0, 0.06, 2)
    base = np.array([x, y, z0])
    fork = np.array([x + lean[0] * hf, y + lean[1] * hf, hf])
    _tube(bark, BARK_MAT, base, fork, r0 * 1.15, r0 * 0.85, 8)
    # crown ellipsoid / 樹冠の楕円体
    cz = hf + (H - hf) * 0.52
    rz = (H - hf) * 0.5
    cc = np.array([fork[0], fork[1], cz])
    tips = []
    nl = int(rs.integers(4, 7))
    az0 = rs.uniform(0, 2 * math.pi)
    for i in range(nl):
        az = az0 + 2 * math.pi * i / nl + rs.uniform(-0.3, 0.3)
        el = math.radians(rs.uniform(35, 60))
        ln = R * rs.uniform(0.85, 1.1) / math.cos(el) * 0.75
        d = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
        p, r = fork.copy(), r0 * 0.6
        pts = [p]
        for k in range(4):                               # a limb curving upward / 上に反る大枝
            d = d + np.array([0, 0, 0.12]) + rs.normal(0, 0.08, 3)
            d /= np.linalg.norm(d)
            q = p + d * ln / 4
            _tube(bark, BARK_MAT, p, q, r, r * 0.78, 6)
            pts.append(q)
            p, r = q, r * 0.78
        tips.append(p)
        for f in (0.4, 0.62, 0.84):                       # side branches / 小枝
            a = pts[0] + (pts[-1] - pts[0]) * f
            j = min(3, int(f * 4))
            a = pts[j] + (pts[j + 1] - pts[j]) * (f * 4 - j)
            for s in (-1, 1):
                sd = np.cross(d, [0, 0, 1])
                sd = sd / (np.linalg.norm(sd) + 1e-9) * s + d * 0.6 + np.array([0, 0, 0.35]) + rs.normal(0, 0.15, 3)
                sd /= np.linalg.norm(sd)
                bl = ln * rs.uniform(0.25, 0.4) * (1.1 - f * 0.5)
                e = a + sd * bl
                rb = r0 * 0.6 * (0.78 ** (f * 4)) * 0.5
                _tube(bark, BARK_MAT, a, e, rb, rb * 0.5, 5)
                tips.append(e)
    # leaf cards round the twig tips, kept in the crown shell / 小枝の先の周りに葉カード
    target = int(420 * (R / 3.0) ** 2 * (rz / 2.5))
    tips = np.array(tips)
    placed = 0
    tries = 0
    while placed < target and tries < target * 6:
        tries += 1
        tp = tips[rs.integers(len(tips))]
        c = tp + rs.normal(0, [0.55 * R / 3, 0.55 * R / 3, 0.45 * rz / 2.5])
        e = (c - cc) / np.array([R, R, rz])
        k = float(np.linalg.norm(e))
        if k > 1.0 or k < 0.42 or c[2] < hf + 0.4:
            continue
        out = e / max(k, 1e-6)
        n = out * 0.6 + np.array([0, 0, 0.5]) + rs.normal(0, 0.6, 3)
        if np.linalg.norm(n) < 1e-3:
            continue
        up = rs.normal(0, 1, 3)
        _card(leaves, LEAF_MAT, c, n, up / np.linalg.norm(up), CARD * rs.uniform(0.85, 1.15))
        placed += 1
    return H, R


# ---- textures / テクスチャ
# per-season leaf colours (Japanese maple in Kyushu): fresh green in spring,
# deep green in summer, turning in early November, red-orange 紅葉 in late
# November, bare in winter (a few dry leaves) / 季節ごとの葉の色
PALETTES = {
    "spring": [(150, 200, 85), (170, 212, 100), (132, 186, 72), (186, 120, 80)],
    "summer": [(70, 122, 46), (58, 104, 40), (86, 136, 56)],
    "autumn_early": [(96, 140, 52), (196, 172, 52), (222, 128, 36), (190, 60, 34)],
    "autumn": [(196, 38, 30), (176, 28, 34), (224, 94, 30), (230, 150, 42), (158, 24, 26)],
    "winter": [(140, 92, 56), (120, 80, 50)],
}
KEEP = {"winter": 0.0}             # share of leaves left on the twigs: bare in winter / 冬は落葉


def leaf_shape(cx, cy, size, rot, lobes=7):
    """A palmate maple leaf outline: deep pointed lobes, fine teeth. / 掌状の葉の輪郭。"""
    pts = []
    n = lobes * 24
    for i in range(n):
        a = -math.pi * 0.95 + 1.9 * math.pi * i / n          # open at the stalk / 葉柄側は開く
        ph = (a + math.pi * 0.95) / (1.9 * math.pi) * lobes
        lobe = abs(math.sin(math.pi * ph))
        r = size * (0.32 + 0.68 * lobe ** 2.2)
        r *= 1.0 + 0.06 * (1 if (i % 3 == 0) else -0.3) * lobe       # teeth / 鋸歯
        r *= 1.0 - 0.25 * (abs(a) / math.pi) ** 2                     # lower lobes shorter / 下の裂片は短い
        pts.append((cx + r * math.sin(a + rot), cy - r * math.cos(a + rot)))
    return pts


def leaf_texture(season, n=512, seed=4):
    """RGBA card texture: a twig with three side twigs and ~20 small leaves in
    opposite pairs, in the season's colours; transparent between the leaves.
    季節の葉カード（RGBA）: 小枝 4 本・対生の葉 約 20 枚。"""
    from PIL import Image, ImageDraw, ImageFilter
    rs = np.random.default_rng(seed)
    im = Image.new("RGBA", (n, n), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    pal = PALETTES[season]
    keep = KEEP.get(season, 1.0)
    twig = (120, 104, 90, 255)
    # the main twig up the card, side twigs off it / 主軸と側枝
    main = [(n * 0.5, n * 0.99)]
    for k in range(5):
        main.append((main[-1][0] + rs.uniform(-0.04, 0.04) * n, main[-1][1] - 0.165 * n))
    twigs = [main]
    for j, s in ((1, -1), (2, 1), (3, -1)):
        bx, by = main[j]
        t = [(bx, by)]
        for k in range(3):
            t.append((t[-1][0] + s * rs.uniform(0.06, 0.09) * n, t[-1][1] - rs.uniform(0.04, 0.07) * n))
        twigs.append(t)
    for t in twigs:
        d.line(t, fill=twig, width=max(3, n // 120))
    leaves = []
    for t in twigs:
        for j in range(1, len(t)):
            bx, by = t[j]
            dx, dy = t[j][0] - t[j - 1][0], t[j][1] - t[j - 1][1]
            ang = math.atan2(dx, -dy)
            for s in (-1, 1):                                   # opposite pair / 対生
                a = ang + s * rs.uniform(0.6, 1.1)
                L = rs.uniform(0.085, 0.12) * n
                cx, cy = bx + math.sin(a) * L * 1.5, by - math.cos(a) * L * 1.5
                if min(cx, cy, n - cx, n - cy) < L * 1.05:       # whole leaf inside the card / カード内に収める
                    continue
                leaves.append((bx, by, cx, cy, L, a))
    for bx, by, cx, cy, L, a in sorted(leaves, key=lambda v: v[3]):
        if rs.random() > keep:
            continue
        d.line([(bx, by), (cx, cy)], fill=twig, width=max(2, n // 220))
        base = np.array(pal[int(rs.integers(len(pal)))], float) * rs.uniform(0.88, 1.08)
        col = tuple(int(c) for c in np.clip(base, 0, 255)) + (255,)
        d.polygon(leaf_shape(cx, cy, L, a + rs.uniform(-0.4, 0.4)), fill=col)
        dark = tuple(int(c * 0.75) for c in col[:3]) + (255,)
        for v in range(7):                                      # veins / 葉脈
            va = a - math.pi * 0.8 + 1.6 * math.pi * (v + 0.5) / 7
            d.line([(cx, cy), (cx + math.sin(va) * L * 0.75, cy - math.cos(va) * L * 0.75)], fill=dark, width=1)
    arr = np.asarray(im).astype(np.float64)
    al = arr[..., 3:4] / 255.0
    # bleed colour under the transparent texels (premultiplied blur) so mips do
    # not halo; alpha hard (the masked material clips) / 透明部へ色を滲ませ、アルファは二値
    pre = Image.fromarray(np.clip(arr[..., :3] * al, 0, 255).astype(np.uint8))
    ab = Image.fromarray((al[..., 0] * 255).astype(np.uint8))
    num = np.asarray(pre.filter(ImageFilter.GaussianBlur(8)), np.float64)
    den = np.asarray(ab.filter(ImageFilter.GaussianBlur(8)), np.float64)[..., None] / 255.0
    bleed = np.where(den > 1e-3, num / np.maximum(den, 1e-3), np.array(PALETTES[season][0], float))
    hard = al[..., 0] > 0.5
    out = np.zeros((n, n, 4), np.uint8)
    out[..., :3] = np.clip(np.where(hard[..., None], arr[..., :3], bleed), 0, 255).astype(np.uint8)
    out[..., 3] = np.where(hard, 255, 0)
    return Image.fromarray(out, "RGBA")


def bark_texture(n=256, seed=9):
    """Smooth grey-brown maple bark with faint vertical streaks and lenticels. / 樹皮。"""
    from PIL import Image
    import textures as X
    a = X.tint(X.noise(n, (4, 16, 64), (0.5, 0.3, 0.2)), (92, 82, 72), (128, 116, 102)).astype(float)
    rs = np.random.default_rng(seed)
    for _ in range(40):
        x = int(rs.integers(0, n))
        a[:, x:x + int(rs.integers(1, 4))] *= rs.uniform(0.82, 0.95)
    for _ in range(60):
        x, y = int(rs.integers(0, n - 6)), int(rs.integers(0, n - 2))
        a[y:y + 2, x:x + 6] *= 1.12
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))


def write_textures(tex_dir):
    """The FBX gets the summer leaves and the bark; every season's card is
    written beside them for the runtime swap (hil_sim/seasons.py) and the
    post-import material step. / 夏の葉と樹皮（FBX 用）、全季節の葉（実行時用）。"""
    import os
    for s in SEASONS:
        leaf_texture(s).save(os.path.join(tex_dir, "maple_leaves_%s.png" % s))
    leaf_texture("summer").convert("RGB").save(os.path.join(tex_dir, LEAF_MAT + ".png"))
    bark_texture().save(os.path.join(tex_dir, BARK_MAT + ".png"))
    return {LEAF_MAT: (LEAF_MAT, None, 0.7, 0.0, False), BARK_MAT: (BARK_MAT, None, 0.9, 0.0, False)}

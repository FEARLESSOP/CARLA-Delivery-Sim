"""Japanese traffic cone (カラーコーン, 700 mm): mesh and drawn textures.

The proportions were learned from a SketchUp/FBX cone the user gave as a
reference (2026-11-04, ~/Downloads/oita kosen/3Dmodel-sketchup/.../japanese-
traffic-cone; studied only, nothing copied). In units of the height H:
  square rubber base 0.80 H wide, 0.055 H thick, bevelled up to a foot ring
  (r 0.34 H); the body from r 0.31 H at the foot to r 0.06 H at a flat top,
  slightly convex (radius kinks at 0.46, 0.60, 0.65 and 0.83 H); two white
  retro-reflective bands (0.46-0.60 H and 0.65-0.83 H) with an orange strip
  between; 「ご注意」 down one side; 32 segments, ~250 triangles, three
  materials (base, orange, white).
Real cones are 700 mm high on a ~380 mm base: that size is used here.

カラーコーン（700 mm）のメッシュと描画テクスチャ。参考モデルから比率を学習（複製なし）。
"""

import math

import numpy as np

H = 0.70                 # height (m) / 高さ
BASE = 0.38              # square base side / 台座の一辺
SEG = 32
# body profile (z / H, r / H), foot to top / 胴の断面（下から上）
PROFILE = [(0.11, 0.308), (0.46, 0.180), (0.60, 0.143), (0.65, 0.131), (0.83, 0.081), (1.0, 0.058)]
BANDS = [(0.46, 0.60), (0.65, 0.83)]          # white bands / 白い反射帯
MATS = ("jpcone_base", "jpcone_orange", "jpcone_white")


def cone(m, x, y, z=0.0, yaw=0.0, scale=1.0):
    """One cone standing at (x, y, z), its 「ご注意」 side facing yaw (rad).
    1 本のカラーコーン。"""
    h = H * scale
    b = BASE * scale / 2
    c, s = math.cos(yaw), math.sin(yaw)
    rot = lambda u, v: (x + c * u - s * v, y + s * u + c * v)
    # rubber base: a low box, then a bevel up to the foot ring / ゴム台座と面取り
    tb = 0.055 * h
    q = [rot(-b, -b), rot(b, -b), rot(b, b), rot(-b, b)]
    for i in range(4):
        (ax, ay), (bx, by) = q[i], q[(i + 1) % 4]
        m.quad(MATS[0], (ax, ay, z), (bx, by, z), (bx, by, z + tb), (ax, ay, z + tb), (0, 0), (1, 0), (1, 0.15), (0, 0.15))
    rf, zf = 0.34 * h, 0.09 * h
    ring = [rot(rf * math.cos(2 * math.pi * k / SEG), rf * math.sin(2 * math.pi * k / SEG)) for k in range(SEG)]
    # base top: from the square edge in to the foot ring, as quads per side / 台座上面
    for k in range(SEG):
        a0, a1 = 2 * math.pi * k / SEG, 2 * math.pi * (k + 1) / SEG
        e0 = _square_hit(a0, b)
        e1 = _square_hit(a1, b)
        p0, p1 = rot(*e0), rot(*e1)
        m.quad(MATS[0], (p0[0], p0[1], z + tb), (p1[0], p1[1], z + tb), (ring[(k + 1) % SEG][0], ring[(k + 1) % SEG][1], z + zf),
               (ring[k][0], ring[k][1], z + zf), (e0[0] / b * 0.5 + 0.5, e0[1] / b * 0.5 + 0.5),
               (e1[0] / b * 0.5 + 0.5, e1[1] / b * 0.5 + 0.5), (0.5 + 0.4 * math.cos(a1), 0.5 + 0.4 * math.sin(a1)),
               (0.5 + 0.4 * math.cos(a0), 0.5 + 0.4 * math.sin(a0)))
    # body rings, split at the band edges so each band is its own material / 胴（帯の境で分割）
    zs = sorted({p[0] for p in PROFILE} | {e for bd in BANDS for e in bd})
    rr = lambda t: float(np.interp(t, [p[0] for p in PROFILE], [p[1] for p in PROFILE]))
    for t0, t1 in zip(zs, zs[1:]):
        mat = MATS[2] if any(a <= t0 and t1 <= bb for a, bb in BANDS) else MATS[1]
        r0, r1 = rr(t0) * h, rr(t1) * h
        for k in range(SEG):
            a0, a1 = 2 * math.pi * k / SEG, 2 * math.pi * (k + 1) / SEG
            p = [rot(r0 * math.cos(a0), r0 * math.sin(a0)), rot(r0 * math.cos(a1), r0 * math.sin(a1)),
                 rot(r1 * math.cos(a1), r1 * math.sin(a1)), rot(r1 * math.cos(a0), r1 * math.sin(a0))]
            # u round the cone (0 at the 「ご注意」 side), v up / u は周方向、v は高さ
            u0, u1 = k / SEG, (k + 1) / SEG
            m.quad(mat, (p[0][0], p[0][1], z + t0 * h), (p[1][0], p[1][1], z + t0 * h), (p[2][0], p[2][1], z + t1 * h),
                   (p[3][0], p[3][1], z + t1 * h), (u0, t0), (u1, t0), (u1, t1), (u0, t1))
    # flat top / 天面
    rt = PROFILE[-1][1] * h
    ctr = (x, y, z + h)
    for k in range(SEG):
        a0, a1 = 2 * math.pi * k / SEG, 2 * math.pi * (k + 1) / SEG
        p0, p1 = rot(rt * math.cos(a0), rt * math.sin(a0)), rot(rt * math.cos(a1), rt * math.sin(a1))
        m.tris(MATS[1], [(p0[0], p0[1], z + h), (p1[0], p1[1], z + h), ctr], (0, 0, 1),
               [(0.0, 0.99), (0.01, 0.99), (0.005, 1.0)], [0, 1, 2])


def _square_hit(a, b):
    """Where a ray from the centre at angle a meets the square of half side b. / 正方形との交点。"""
    ca, sa = math.cos(a), math.sin(a)
    t = b / max(abs(ca), abs(sa))
    return ca * t, sa * t


def write_textures(tex_dir):
    """Orange body (u round, v up) with 「ご注意」 in white down the front, white
    reflective bands with faint micro-prism grain, a black rubber base with
    dust. Drawn here. / 胴（ご注意）・反射帯・ゴム台座のテクスチャ。"""
    import os
    from PIL import Image, ImageDraw, ImageFont
    import textures as X
    n = 512
    rs = np.random.default_rng(700)
    body = X.tint(X.noise(n, (4, 16, 64), (0.5, 0.3, 0.2)), (214, 96, 28), (236, 120, 40)).astype(float)
    im = Image.fromarray(np.clip(body, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(im)
    try:
        f = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 46)
        # v = height fraction: the text sits between the foot (0.11) and the lower band (0.46)
        # 文字は足元と下の帯の間
        for i, ch in enumerate("ご注意"):
            vy = 0.40 - i * 0.095
            d.text((n * 0.5 - 23, n * (1 - vy) - 23), ch, fill=(246, 244, 238), font=f)
    except OSError:
        pass
    im.save(os.path.join(tex_dir, "jpcone_orange.png"))
    wh = X.tint(X.noise(256, (2, 8, 32), (0.5, 0.3, 0.2)), (228, 230, 232), (246, 247, 248)).astype(float)
    wh[::4, :] *= 0.97                                   # micro-prism rows / マイクロプリズム
    Image.fromarray(np.clip(wh, 0, 255).astype(np.uint8)).save(os.path.join(tex_dir, "jpcone_white.png"))
    bs = X.tint(X.noise(256, (4, 16, 64), (0.5, 0.3, 0.2)), (34, 34, 34), (70, 68, 64)).astype(float)
    for _ in range(30):
        cx, cy = rs.integers(0, 256, 2)
        bs[max(0, cy - 6):cy + 6, max(0, cx - 10):cx + 10] *= 1.25         # dust / ほこり
    Image.fromarray(np.clip(bs, 0, 255).astype(np.uint8)).save(os.path.join(tex_dir, "jpcone_base.png"))
    # body smooth-shaded (32 flat facets read as a polygon) / 胴はスムーズシェード
    return {"jpcone_orange": ("jpcone_orange", None, 0.6, 0.0, True),
            "jpcone_white": ("jpcone_white", None, 0.3, 0.0, True),
            "jpcone_base": ("jpcone_base", None, 0.9, 0.0, False)}

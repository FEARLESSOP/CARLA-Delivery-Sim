"""Japanese trees with seasons: maple (モミジ), cherry (サクラ, Somei-yoshino),
ginkgo (イチョウ), zelkova (ケヤキ) and camphor (クスノキ, evergreen).

One generator for all five (the maple recipe of maple.py, learned from the
user's reference model): a trunk, limbs and twigs as tapered tubes, and the
crown as double-sided leaf-cluster cards on the outer shell. Per species:
the branching (fork height, limb angle, a central leader for the ginkgo), the
crown shape (round, wide umbrella, cone, vase), the leaf drawing and its
colours per season. Each species' leaves are a mesh of their own
(`Terrain_<Species>Leaves_<Map>_<tile>`) so CARLA can swap their texture
live (hil_sim/seasons.py) and the import gives them the foliage material
(maps/ue_foliage.py: masked, two-sided, sways with the weather's wind).

Seasons: hanami (花見, cherry in blossom, late March - early April), spring,
summer, autumn_early, autumn (紅葉 / 黄葉), winter (bare except the camphor).

Coordinates: the map frame (x east, y north, z up, metres).

日本の樹木 5 種（季節つき）。各種の葉は独立メッシュ（季節の差し替え・植生マテリアル用）。
"""

import math

import numpy as np

from maple import _card, _tube, leaf_shape

SEASONS = ("hanami", "spring", "summer", "autumn_early", "autumn", "winter")
# object kind and material prefix per species (no "sign"/"light" in names) / オブジェクト名
KIND = {"maple": "Terrain_MapleLeaves", "sakura": "Terrain_SakuraLeaves", "ginkgo": "Terrain_GinkgoLeaves",
        "zelkova": "Terrain_ZelkovaLeaves", "camphor": "Terrain_CamphorLeaves"}

# form per species / 樹形
FORM = {
    #          fork share   limb elevation  limbs   crown R/H  shape       cards per 9 m2*2.5 m  card m
    "maple":   dict(fork=(0.24, 0.30), el=(35, 60), limbs=(4, 7), rw=0.45, shape="round", dens=420, card=0.75),
    "sakura":  dict(fork=(0.16, 0.22), el=(18, 38), limbs=(4, 6), rw=0.62, shape="umbrella", dens=400, card=0.8),
    "ginkgo":  dict(fork=(0.10, 0.14), el=(40, 55), limbs=(9, 13), rw=0.26, shape="cone", dens=460, card=0.7),
    "zelkova": dict(fork=(0.22, 0.28), el=(58, 75), limbs=(5, 8), rw=0.45, shape="vase", dens=400, card=0.75),
    "camphor": dict(fork=(0.20, 0.26), el=(28, 52), limbs=(5, 7), rw=0.50, shape="round", dens=520, card=0.8),
}
HEIGHT = {"maple": (5.5, 7.5), "sakura": (6.0, 8.0), "ginkgo": (10.0, 14.0), "zelkova": (10.0, 14.0),
          "camphor": (9.0, 13.0)}


def dims(species, s=1.0, over=False, clear=4.9):
    """(crown radius, crown bottom, crown height) a tree of this species and
    size factor will have; props.py checks the overhang over roads with it.
    樹冠の半径・下端・高さ（道路上の張り出し判定用）。"""
    f = FORM[species]
    H = sum(HEIGHT[species]) / 2 * s
    R = f["rw"] * H
    bottom = H * sum(f["fork"]) / 2 + 0.4
    if over:
        R, bottom = min(R, 4.0), max(bottom, clear)
        H = max(H, bottom + 3.0)
    return R, bottom, H - bottom


def tree(bark, leaves, x, y, species, seed, height=None, crown=None, clear=None, lite=False, z0=-0.2):
    """One tree: trunk and branches into ``bark`` (material <species>_bark),
    leaf cards into ``leaves`` (<species>_leaves). ``crown`` caps the crown
    radius, ``clear`` is the lowest a leaf may hang (over roads), ``lite``
    for woods and the edge of the map (fewer, larger cards, no twigs).
    Returns (height, crown radius). / 1 本の木。"""
    f = FORM[species]
    rs = np.random.default_rng(abs(int(seed)) & 0xFFFFFFFF)
    H = float(height if height is not None else rs.uniform(*HEIGHT[species]))
    if clear is not None:
        H = max(H, clear + 3.0)
    R = f["rw"] * H if crown is None else min(f["rw"] * H, crown)
    hf = H * rs.uniform(*f["fork"])
    if clear is not None:
        # over a road: limbs start at the clearance (a low fork sends limbs out
        # under the crown, over the road) / 道路上: 大枝は離隔高さから出す
        hf = max(hf, clear - 0.2)
        H = max(H, hf + 3.0)
    zb = max(hf + 0.4, clear if clear is not None else 0.0)       # crown bottom / 樹冠下端
    bm, lm = species + "_bark", species + "_leaves"
    r0 = 0.06 + 0.014 * H
    lean = rs.normal(0, 0.05, 2)
    if clear is not None:
        # by a road the trunk stands straight: a leaning 12 m ginkgo leader put
        # its trunk over the road / 道路脇の幹は真っ直ぐ（傾いたイチョウの主幹が道路に出た）
        lean = lean * 0.0
    base = np.array([x, y, z0])
    top_trunk = hf if species != "ginkgo" else H * 0.9            # the ginkgo keeps a leader / イチョウは主幹
    fork = np.array([x + lean[0] * top_trunk, y + lean[1] * top_trunk, top_trunk])
    _tube(bark, bm, base, fork, r0 * 1.15, r0 * (0.85 if species != "ginkgo" else 0.3), 8)
    rz = (H - zb) * 0.5
    cz = zb + rz
    cc = np.array([x, y, cz])

    def inside(c):
        """Is a card centre inside the crown shell? / 樹冠の殻の中か。"""
        dz = (c[2] - zb) / max(H - zb, 1e-6)                       # 0 bottom .. 1 top
        if dz < 0 or dz > 1:
            return False
        rxy = math.hypot(c[0] - x, c[1] - y)
        if f["shape"] == "cone":
            ra = R * (1.0 - dz) ** 0.85 + 0.3
        elif f["shape"] == "vase":
            ra = R * (0.45 + 0.55 * math.sin(math.pi * min(1.0, dz * 0.85 + 0.15)) ** 0.6)
        elif f["shape"] == "umbrella":
            ra = R * math.sqrt(max(0.0, 1.0 - (2 * dz - 0.8) ** 2 / 1.44))
        else:
            ra = R * math.sqrt(max(0.0, 1.0 - (2 * dz - 1.0) ** 2))
        return 0.4 * ra <= rxy <= ra if ra > 0.3 else rxy <= ra

    tips = []
    nl = int(rs.integers(*f["limbs"]))
    az0 = rs.uniform(0, 2 * math.pi)
    for i in range(nl):
        az = az0 + 2 * math.pi * i / nl * (1.0 if species != "ginkgo" else 2.618) + rs.uniform(-0.3, 0.3)
        el = math.radians(rs.uniform(*f["el"]))
        if species == "ginkgo":
            # whorls up the leader, shorter higher up: the cone / 主幹に沿って段々に短く
            hz = zb + (H * 0.85 - zb) * (i + 0.5) / nl
            start = np.array([x + lean[0] * hz, y + lean[1] * hz, hz])
            ln = R * (1.0 - (hz - zb) / max(H - zb, 1e-6)) * 1.05 + 0.6
        else:
            start = fork.copy()
            ln = R * rs.uniform(0.85, 1.1) / max(math.cos(el), 0.3) * 0.75
        d = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
        # limbs end inside the crown (upright zelkova limbs poked out of the top)
        # 大枝は樹冠内で終わる（ケヤキの立った枝が上に突き出た）
        ln = min(ln, (H * 0.9 - start[2]) / max(math.sin(el) + 0.15, 0.3))
        p, r = start.copy(), r0 * (0.6 if species != "ginkgo" else 0.28)
        pts = [p]
        nseg = 3 if lite else 4
        for k in range(nseg):                            # a limb curving upward / 上に反る大枝
            lift = 0.12 if f["shape"] != "umbrella" else 0.02
            d = d + np.array([0, 0, lift]) + rs.normal(0, 0.08, 3)
            d /= np.linalg.norm(d)
            q = p + d * ln / nseg
            _tube(bark, bm, p, q, r, r * 0.78, 6 if not lite else 5)
            pts.append(q)
            p, r = q, r * 0.78
        tips.append(p)
        if lite:
            tips += [pts[1], pts[2]]
            continue
        for fr in (0.4, 0.62, 0.84):                      # side branches / 小枝
            j = min(nseg - 1, int(fr * nseg))
            a = pts[j] + (pts[j + 1] - pts[j]) * (fr * nseg - j)
            for sg in (-1, 1):
                sd = np.cross(d, [0, 0, 1])
                sd = sd / (np.linalg.norm(sd) + 1e-9) * sg + d * 0.6 + np.array([0, 0, 0.35]) + rs.normal(0, 0.15, 3)
                sd /= np.linalg.norm(sd)
                bl = ln * rs.uniform(0.25, 0.4) * (1.1 - fr * 0.5)
                e = a + sd * bl
                rb = r * 0.5
                _tube(bark, bm, a, e, rb, rb * 0.5, 5)
                tips.append(e)
    # leaf cards round the twig tips, in the crown shell / 小枝の先の周りに葉カード
    # big crowns take bigger cards, not more of them (budget ~1.2-2 k triangles
    # a tree, ~0.6 k lite) / 大きな樹冠はカードを大きく（枚数は抑える）
    grow = max(1.0, R / 3.0) * (1.35 if lite else 1.0)
    card = f["card"] * grow
    target = int(f["dens"] * (R / 3.0) ** 2 * (rz / 2.5) / grow ** 2 * (0.55 if lite else 1.0))
    target = max(40, min(target, 180 if lite else 600))
    tips = np.array(tips)
    placed = tries = 0
    while placed < target and tries < target * 8:
        tries += 1
        tp = tips[rs.integers(len(tips))]
        c = tp + rs.normal(0, [0.6 * R / 3 + 0.2, 0.6 * R / 3 + 0.2, 0.45 * rz / 2.5 + 0.1])
        if not inside(c) or c[2] - card * 0.6 < zb:
            continue
        out = (c - cc) / np.array([R, R, max(rz, 0.5)])
        out /= max(np.linalg.norm(out), 1e-6)
        n = out * 0.6 + np.array([0, 0, 0.5]) + rs.normal(0, 0.6, 3)
        if np.linalg.norm(n) < 1e-3:
            continue
        up = rs.normal(0, 1, 3)
        _card(leaves, lm, c, n, up / np.linalg.norm(up), card * rs.uniform(0.85, 1.15))
        placed += 1
    return H, R


def pick(x, y, context):
    """A species for a tree at (x, y) by where it stands (deterministic from the
    position, so rebuilds agree). / 場所に応じた樹種（位置から決定的に）。"""
    h = ((int(round(x * 10)) * 73856093) ^ (int(round(y * 10)) * 19349663)) & 0xFFFF
    u = h / 65536.0
    table = {
        # street trees: ginkgo and zelkova avenues, some cherries / 街路樹
        "street": (("ginkgo", 0.42), ("zelkova", 0.30), ("sakura", 0.28)),
        # lawns, parks, gardens / 公園・庭
        "park": (("sakura", 0.32), ("maple", 0.26), ("camphor", 0.24), ("zelkova", 0.18)),
        # woods and the land outside the ring: mostly evergreen / 林・外周（常緑主体）
        "wood": (("camphor", 0.55), ("zelkova", 0.25), ("sakura", 0.10), ("maple", 0.10)),
    }[context]
    acc = 0.0
    for sp, w in table:
        acc += w
        if u < acc:
            return sp
    return table[-1][0]


# ---- textures / テクスチャ
# leaf colours per species and season (Kyushu); "flowers" draws blossom
# instead of leaves; keep = share of leaves on the twigs / 季節ごとの葉色
G_FRESH = [(150, 200, 85), (170, 212, 100), (132, 186, 72)]
G_DEEP = [(70, 122, 46), (58, 104, 40), (86, 136, 56)]
PAL = {
    "maple": {"hanami": (G_FRESH, 0.35), "spring": (G_FRESH + [(186, 120, 80)], 1.0), "summer": (G_DEEP, 1.0),
              "autumn_early": ([(96, 140, 52), (196, 172, 52), (222, 128, 36), (190, 60, 34)], 1.0),
              "autumn": ([(196, 38, 30), (176, 28, 34), (224, 94, 30), (230, 150, 42), (158, 24, 26)], 1.0),
              "winter": ([(140, 92, 56)], 0.0)},
    "sakura": {"hanami": ("flowers", 1.0), "spring": (G_FRESH + [(150, 110, 70)], 1.0), "summer": (G_DEEP, 1.0),
               "autumn_early": ([(110, 140, 60), (200, 150, 50), (210, 110, 50)], 1.0),
               "autumn": ([(200, 90, 50), (186, 60, 40), (214, 140, 60), (160, 70, 40)], 0.8),
               "winter": ([(120, 80, 50)], 0.0)},
    "ginkgo": {"hanami": ([(160, 206, 100)], 0.25), "spring": ([(160, 206, 100), (176, 214, 110)], 1.0),
               "summer": ([(82, 136, 52), (96, 150, 60)], 1.0),
               "autumn_early": ([(150, 180, 60), (210, 196, 60), (226, 206, 70)], 1.0),
               "autumn": ([(240, 206, 44), (246, 216, 60), (232, 190, 36)], 1.0), "winter": ([(200, 170, 60)], 0.0)},
    "zelkova": {"hanami": (G_FRESH, 0.4), "spring": (G_FRESH, 1.0), "summer": (G_DEEP, 1.0),
                "autumn_early": ([(120, 150, 60), (190, 160, 60), (190, 120, 50)], 1.0),
                "autumn": ([(190, 110, 50), (170, 80, 40), (210, 150, 60), (150, 90, 50)], 0.85),
                "winter": ([(130, 90, 60)], 0.0)},
    "camphor": {"hanami": ([(64, 112, 44), (90, 140, 60)], 1.0),
                "spring": ([(64, 112, 44), (160, 190, 90), (170, 100, 70)], 1.0),
                "summer": ([(52, 100, 40), (64, 116, 46)], 1.0), "autumn_early": ([(52, 100, 40), (64, 116, 46)], 1.0),
                "autumn": ([(50, 96, 40), (62, 110, 46), (150, 60, 40)], 1.0),
                "winter": ([(48, 92, 38), (60, 106, 44)], 1.0)},
}


def _shape(species, cx, cy, L, a):
    """The leaf outline of a species (a list of points). / 種ごとの葉の輪郭。"""
    if species == "maple":
        return leaf_shape(cx, cy, L, a)
    pts = []
    n = 36
    for i in range(n):
        t = 2 * math.pi * i / n
        if species == "ginkgo":
            # fan: a wedge from the stalk, wavy outer edge, a notch in the middle / 扇形
            th = -1.0 + 2.0 * i / (n - 1)
            r = L * (1.0 + 0.06 * math.sin(th * 9)) * (0.82 if abs(th) < 0.12 else 1.0)
            ang = a + th * 0.95
            if i == 0:
                pts.append((cx - math.sin(a) * L * 0.9, cy + math.cos(a) * L * 0.9))
            pts.append((cx - math.sin(a) * L * 0.9 + math.sin(ang) * r * 1.4,
                        cy + math.cos(a) * L * 0.9 - math.cos(ang) * r * 1.4))
            continue
        # ovate with a pointed tip, serrated (zelkova strong, sakura fine, camphor smooth)
        # 卵形（ケヤキは粗い鋸歯、サクラは細かい鋸歯、クスノキは全縁）
        w = {"zelkova": 0.42, "sakura": 0.5, "camphor": 0.48}[species]
        tooth = {"zelkova": 0.08, "sakura": 0.04, "camphor": 0.0}[species]
        u, v = math.cos(t), math.sin(t)
        r = 1.0 + tooth * (1 if i % 2 else -1)
        px = u * L * (1.0 + 0.25 * max(0.0, u))                    # pointed tip / 先端が尖る
        py = v * L * w * (1.0 - 0.35 * max(0.0, u) ** 2) * r
        ca, sa = math.cos(a - math.pi / 2), math.sin(a - math.pi / 2)
        pts.append((cx + px * ca - py * sa, cy + px * sa + py * ca))
    return pts


def leaf_texture(species, season, n=512, seed=4):
    """RGBA card: twigs with leaves (or blossom) of the species and season;
    transparent between, hard alpha, colour bled under the clear texels.
    種と季節の葉カード（RGBA）。"""
    from PIL import Image, ImageDraw, ImageFilter
    rs = np.random.default_rng(seed + sum(map(ord, species)))   # str hash is salted per run / 文字列ハッシュは毎回変わる
    im = Image.new("RGBA", (n, n), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    pal, keep = PAL[species][season]
    twig = (120, 104, 90, 255) if species != "sakura" else (88, 70, 66, 255)
    main = [(n * 0.5, n * 0.99)]
    for k in range(5):
        main.append((main[-1][0] + rs.uniform(-0.04, 0.04) * n, main[-1][1] - 0.165 * n))
    twigs = [main]
    for j, sg in ((1, -1), (2, 1), (3, -1)):
        bx, by = main[j]
        t = [(bx, by)]
        for k in range(3):
            t.append((t[-1][0] + sg * rs.uniform(0.06, 0.09) * n, t[-1][1] - rs.uniform(0.04, 0.07) * n))
        twigs.append(t)
    for t in twigs:
        d.line(t, fill=twig, width=max(3, n // 120))
    if pal == "flowers":
        # Somei-yoshino: pale pink five-petal flowers in clusters along the twigs
        # ソメイヨシノ: 淡い桃色の五弁花を小枝に沿って房状に
        for t in twigs:
            for j in range(1, len(t)):
                for _ in range(5):
                    cx = t[j][0] + rs.normal(0, 0.035 * n)
                    cy = t[j][1] + rs.normal(0, 0.035 * n)
                    if min(cx, cy, n - cx, n - cy) < 0.04 * n:
                        continue
                    pr = rs.uniform(0.018, 0.026) * n
                    col = tuple(int(c) for c in np.clip(np.array((248, 214, 224)) * rs.uniform(0.93, 1.03), 0, 255))
                    for k in range(5):
                        ang = 2 * math.pi * k / 5 + rs.uniform(0, 1)
                        px, py = cx + math.cos(ang) * pr * 0.9, cy + math.sin(ang) * pr * 0.9
                        d.ellipse([px - pr * 0.75, py - pr * 0.75, px + pr * 0.75, py + pr * 0.75], fill=col + (255,))
                    d.ellipse([cx - pr * 0.3, cy - pr * 0.3, cx + pr * 0.3, cy + pr * 0.3], fill=(214, 110, 140, 255))
    else:
        leaves = []
        for t in twigs:
            for j in range(1, len(t)):
                bx, by = t[j]
                dx, dy = t[j][0] - t[j - 1][0], t[j][1] - t[j - 1][1]
                ang = math.atan2(dx, -dy)
                for sg in (-1, 1):
                    a = ang + sg * rs.uniform(0.6, 1.1)
                    L = rs.uniform(0.07, 0.10) * n if species != "camphor" else rs.uniform(0.075, 0.1) * n
                    cx, cy = bx + math.sin(a) * L * 1.5, by - math.cos(a) * L * 1.5
                    if min(cx, cy, n - cx, n - cy) < L * 1.2:
                        continue
                    leaves.append((bx, by, cx, cy, L, a))
        if species in ("camphor", "zelkova"):
            # dense crowns: a second layer of leaves round the twigs / 密な樹冠は葉を重ねる
            for bx, by, cx, cy, L, a in list(leaves):
                a2 = a + rs.uniform(-0.9, 0.9)
                c2 = (cx + rs.normal(0, L * 0.6), cy + rs.normal(0, L * 0.6))
                if min(c2[0], c2[1], n - c2[0], n - c2[1]) > L * 1.2:
                    leaves.append((bx, by, c2[0], c2[1], L * 0.9, a2))
        for bx, by, cx, cy, L, a in sorted(leaves, key=lambda v: v[3]):
            if rs.random() > keep:
                continue
            d.line([(bx, by), (cx, cy)], fill=twig, width=max(2, n // 220))
            base = np.array(pal[int(rs.integers(len(pal)))], float) * rs.uniform(0.88, 1.08)
            col = tuple(int(c) for c in np.clip(base, 0, 255)) + (255,)
            d.polygon(_shape(species, cx, cy, L, a + rs.uniform(-0.4, 0.4)), fill=col)
            dark = tuple(int(c * 0.75) for c in col[:3]) + (255,)
            d.line([(bx, by), (cx + math.sin(a) * L * 0.8, cy - math.cos(a) * L * 0.8)], fill=dark, width=1)
    arr = np.asarray(im).astype(np.float64)
    al = arr[..., 3:4] / 255.0
    pre = Image.fromarray(np.clip(arr[..., :3] * al, 0, 255).astype(np.uint8))
    ab = Image.fromarray((al[..., 0] * 255).astype(np.uint8))
    num = np.asarray(pre.filter(ImageFilter.GaussianBlur(8)), np.float64)
    den = np.asarray(ab.filter(ImageFilter.GaussianBlur(8)), np.float64)[..., None] / 255.0
    fb = np.array(G_DEEP[0] if pal == "flowers" else pal[0], float)
    bleed = np.where(den > 1e-3, num / np.maximum(den, 1e-3), fb)
    hard = al[..., 0] > 0.5
    out = np.zeros((n, n, 4), np.uint8)
    out[..., :3] = np.clip(np.where(hard[..., None], arr[..., :3], bleed), 0, 255).astype(np.uint8)
    out[..., 3] = np.where(hard, 255, 0)
    return Image.fromarray(out, "RGBA")


BARK = {"maple": ((92, 82, 72), (128, 116, 102)), "sakura": ((62, 48, 46), (96, 76, 70)),
        "ginkgo": ((96, 88, 78), (132, 122, 108)), "zelkova": ((112, 104, 92), (150, 140, 124)),
        "camphor": ((70, 60, 50), (104, 92, 78))}


def bark_texture(species, n=256, seed=9):
    """Bark per species: smooth grey-brown maple and zelkova (zelkova flaking),
    dark cherry with horizontal lenticels, furrowed ginkgo and camphor. / 樹皮。"""
    from PIL import Image
    import textures as X
    c0, c1 = BARK[species]
    a = X.tint(X.noise(n, (4, 16, 64), (0.5, 0.3, 0.2)), c0, c1).astype(float)
    rs = np.random.default_rng(seed + len(species))
    if species == "sakura":
        for _ in range(70):                                          # lenticels / 皮目
            x, y = int(rs.integers(0, n - 14)), int(rs.integers(0, n - 2))
            a[y:y + 2, x:x + int(rs.integers(6, 14))] *= 1.3
    elif species in ("ginkgo", "camphor"):
        for _ in range(60):                                          # furrows / 縦の溝
            x = int(rs.integers(0, n))
            a[:, x:x + int(rs.integers(1, 3))] *= rs.uniform(0.65, 0.8)
    elif species == "zelkova":
        for _ in range(40):                                          # flakes / 剥がれ
            x, y = int(rs.integers(0, n - 12)), int(rs.integers(0, n - 20))
            a[y:y + int(rs.integers(8, 20)), x:x + int(rs.integers(5, 12))] *= rs.uniform(1.1, 1.25)
    else:
        for _ in range(40):
            x = int(rs.integers(0, n))
            a[:, x:x + int(rs.integers(1, 4))] *= rs.uniform(0.82, 0.95)
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))


def write_textures(tex_dir, species=tuple(KIND)):
    """Per species: the summer card (FBX), every season's card (runtime swap
    and the foliage step) and the bark. / 種ごとに夏の葉・全季節の葉・樹皮。"""
    import os
    out = {}
    for sp in species:
        for s in SEASONS:
            leaf_texture(sp, s).save(os.path.join(tex_dir, "%s_leaves_%s.png" % (sp, s)))
        leaf_texture(sp, "summer").convert("RGB").save(os.path.join(tex_dir, sp + "_leaves.png"))
        bark_texture(sp).save(os.path.join(tex_dir, sp + "_bark.png"))
        out[sp + "_leaves"] = (sp + "_leaves", None, 0.7, 0.0, False)
        out[sp + "_bark"] = (sp + "_bark", None, 0.9, 0.0, False)
    return out


def leafsets():
    """An empty leaf mesh per species. / 種ごとの空の葉メッシュ。"""
    from mesh import Mesh
    return {sp: Mesh() for sp in KIND}


def landmark_tree(bark, leaves, rigid, x, y, r, context, lite=False):
    """A tree for a landmark module from an imagery crown of radius r: the
    species by place, the height from the crown, the crown kept within r - 0.4
    (the module already cut r to stay off roads and paths). ``rigid(x, y, rr)``
    gives a Mesh lifted to the ground at the foot; ``leaves`` is leafsets().
    Returns the species. / 詳細モデル用の木（画像の樹冠半径から）。"""
    sp = pick(x, y, context)
    lo, hi = HEIGHT[sp]
    hgt = float(np.clip(r / FORM[sp]["rw"], lo * 0.6, hi * 1.1))
    lm = rigid(x, y, 0.4)
    tree(bark, lm, x, y, sp, seed=int(x * 10) * 31 + int(y * 10), height=hgt, crown=max(1.0, r - 0.4), lite=lite)
    leaves[sp].merge(lm)
    return sp

"""Singapore road surfaces and road paint (maps/.venv), for maps.json "sg_lta"
(guide ch. 51). Procedural only (textures_jp's generators, own random stream and
colours), nothing taken from photos. One shared set for the whole map (a 24 km2
map must not grow textures per road): 12 materials, ~23 MB of PNG (main, expressway
and estate asphalt 1024 px; wheel paths and patches 512 px).

Tiles (metres per repeat, ground.py uv_scale): asphalt 5 m, footpath 3 m, kerb
top 1 m, kerb face 2 m (two 1 m precast units), paint 1 m.

  asphalt_sg          main roads: dense-graded, mid grey (Street View: lighter and
                      greyer than Japan's)
  asphalt_sg_wheel    their wheel paths: darker, polished, smoother
  asphalt_sg_expwy    expressways: stone-mastic, lighter, coarse grey aggregate
  asphalt_sg_xwheel   expressway wheel paths
  asphalt_sg_estate   estate roads: oxidised, lighter, ravelled
  asphalt_sg_crack    their outer 0.6 m: edge and alligator cracking
  asphalt_sg_patch    repairs and trench reinstatements: new, black
  kerb_sg / kerb_face_sg   precast concrete kerb, 1 m units, grime at road level
  footpath_sg         in-situ concrete footpath, broom finish, saw-cut joints every
                      1.5 m (HDB estates, Street View)
  line_white_sg / line_yellow_sg   thermoplastic paint (3 mm, glass beads), worn;
                      LTA yellow (not Japan's orange-yellow)

シンガポールの路面・縁石・歩道・路面標示（手続き生成、共有テクスチャ）。
"""

import numpy as np

from textures_jp import asphalt, concrete, cracks, paint, save, smooth

# name: (texture, colour, roughness, metallic, smooth) as textures.MATERIALS / 材質表
MATERIALS = {
    "asphalt_sg": ("asphalt_sg", None, 0.86, 0.0, False),
    "asphalt_sg_wheel": ("asphalt_sg_wheel", None, 0.74, 0.0, False),
    "asphalt_sg_expwy": ("asphalt_sg_expwy", None, 0.88, 0.0, False),
    "asphalt_sg_xwheel": ("asphalt_sg_xwheel", None, 0.76, 0.0, False),
    "asphalt_sg_estate": ("asphalt_sg_estate", None, 0.9, 0.0, False),
    "asphalt_sg_crack": ("asphalt_sg_crack", None, 0.9, 0.0, False),
    "asphalt_sg_patch": ("asphalt_sg_patch", None, 0.82, 0.0, False),
    "kerb_sg": ("kerb_sg", None, 0.85, 0.0, False),
    "kerb_face_sg": ("kerb_face_sg", None, 0.85, 0.0, False),
    "footpath_sg": ("footpath_sg", None, 0.86, 0.0, False),
    "line_white_sg": ("line_white_sg", None, 0.55, 0.0, False),
    "line_yellow_sg": ("line_yellow_sg", None, 0.58, 0.0, False),
}
TILE = {"asphalt": 5.0, "footpath": 3.0, "kerb": 1.0, "kerb_face": 2.0, "paint": 1.0}


def write(d):
    rng = np.random.default_rng(65)
    n = 1024
    col, h, r = asphalt(n, rng, (47, 47, 49), (92, 91, 88), rough=(0.86, 0.06))
    save(d, "asphalt_sg", col, h, r, 1.6)
    # wheel paths and patches: narrow pieces, 512 px is enough (content size) / 轍・補修は 512 px
    col, h, r = asphalt(512, rng, (40, 40, 42), (78, 77, 75), size=0.6, polish=0.35, tone=0.08, rough=(0.74, 0.05))
    save(d, "asphalt_sg_wheel", col, h, r, 1.2)
    # expressway: coarser, lighter grey stone, more open texture / 高速: 粗い明るい骨材
    col, h, r = asphalt(n, rng, (56, 56, 57), (112, 110, 106), density=0.42, size=1.2, tone=0.10,
                        pits=0.010, rough=(0.88, 0.05))
    save(d, "asphalt_sg_expwy", col, h, r, 1.9)
    col, h, r = asphalt(512, rng, (48, 48, 50), (96, 95, 92), density=0.42, size=0.7, polish=0.4, tone=0.08,
                        pits=0.006, rough=(0.76, 0.05))
    save(d, "asphalt_sg_xwheel", col, h, r, 1.4)
    col, h, r = asphalt(n, rng, (60, 59, 57), (104, 101, 96), density=0.40, size=1.1, tone=0.14,
                        pits=0.012, rough=(0.9, 0.05))
    save(d, "asphalt_sg_estate", col, h, r, 1.9)
    k = cracks(n, rng, cells=12, cover=0.5)
    save(d, "asphalt_sg_crack", col * (1.0 - 0.6 * k[..., None]), h - 2.2 * k, r + 0.06 * k, 2.0)
    col, h, r = asphalt(512, rng, (35, 35, 37), (68, 67, 65), density=0.28, size=0.5, tone=0.05,
                        pits=0.002, rough=(0.82, 0.04))
    save(d, "asphalt_sg_patch", col, h, r, 1.3)
    # kerb top / face: light grey precast concrete, 1 m units on a 2 m face tile,
    # grime at road level (image bottom = v 0 = the road; the face uses 0..0.15 m)
    # 縁石: 明るい灰色のプレキャスト、1 m ごとの目地、路面際の汚れ
    col, h, r = concrete(512, rng, (160, 159, 153), (190, 189, 183))
    save(d, "kerb_sg", col, h, r, 1.6)
    col, h, r = concrete(512, rng, (165, 164, 158), (192, 191, 185))
    yy = np.arange(512)[:, None] / 511.0
    grime = np.clip((yy - 0.9) / 0.1, 0, 1) ** 1.5
    col *= (1.0 - 0.45 * grime)[..., None]
    for x in (0, 256):
        for dx in range(-2, 3):
            col[:, (x + dx) % 512] *= 0.65
            h[:, (x + dx) % 512] -= 1.2
    save(d, "kerb_face_sg", col, h, r + 0.04 * grime, 1.6)
    # footpath: broom-finished concrete, saw-cut joints every 1.5 m on a 3 m tile
    # 歩道: ほうき目仕上げのコンクリート、1.5 m ごとの目地
    col, h, r = concrete(1024, rng, (150, 149, 143), (182, 180, 173), pores=0.003)
    broom = smooth(1024, 0.6, rng)
    broom = np.repeat(broom[:, :1], 1024, axis=1) * 0.6 + 0.4 * broom          # fine lines across / 細い筋
    col += 5.0 * broom[..., None]
    h = h + 0.4 * broom
    for x in (0, 512):
        for dx in range(-2, 3):
            col[:, (x + dx) % 1024] *= 0.7
            col[(x + dx) % 1024, :] *= 0.7
            h[:, (x + dx) % 1024] -= 1.5
            h[(x + dx) % 1024, :] -= 1.5
    save(d, "footpath_sg", col, h, r, 1.5)
    col, h, r = paint(512, rng, (238, 238, 232), wear=0.06)
    save(d, "line_white_sg", col, h, r, 1.4)
    col, h, r = paint(512, rng, (236, 192, 40), wear=0.07)
    save(d, "line_yellow_sg", col, h, r, 1.4)

"""Check a RecastBuilder navmesh (.bin) against Detour's id limits (guide ch. 7).

  python check_navmesh.py <Map>.bin [recast.log]   (any venv; exit 1 on a problem)

CARLA's RecastBuilder writes 51.2 m tiles; tile and polygon ids share 22 bits of a
32-bit ref (tile bits = log2 of the next power of two >= the tile grid). A tile
with polyCount >= maxPolys would give polygons ids that run into the next tile's
(walkers path through wrong polygons). Big maps (~9,600 tiles: maxPolys 256) need
maps.json "nav" walkable_only (nav_obj.py). A grid larger than maxTiles drops tiles
("failed with status: 80000004" = out of slots in RecastBuilder's log): a full slot
table (tiles == maxTiles) fails, and with the log every failed tile is counted.
Prints tiles, maxTiles / maxPolys and the polys per tile (max, p99, mean).

RecastBuilder のナビメッシュ (.bin) が Detour の ID 上限内かを確認する
（タイルあたりのポリゴン数 < maxPolys）。問題があれば終了コード 1。
"""
import struct
import sys


def tiles(path):
    """(header dict, [polyCount per tile]) / ヘッダとタイルごとのポリゴン数"""
    b = open(path, "rb").read()
    magic, ver, n = struct.unpack_from("<iii", b, 0)
    if magic != 0x4D534554:                         # 'MSET'
        raise ValueError("not a Recast tile set: %s" % path)
    orig = struct.unpack_from("<3f", b, 12)
    tw, th = struct.unpack_from("<2f", b, 24)
    max_t, max_p = struct.unpack_from("<ii", b, 32)
    off, pc = 40, []
    for _ in range(n):
        size = struct.unpack_from("<i", b, off + 4)[0]   # 32-bit tile ref, then the data size
        off += 8
        head = struct.unpack_from("<iiiiiIii", b, off)   # dtMeshHeader: magic .. polyCount
        if head[0] != 0x444E4156:                        # 'DNAV'
            raise ValueError("tile %d: bad dtMeshHeader" % len(pc))
        pc.append(head[6])
        off += size
    return {"tiles": n, "origin": orig, "tile_m": (tw, th), "max_tiles": max_t, "max_polys": max_p}, pc


def main():
    h, pc = tiles(sys.argv[1])
    s = sorted(pc) or [0]
    over = sum(p >= h["max_polys"] for p in pc)
    print("[navmesh] %d tiles (max %d), tile %.1f m, maxPolys %d; polys/tile max %d p99 %d mean %.0f; over the limit: %d"
          % (h["tiles"], h["max_tiles"], h["tile_m"][0], h["max_polys"], s[-1], s[int(0.99 * (len(s) - 1))],
             sum(s) / len(s), over))
    failed = 0
    if len(sys.argv) > 2:
        failed = open(sys.argv[2], errors="replace").read().count("failed with status")
        print("[navmesh] RecastBuilder log: %d tiles failed" % failed)
    ok = over == 0 and failed == 0 and h["tiles"] < h["max_tiles"]
    if h["tiles"] >= h["max_tiles"]:
        print("[navmesh] every tile slot used: the grid was probably larger (tiles dropped)")
    print("[navmesh] %s" % ("OK" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

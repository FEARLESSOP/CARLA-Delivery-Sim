"""OpenStreetMap features in the map's frame (x east, y north, metres).

OSM の地物（建物・歩道・土地利用・横断歩道など）を地図座標で読む。
"""

import xml.etree.ElementTree as ET

from shapely.geometry import LineString, Polygon, Point
from shapely.ops import linemerge, polygonize, unary_union
from shapely.validation import make_valid

import geo
import osmfix


class OSM(object):
    def __init__(self, path, meta):
        fr = geo.Frame(meta)
        root = ET.parse(path).getroot()
        osmfix.apply(root, meta, quiet=True)     # maps.json "osm_fixes" / 地図ごとの OSM 修正
        self.nodes, self.ntags = {}, {}
        for n in root.iter("node"):
            self.nodes[n.get("id")] = fr.xy(float(n.get("lon")), float(n.get("lat")))
            t = {k.get("k"): k.get("v") for k in n.findall("tag")}
            if t:
                self.ntags[n.get("id")] = t
        self.ways = {}
        for w in root.iter("way"):
            refs = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in self.nodes]
            self.ways[w.get("id")] = (refs, {k.get("k"): k.get("v") for k in w.findall("tag")})
        self.rels = []
        for r in root.iter("relation"):
            mem = [(m.get("type"), m.get("ref"), m.get("role")) for m in r.findall("member")]
            self.rels.append((mem, {k.get("k"): k.get("v") for k in r.findall("tag")}))

    def line(self, refs):
        return LineString([self.nodes[r] for r in refs]) if len(refs) >= 2 else None

    def poly(self, refs):
        if len(refs) < 4 or refs[0] != refs[-1]:
            return None
        p = make_valid(Polygon([self.nodes[r] for r in refs]))
        return p if not p.is_empty and p.area > 0.5 else None

    def lines(self, key):
        """(LineString, tags, way id) for open or closed ways with ``key``. / 線の地物。"""
        for wid, (refs, t) in self.ways.items():
            if key in t:
                ln = self.line(refs)
                if ln is not None:
                    yield ln, t, wid

    def areas(self, test):
        """(polygon, tags) for closed ways and multipolygon relations passing ``test``. / 面の地物。"""
        for p, t, _ in self.areas_id(test):
            yield p, t

    def areas_id(self, test):
        """(polygon, tags, way id or None for a relation), as areas(). / 面の地物と way id。"""
        for wid, (refs, t) in self.ways.items():
            if test(t):
                p = self.poly(refs)
                if p is not None:
                    yield p, t, wid
        for mem, t in self.rels:
            if t.get("type") != "multipolygon" or not test(t):
                continue
            outer = [self.line(self.ways[ref][0]) for typ, ref, role in mem
                     if typ == "way" and role == "outer" and ref in self.ways]
            inner = [self.line(self.ways[ref][0]) for typ, ref, role in mem
                     if typ == "way" and role == "inner" and ref in self.ways]
            outer = [l for l in outer if l is not None]
            inner = [l for l in inner if l is not None]
            if not outer:
                continue
            o = unary_union(list(polygonize(linemerge(outer))))
            if inner:
                o = o.difference(unary_union(list(polygonize(linemerge(inner)))))
            o = make_valid(o)
            if not o.is_empty and o.area > 0.5:
                yield o, t, None

    def points(self, test):
        for nid, t in self.ntags.items():
            if test(t):
                yield Point(self.nodes[nid]), t, nid

    def ways_through(self, nid):
        return [(refs, t) for refs, t in self.ways.values() if nid in refs]

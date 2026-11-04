"""Live preview of map builds in Blender's window: reloads tiles as build.py rewrites them.

  blender --python live_preview.py -- <out_dir> [<out_dir> ...]

Each <out_dir> (build/<Name> or a scratch copy) becomes its own collection
(hide one in the outliner to compare). Every few seconds the meshes folder is
checked; when build.py has finished writing (manifest.json newer than the
tiles and quiet for a few seconds), only the changed tiles, materials and
textures are reloaded. Loading uses blender_build.py's own functions, so the
preview shows exactly what goes into the FBX (preview material wiring).

ビルド中の地図を Blender の画面で自動再読み込みして確認する。
"""

import json
import os
import sys
import time

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
POLL = 5.0      # seconds between checks
QUIET = 4.0     # files must be untouched this long before loading

# blender_build.py runs main() on import: take its functions without it
_src = open(os.path.join(HERE, "blender_build.py")).read()
BB = {"__name__": "blender_build"}
exec(compile(_src.rsplit("\nmain()", 1)[0], "blender_build.py", "exec"), BB)


class Watch:
    def __init__(self, out):
        self.out = os.path.abspath(out)
        self.tag = os.path.basename(os.path.dirname(self.out)) + "/" + os.path.basename(self.out)
        self.col = bpy.data.collections.new(self.tag)
        bpy.context.scene.collection.children.link(self.col)
        self.seen = {}      # object name -> npz mtime
        self.mats = {}      # material name -> (spec, bpy material)
        self.imgs = {}      # image path -> mtime

    def mname(self, k):
        return self.tag + ":" + k

    def ready(self):
        man = os.path.join(self.out, "meshes", "manifest.json")
        if not os.path.exists(man):
            return None
        mt = os.path.getmtime(man)
        newest = max(os.path.getmtime(e.path) for e in os.scandir(os.path.join(self.out, "meshes")))
        if newest > mt + 1.0 or time.time() - newest < QUIET:
            return None     # build.py still writing
        return man

    def material(self, k, spec):
        old = self.mats.get(k)
        if old and old[0] == spec:
            return old[1]
        if old:
            bpy.data.materials.remove(old[1])
        m = BB["material"](k, spec, os.path.join(self.out, "textures"), ue=False)
        m.name = self.mname(k)
        self.mats[k] = (spec, m)
        return m

    def reload_images(self):
        n = 0
        tex = os.path.join(self.out, "textures") + os.sep
        for img in bpy.data.images:
            p = bpy.path.abspath(img.filepath)
            if not p.startswith(tex) or not os.path.exists(p):
                continue
            mt = os.path.getmtime(p)
            if self.imgs.get(p, mt) != mt:
                img.reload()
                n += 1
            self.imgs[p] = mt
        return n

    def update(self):
        man_path = self.ready()
        if not man_path:
            return False
        try:
            man = json.load(open(man_path))
        except ValueError:
            return False    # half-written manifest: next round
        mats = {k: self.material(k, tuple(v) if isinstance(v, list) else v)
                for k, v in man["materials"].items()}
        smooth = {k for k, v in man["materials"].items() if v[4]}
        want = {o["name"] for o in man["objects"]}
        changed = 0
        for name in list(self.seen):
            if name not in want:
                self.drop(name)
                changed += 1
        for o in man["objects"]:
            path = os.path.join(self.out, "meshes", o["name"] + ".npz")
            if not os.path.exists(path):
                continue
            mt = os.path.getmtime(path)
            if self.seen.get(o["name"]) == mt:
                continue
            try:
                self.drop(o["name"])
                ob = BB["build_object"](o["name"], path, mats, smooth)
            except Exception as e:      # tile being rewritten: retry next round
                print("live_preview: skip", o["name"], e)
                continue
            bpy.context.scene.collection.objects.unlink(ob)
            self.col.objects.link(ob)
            ob.name = ob.data.name = self.mname(o["name"])
            self.seen[o["name"]] = mt
            changed += 1
        imgs = self.reload_images()
        if changed or imgs:
            print("live_preview: %s  %d tiles, %d textures reloaded  (%s)"
                  % (self.tag, changed, imgs, time.strftime("%H:%M:%S")))
        return bool(changed or imgs)

    def drop(self, name):
        ob = bpy.data.objects.get(self.mname(name))
        if ob:
            me = ob.data
            bpy.data.objects.remove(ob)
            bpy.data.meshes.remove(me)
        self.seen.pop(name, None)


def setup_view():
    for area in bpy.context.screen.areas:
        if area.type != "VIEW_3D":
            continue
        for sp in area.spaces:
            if sp.type == "VIEW_3D":
                sp.clip_start, sp.clip_end = 0.1, 5000
                sp.shading.type = "MATERIAL"
        region = next(r for r in area.regions if r.type == "WINDOW")
        with bpy.context.temp_override(area=area, region=region):
            bpy.ops.view3d.view_all()


def main():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if not a:
        print(__doc__)
        return
    bpy.ops.wm.read_factory_settings(use_empty=True)
    watches = [Watch(d) for d in a]
    if bpy.app.background:      # self-test: load once and report (timers need the window)
        for w in watches:
            w.update()
            print("live_preview: %s  %d objects" % (w.tag, len(w.col.objects)))
        return
    first = [True]

    def tick():
        for w in watches:
            if w.update() and first[0]:
                first[0] = False
                setup_view()
        return POLL

    bpy.app.timers.register(tick, first_interval=1.0, persistent=True)
    print("live_preview: watching", ", ".join(w.out for w in watches))


main()

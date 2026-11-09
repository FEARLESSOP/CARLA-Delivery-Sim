"""Import one props package with CARLA's Import.py machinery, without touching Import/.

Util/BuildTools/Import.py imports EVERY .json under <carla>/Import as a
package, whatever --package says (car guide ch. 6); build_cars.sh parks the
other folders while it runs, which would pull them away from a map build
running at the same time. This calls Import.py's own functions for just our
package JSON, which may live anywhere (here: Dist/<Pkg>_stage), and runs in
that folder so its importsetting.json can't collide with another import.

  cd <stage> && UE4_ROOT=... <carla-venv>/python import_props_package.py <carla_src> <stage>/<Pkg>.json

Import.py の関数を直接呼び、Import/ を触らずに 1 パッケージだけ取り込む。
"""

import importlib.util
import os
import sys

carla_src, pkg_json = sys.argv[1], os.path.abspath(sys.argv[2])
spec = importlib.util.spec_from_file_location(
    "carla_import", os.path.join(carla_src, "Util", "BuildTools", "Import.py"))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
mod.import_assets_from_json_list([[os.path.dirname(pkg_json), os.path.basename(pkg_json)]], 1)
print("imported", pkg_json)

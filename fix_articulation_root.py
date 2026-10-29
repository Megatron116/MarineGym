"""Report (and optionally fix) where ArticulationRootAPI sits in the Spheroid USDs.

MarineGym's RobotBase.spawn() requires ArticulationRootAPI on the asset's default (top) prim.
Run with Isaac Sim's python.bat from the repo root:
    python.bat .claude/scratch/fix_articulation_root.py          # report only
    python.bat .claude/scratch/fix_articulation_root.py --fix    # move root to the top prim and save
"""
import sys

from omni.isaac.kit import SimulationApp

app = SimulationApp({"headless": True})  # loads the USD / PhysX schema libraries

from pxr import Usd, UsdPhysics, PhysxSchema

FILES = [
    "marinegym/robots/assets/usd/Spheroid/Spheroid.usd",
    "marinegym/robots/assets/usd/Spheroid/fixed_usd/Spheroid/Spheroid.usd",
]
FIX = "--fix" in sys.argv

for path in FILES:
    stage = Usd.Stage.Open(path)
    top = stage.GetDefaultPrim()
    roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    fixed_joints = [p.GetPath().pathString for p in stage.Traverse()
                    if p.IsA(UsdPhysics.FixedJoint)]
    print(f"== {path}")
    print(f"   default prim:       {top.GetPath() if top else None}")
    print(f"   articulation roots: {[r.GetPath().pathString for r in roots]}")
    print(f"   fixed joints:       {fixed_joints}")
    if not FIX:
        continue
    if not top:
        print("   !! no default prim; not changing anything")
        continue
    for r in roots:
        if r != top:
            r.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if r.HasAPI(PhysxSchema.PhysxArticulationAPI):
                r.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    UsdPhysics.ArticulationRootAPI.Apply(top)
    PhysxSchema.PhysxArticulationAPI.Apply(top)
    stage.GetRootLayer().Save()
    print(f"   -> articulation root now on {top.GetPath()} (saved)")

app.close()

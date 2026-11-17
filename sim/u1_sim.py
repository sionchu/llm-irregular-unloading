# U1 container-unloading benchmark (physics). Builds one seeded single-layer box wall at a container door,
# settles it, renders the camera observation (+ numbered marks, detection table), and executes a pick sequence
# with a front-face suction pad (Isaac Surface Gripper, velocity-servoed). Measures picks, collapses, leftovers.
#   C:\isaacsim\python.bat u1_sim.py --seed 100 --obs                     -> obs/wall_100.png, obs/wall_100_table.json
#   C:\isaacsim\python.bat u1_sim.py --seed 100 --plan plans/rule_100.json -> results/rule_100.json
import argparse
import json
import os

import numpy as np
from isaacsim import SimulationApp

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, required=True)
ap.add_argument("--obs", action="store_true")
ap.add_argument("--plan", default="")
ap.add_argument("--frames", default="", help="dir to dump frames of the execution (video)")
ap.add_argument("--datagen", default="", help="dir for VLA training samples (rule expert, raw images)")
ap.add_argument("--vla", default="", help="handshake dir for closed-loop policy evaluation")
ap.add_argument("--show", default="", help="dir for showcase frames: same physics, plus a display-only KR210 on a mobile "
                                           "base at a dock door and an outside camera (arm posed by Lula IK, no collision)")
ap.add_argument("--show_every", type=int, default=2, help="save every n-th physics update in --show mode")
ap.add_argument("--max_picks", type=int, default=0)
args = ap.parse_args()
RES = tuple(int(v) for v in os.environ.get("U1_RES", "1280x720").split("x"))
app = SimulationApp({"headless": True, "width": RES[0], "height": RES[1], "renderer": "RaytracedLighting"})
import carb.settings

_cs = carb.settings.get_settings()
_cs.set("/rtx/post/aa/op", 2)
_cs.set("/rtx/post/dlss/execMode", 0)
_cs.set("/rtx/post/motionblur/enabled", False)
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot.surface_gripper")
if args.show:
    enable_extension("isaacsim.robot_motion.lula")
    enable_extension("isaacsim.robot_motion.motion_generation")
app.update()
import isaacsim.core.experimental.utils.app as app_utils
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.surface_gripper import GripperView, _surface_gripper
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
from scipy.spatial.transform import Rotation, Slerp
from usd.schema.isaac import robot_schema

HERE = os.path.dirname(os.path.abspath(__file__))
SC = os.environ.get("U1_SC", "")
SFX = f"_{SC}" if SC else ""
# U2 simple grip model applied to polybags only (assumption): (p_no_grip, p_drop_in_transit); A = single pad, C = 4-zone cup
GRIP_P = {"A": (0.342, 0.052), "C": (0.155, 0.0)}.get(os.environ.get("U1_GRIP", ""))
GRNG = np.random.default_rng(args.seed * 7 + 13)
for d in ("obs", "results", "plans"):
    os.makedirs(os.path.join(HERE, d + SFX), exist_ok=True)
WALL = json.load(open(os.path.join(HERE, "walls" + SFX, f"wall_{args.seed}.json"), encoding="utf-8"))
B = WALL["boxes"]
PAD = (0.36, 0.30, 0.06)
LIMIT = 1500.0  # U1 isolates ORDER reasoning: suction limit set high on purpose (stated on the card)
CAM_EYE, CAM_TGT, FOCAL = np.array([0.0, -3.3, 1.30]), np.array([0.0, 0.3, 1.22]), 12.5

ctx = omni.usd.get_context()
ctx.new_stage()
st = ctx.get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
st.SetDefaultPrim(UsdGeom.Xform.Define(st, "/World").GetPrim())
sc = UsdPhysics.Scene.Define(st, "/World/physicsScene")
sc.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
sc.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim()).CreateTimeStepsPerSecondAttr(120)
_m = {}


def mat(name, rgb, rough=0.7, metal=0.0):
    if name in _m:
        return _m[name]
    m = UsdShade.Material.Define(st, f"/World/Looks/{name}")
    s = UsdShade.Shader.Define(st, f"/World/Looks/{name}/S")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    _m[name] = m
    return m


def mesh_box(path, sx, sy, sz, guide=True):
    m = UsdGeom.Mesh.Define(st, path)
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz), (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
    if guide:
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def mesh_cyl(path, rad, length, axis):
    m = UsdGeom.Mesh.Define(st, path)
    pts = []
    for zz in (-length / 2, length / 2):
        for i in range(16):
            a, b_ = rad * np.cos(2 * np.pi * i / 16), rad * np.sin(2 * np.pi * i / 16)
            pts.append((a, zz, b_) if axis == "y" else (a, b_, zz))
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    m.CreateFaceVertexCountsAttr([16, 16] + [4] * 16)
    m.CreateFaceVertexIndicesAttr(list(range(15, -1, -1)) + list(range(16, 32)) + [v for i in range(16) for v in (i, (i + 1) % 16, 16 + (i + 1) % 16, 16 + i)])
    m.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    return m


def vis_cyl(path, rad, length, axis, material):
    c = UsdGeom.Cylinder.Define(st, path)
    c.CreateRadiusAttr(float(rad))
    c.CreateHeightAttr(float(length))
    c.CreateAxisAttr("Y" if axis == "y" else "Z")
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)


def vis_cube(path, size, center, material):
    x = UsdGeom.Xform.Define(st, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    c = UsdGeom.Cube.Define(st, path + "/g")
    c.CreateSizeAttr(1.0)
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)


def static_box(path, size, center, material, visible=True):
    x = UsdGeom.Xform.Define(st, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    mesh_box(path + "/col", *size)
    if visible:
        vis_cube(path + "/vis", size, (0, 0, 0), material)


# container section (corrugated steel look kept simple)
steel = mat("Container", (0.55, 0.20, 0.12), 0.6, 0.3)
floor_m = mat("Plywood", (0.45, 0.33, 0.22), 0.8)
static_box("/World/C/floor", (2.6, 3.4, 0.1), (0, 0.9, -0.05), floor_m)
static_box("/World/C/left", (0.1, 3.4, 2.8), (-1.20, 0.9, 1.4), steel)
static_box("/World/C/right", (0.1, 3.4, 2.8), (1.20, 0.9, 1.4), steel)
static_box("/World/C/roof", (2.5, 3.4, 0.1), (0, 0.9, 2.74), steel)
static_box("/World/C/back", (2.4, 0.1, 2.7), (0, 0.72, 1.35), mat("Backstop", (0.30, 0.25, 0.20), 0.9))
static_box("/World/Dock", (6.0, 3.0, 0.1), (0, -2.0, -0.05), mat("DockFloor", (0.35, 0.36, 0.37), 0.7))
static_box("/World/Removed", (40.0, 4.0, 0.1), (24.0, -2.5, -0.05), mat("DockFloor", (0.35, 0.36, 0.37), 0.7), visible=False)
d = UsdLux.DomeLight.Define(st, "/World/Dome")
d.CreateIntensityAttr(500)
for i, x in enumerate((-0.6, 0.6)):
    r = UsdLux.RectLight.Define(st, f"/World/L{i}")
    r.CreateWidthAttr(0.6)
    r.CreateHeightAttr(0.6)
    r.CreateIntensityAttr(25000)
    xf = UsdGeom.Xformable(r)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, -1.6, 2.4))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(70, 0, 0))

# boxes (front face at y=0, extending +y), cardboard tones vary per box
board = UsdShade.Material.Define(st, "/World/Looks/BoardPhys")
UsdPhysics.MaterialAPI.Apply(board.GetPrim()).CreateStaticFrictionAttr(0.55)
UsdPhysics.MaterialAPI(board.GetPrim()).CreateDynamicFrictionAttr(0.5)
bagm = UsdShade.Material.Define(st, "/World/Looks/BagPhys")
UsdPhysics.MaterialAPI.Apply(bagm.GetPrim()).CreateStaticFrictionAttr(0.30)
UsdPhysics.MaterialAPI(bagm.GetPrim()).CreateDynamicFrictionAttr(0.25)
rng = np.random.default_rng(args.seed)
for b in B:
    kind = b.get("kind", "box")
    p = f"/World/Boxes/b{b['id']}"
    x = UsdGeom.Xform.Define(st, p)
    x.AddTranslateOp().Set(Gf.Vec3d(b["x"], b["d"] / 2, b["z"]))
    x.AddRotateYOp().Set(float(b["lean"]))
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
    UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(b["mass"])
    PhysxSchema.PhysxRigidBodyAPI.Apply(x.GetPrim()).CreateSleepThresholdAttr(0.0)
    if kind == "cyl_v":
        col = mesh_cyl(p + "/col", b["w"] / 2, b["h"], "z")
    elif kind == "cyl_h":
        col = mesh_cyl(p + "/col", b["h"] / 2, b["d"], "y")
    else:
        col = mesh_box(p + "/col", b["w"], b["d"], b["h"])
    UsdShade.MaterialBindingAPI.Apply(col.GetPrim()).Bind(bagm if kind == "bag" else board, UsdShade.Tokens.weakerThanDescendants, "physics")
    tone = rng.uniform(0.85, 1.1)
    if args.show:  # textured cardboard (Isaac Simple_Warehouse box props, scaled to the physics box; visual only)
        _A = {"A": (0.70, 0.50, 0.50), "B": (0.50, 0.50, 0.50), "C": (0.50, 0.50, 0.25), "D": (0.38, 0.25, 0.149)}
        asp = b["h"] / max(b["w"], 1e-3)
        key = min(_A, key=lambda k: abs(_A[k][2] / _A[k][0] - asp) + 0.2 * abs(_A[k][1] / _A[k][0] - b["d"] / b["w"]))
        vx = UsdGeom.Xform.Define(st, p + "/vis")
        vx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -b["h"] / 2))
        vx.AddScaleOp().Set(Gf.Vec3f(b["w"] / _A[key][0], b["d"] / _A[key][1], b["h"] / _A[key][2]))
        vx.GetPrim().GetReferences().AddReference(
            "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/Props/"
            f"SM_CardBox{key}_01.usd")
    elif kind == "bag":
        vis_cube(p + "/vis", (b["w"], b["d"], b["h"]), (0, 0, 0), mat(f"Bag{b['id']}", (0.72 * tone, 0.76 * tone, 0.82 * tone), 0.35))
    elif kind == "cyl_v":
        vis_cyl(p + "/vis", b["w"] / 2, b["h"], "z", mat(f"Tube{b['id']}", (0.50 * tone, 0.38 * tone, 0.26 * tone), 0.8))
    elif kind == "cyl_h":
        vis_cyl(p + "/vis", b["h"] / 2, b["d"], "y", mat(f"Tube{b['id']}", (0.50 * tone, 0.38 * tone, 0.26 * tone), 0.8))
    else:
        vis_cube(p + "/vis", (b["w"], b["d"], b["h"]), (0, 0, 0), mat(f"Card{b['id']}", (0.62 * tone, 0.47 * tone, 0.30 * tone), 0.85))
        vis_cube(p + "/tape", (min(0.07, b["w"] * 0.2), 0.004, b["h"] * 0.98), (0, -b["d"] / 2 - 0.002, 0),
                 mat("Tape", (0.80, 0.70, 0.50), 0.4))

# suction pad (dynamic, no gravity, velocity servo) + surface gripper on its face; carrier as body1 anchor
eo = UsdGeom.Xform.Define(st, "/World/EOAT")
UsdPhysics.RigidBodyAPI.Apply(eo.GetPrim())
UsdPhysics.MassAPI.Apply(eo.GetPrim()).CreateMassAttr(8.0)
pe = PhysxSchema.PhysxRigidBodyAPI.Apply(eo.GetPrim())
pe.CreateDisableGravityAttr(True)
pe.CreateSleepThresholdAttr(0.0)
pe.CreateLinearDampingAttr(0.0)
pe.CreateAngularDampingAttr(0.0)
vis_cube("/World/EOAT/body", (PAD[0], PAD[1], 0.12), (0, 0, -0.06), mat("EoatBody", (0.7, 0.72, 0.75), 0.35, 0.6))
vis_cube("/World/EOAT/pad", PAD, (0, 0, PAD[2] / 2), mat("Foam", (0.05, 0.05, 0.05), 0.9))
c = mesh_box("/World/EOAT/col", PAD[0], PAD[1], PAD[2] + 0.12)
UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(0, 0, (PAD[2] - 0.12) / 2))
car = UsdGeom.Xform.Define(st, "/World/Carrier")
car.AddTranslateOp().Set(Gf.Vec3d(0, -5, 0.3))
UsdPhysics.RigidBodyAPI.Apply(car.GetPrim()).CreateKinematicEnabledAttr(True)
att = UsdPhysics.Joint.Define(st, "/World/Attach")
att.CreateBody0Rel().SetTargets(["/World/EOAT"])
att.CreateBody1Rel().SetTargets(["/World/Carrier"])
att.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, PAD[2]))
att.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
att.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
att.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
att.CreateExcludeFromArticulationAttr().Set(True)
for axn in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
    lim = UsdPhysics.LimitAPI.Apply(att.GetPrim(), axn)
    lim.CreateLowAttr().Set(1.0)
    lim.CreateHighAttr().Set(-1.0)
robot_schema.ApplyAttachmentPointAPI(att.GetPrim())
att.GetPrim().CreateAttribute(robot_schema.Attributes.FORWARD_AXIS.name, robot_schema.Attributes.FORWARD_AXIS.type).Set("Z")
att.GetPrim().CreateAttribute(robot_schema.Attributes.CLEARANCE_OFFSET.name, robot_schema.Attributes.CLEARANCE_OFFSET.type).Set(0.002)
sgp = robot_schema.CreateSurfaceGripper(st, "/World/SG")
sgp.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name).SetTargets(["/World/Attach"])

# camera
cam = UsdGeom.Camera.Define(st, "/World/Cam")
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100))
cam.CreateFocalLengthAttr(FOCAL)
HA = 20.955
cam.CreateHorizontalApertureAttr(HA)
cam.CreateVerticalApertureAttr(HA * RES[1] / RES[0])
z_ = CAM_EYE - CAM_TGT
z_ /= np.linalg.norm(z_)
x_ = np.cross([0, 0, 1], z_)
x_ /= np.linalg.norm(x_)
y_ = np.cross(z_, x_)
CM = np.eye(4)
CM[:3, 0], CM[:3, 1], CM[:3, 2], CM[:3, 3] = x_, y_, z_, CAM_EYE
UsdGeom.Xformable(cam.GetPrim()).AddTransformOp().Set(Gf.Matrix4d(CM.T.tolist()))


SHOW = None
if args.show:  # ---------------- showcase dressing (display only: no physics on anything added here) ----------------
    from pxr import Usd
    from scipy.spatial.transform import Rotation as _R

    ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
    LULA = r"C:\isaacsim\extsDeprecated\isaacsim.robot_motion.motion_generation\motion_policy_configs\Kuka\kr210"

    def _xf(path, pos=(0, 0, 0), yaw=0.0, ref=None):
        x = UsdGeom.Xform.Define(st, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.AddRotateZOp().Set(float(yaw))
        if ref:
            x.GetPrim().GetReferences().AddReference(ref)
        return x.GetPrim()

    def _strip(root):
        for _ in range(4):
            inst = [p for p in Usd.PrimRange(st.GetPrimAtPath(root)) if p.IsInstance()]
            if not inst:
                break
            for p in inst:
                p.SetInstanceable(False)
        for p in Usd.PrimRange(st.GetPrimAtPath(root)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
                p.RemoveAPI(UsdPhysics.CollisionAPI)
                if p.HasAPI(UsdPhysics.MeshCollisionAPI):
                    p.RemoveAPI(UsdPhysics.MeshCollisionAPI)
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if p.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(p).CreateJointEnabledAttr(False)

    def _vis(path, size, center, m):
        vis_cube(path, size, center, m)

    concrete, steel2 = mat("Concrete", (0.70, 0.70, 0.68), 0.85), mat("DockSteel", (0.25, 0.27, 0.30), 0.5, 0.5)
    yellow, dark = mat("SafetyYellow", (0.95, 0.75, 0.05), 0.5), mat("BaseGrey", (0.18, 0.19, 0.21), 0.5, 0.3)
    # Isaac Simple_Warehouse interior as the building (its +y wall removed; that side becomes our dock wall)
    WH_OFF = (0.0, -19.0, 0.0)
    _xf("/World/Show/wh", WH_OFF, 0.0, ROOT + "/Isaac/Environments/Simple_Warehouse/warehouse.usd")
    for _ in range(4):
        inst = [p_ for p_ in Usd.PrimRange(st.GetPrimAtPath("/World/Show/wh")) if p_.IsInstance()]
        if not inst:
            break
        for p_ in inst:
            p_.SetInstanceable(False)
    _cache = UsdGeom.BBoxCache(0, ["default", "render"])
    _root = st.GetPrimAtPath("/World/Show/wh")
    _kids = _root.GetChildren()
    if len(_kids) == 1 and not _kids[0].GetName().startswith("SM_"):
        _kids = _kids[0].GetChildren()
    for c_ in _kids:
        r_ = _cache.ComputeWorldBound(c_).ComputeAlignedRange()
        if r_.IsEmpty():
            continue
        nm_ = c_.GetName()
        ymin = r_.GetMin()[1] - WH_OFF[1]
        if ymin > 16.9 and nm_.startswith(("SM_WallA", "SM_WallWire", "SM_SignA", "SM_SignB", "SM_FuseBox", "SM_BracketBeam", "SM_BracketSlot")):
            c_.SetActive(False)
        elif nm_.startswith(("SM_WallWire", "Camera")):
            c_.SetActive(False)
        elif ymin > 12.5 and nm_.startswith(("SM_PaletteA", "KLT_Bins", "SM_RackFrame", "SM_RackShelf", "SM_Rackshield", "SM_CardBoxA",
                                              "SM_PushcartA", "S_AisleSign", "SM_SignCVer", "SM_FloorDecal", "S_Barcode")):
            c_.SetActive(False)
    _strip("/World/Show/wh")
    UsdGeom.Imageable(st.GetPrimAtPath("/World/Dock/vis")).MakeInvisible()
    # dock wall with a door opening framing the container mouth (y = -0.85), leveler, door seal
    wallm = mat("DockWall", (0.50, 0.53, 0.56), 0.55, 0.3)
    st.GetPrimAtPath("/World/Dome").GetAttribute("inputs:intensity").Set(220.0)
    for di, dx in enumerate((-4.4, 4.4)):  # neighbouring dock doors, closed (roll-up slats), bumpers, door lights
        for k in range(12):
            _vis(f"/World/Show/nd{di}_{k}", (2.6, 0.06, 0.24), (dx, -1.13, 0.13 + k * 0.25), mat("DoorSlat", (0.62, 0.64, 0.66), 0.4, 0.6))
        for bx in (-1.0, 1.0):
            _vis(f"/World/Show/bump{di}_{int(bx > 0)}", (0.25, 0.12, 0.45), (dx + bx, -1.18, 0.5), mat("Black", (0.03, 0.03, 0.03), 0.6))
        _vis(f"/World/Show/lampR{di}", (0.12, 0.06, 0.12), (dx + 1.6, -1.15, 2.4), mat("LampRed", (0.9, 0.1, 0.05), 0.4))
    _vis("/World/Show/lampG", (0.12, 0.06, 0.12), (1.6, -1.15, 2.4), mat("LampGreen", (0.1, 0.9, 0.2), 0.4))
    _vis("/World/Show/wallL", (9.1, 0.25, 9.0), (-5.9, -0.98, 4.5), wallm)
    _vis("/World/Show/wallR", (8.2, 0.25, 9.0), (5.45, -0.98, 4.5), wallm)
    _vis("/World/Show/header", (2.7, 0.25, 6.0), (0, -0.98, 6.0), wallm)
    _vis("/World/Show/door", (2.6, 0.08, 0.35), (0, -1.12, 3.05), mat("DoorSlat", (0.62, 0.64, 0.66), 0.4, 0.6))
    for i, x in enumerate((-1.36, 1.36)):
        for k in range(6):
            _vis(f"/World/Show/stripe{i}_{k}", (0.04, 0.02, 0.25), (x, -1.12, 0.2 + k * 0.5), mat("Black", (0.03, 0.03, 0.03), 0.6))
            _vis(f"/World/Show/stripeY{i}_{k}", (0.04, 0.02, 0.25), (x, -1.12, 0.45 + k * 0.5), yellow)
    _vis("/World/Show/sealL", (0.12, 0.35, 3.0), (-1.30, -0.80, 1.5), mat("Seal", (0.08, 0.08, 0.09), 0.9))
    _vis("/World/Show/sealR", (0.12, 0.35, 3.0), (1.30, -0.80, 1.5), mat("Seal", (0.08, 0.08, 0.09), 0.9))
    _vis("/World/Show/floor", (12.0, 9.0, 0.05), (0, -5.4, -0.03), mat("DockFloor2", (0.42, 0.43, 0.45), 0.8))
    _vis("/World/Show/leveler", (2.2, 0.6, 0.04), (0, -1.0, 0.0), steel2)
    for i, x in enumerate((-1.45, 1.45)):
        _vis(f"/World/Show/bollard{i}", (0.12, 0.12, 1.1), (x, -1.35, 0.55), yellow)
    # mobile base under the arm + rear take-away conveyor (where picked cases go)
    BASE_P = np.array([float(v) for v in os.environ.get("U1_BASE", "0.0,-2.8,0.62").split(",")])
    # mobile base: tracked chassis look (dark body, yellow edge, bumpers, exposed wheels), arm sits on its top plate
    _vis("/World/Show/base", (1.30, 1.70, 0.40), (0, BASE_P[1] - 0.1, 0.30), dark)
    _vis("/World/Show/baseTop", (1.34, 1.74, 0.06), (0, BASE_P[1] - 0.1, BASE_P[2] - 0.03), yellow)
    _vis("/World/Show/bumperF", (1.36, 0.08, 0.12), (0, BASE_P[1] + 0.80, 0.18), mat("Black", (0.03, 0.03, 0.03), 0.6))
    _vis("/World/Show/bumperR", (1.36, 0.08, 0.12), (0, BASE_P[1] - 1.00, 0.18), mat("Black", (0.03, 0.03, 0.03), 0.6))
    for i, (x, dy) in enumerate(((-0.70, 0.55), (0.70, 0.55), (-0.70, -0.75), (0.70, -0.75))):
        _vis(f"/World/Show/wheel{i}", (0.14, 0.34, 0.34), (x, BASE_P[1] + dy, 0.17), mat("Tyre", (0.05, 0.05, 0.05), 0.9))
    _vis("/World/Show/lidarF", (0.14, 0.14, 0.10), (0.52, BASE_P[1] + 0.72, 0.55), yellow)
    _vis("/World/Show/beacon", (0.08, 0.08, 0.20), (0.55, BASE_P[1] - 0.85, 0.72), mat("LampGreen", (0.1, 0.9, 0.2), 0.4))
    # take-away conveyor on the left of the base: belt + side guards + legs (static collider on the belt top)
    CONV_X = float(os.environ.get("U1_CONV_X", "-1.05"))
    CONV_TOP = float(os.environ.get("U1_CONV_TOP", "0.80"))
    CONV_Y0, CONV_Y1 = float(os.environ.get("U1_CONV_Y0", "-1.25")), -5.2
    CONV_Y = float(os.environ.get("U1_CONV_Y", "-1.85"))
    _clen = CONV_Y0 - CONV_Y1
    _cy = (CONV_Y0 + CONV_Y1) / 2
    static_box("/World/Show/convBelt", (0.62, _clen, 0.06), (CONV_X, _cy, CONV_TOP - 0.03), mat("Belt", (0.10, 0.10, 0.11), 0.8))
    for sx in (-1, 1):
        _vis(f"/World/Show/convGuard{int(sx > 0)}", (0.04, _clen, 0.12), (CONV_X + sx * 0.33, _cy, CONV_TOP + 0.03), mat("ConvBlue", (0.15, 0.35, 0.62), 0.4, 0.3))
    for k in range(5):
        yy = CONV_Y0 - 0.2 - k * (_clen - 0.4) / 4
        for sx in (-1, 1):
            _vis(f"/World/Show/convLeg{k}_{int(sx > 0)}", (0.05, 0.05, CONV_TOP - 0.06), (CONV_X + sx * 0.28, yy, (CONV_TOP - 0.06) / 2), steel2)
    _vis("/World/Show/convMotor", (0.22, 0.22, 0.22), (CONV_X - 0.42, CONV_Y1 + 0.3, CONV_TOP - 0.14), mat("ConvBlue", (0.15, 0.35, 0.62), 0.4, 0.3))
    _vis("/World/Show/mast", (0.05, 0.05, 1.3), (0.50, BASE_P[1] + 0.55, BASE_P[2] + 0.65), steel2)
    _vis("/World/Show/cam3d", (0.30, 0.10, 0.08), (0.50, BASE_P[1] + 0.57, BASE_P[2] + 1.32), dark)
    # context props (Isaac Simple_Warehouse) on the dock floor
    for i, (pos, yaw, a) in enumerate((((3.2, -2.0, 0), 90, "SM_PaletteA_01"), ((3.2, -2.0, 0.15), 0, "SM_CardBoxA_01"),
                                         ((3.0, -4.6, 0), 0, "SM_PaletteA_01"), ((3.0, -4.6, 0.15), 15, "SM_CardBoxB_01"),
                                         ((3.4, -3.4, 0), 90, "SM_PaletteA_01"))):
        _xf(f"/World/Show/prop{i}", pos, yaw, ROOT + f"/Isaac/Environments/Simple_Warehouse/Props/{a}.usd")
        _strip(f"/World/Show/prop{i}")
    r = UsdLux.RectLight.Define(st, "/World/Show/DockLight")
    r.CreateWidthAttr(3.0)
    r.CreateHeightAttr(2.0)
    r.CreateIntensityAttr(9000)
    UsdGeom.Xformable(r).AddTranslateOp().Set(Gf.Vec3d(0, -3.0, 4.6))
    for b_ in B:
        _strip(f"/World/Boxes/b{b_['id']}/vis")
    # display-only arm (official Isaac KR210 asset), links posed from Lula FK each frame
    _xf("/World/Show/arm", BASE_P, 90.0, ROOT + "/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd")
    _strip("/World/Show/arm")
    # flange adapter between tool0 and the U1 pad housing (visual)
    vis_cube("/World/EOAT/adapter", (0.16, 0.16, 0.08), (0, 0, -0.16), mat("EoatBody", (0.7, 0.72, 0.75), 0.35, 0.6))
    for i, (cx, cy) in enumerate([(x, y) for x in (-0.12, 0, 0.12) for y in (-0.09, 0.09)]):
        vis_cube(f"/World/EOAT/valve{i}", (0.05, 0.05, 0.04), (cx, cy, -0.14), mat("Valve", (0.15, 0.35, 0.62), 0.4))

    # vacuum status ring around the pad face + status beacon (grey = idle, amber = approach/seek, green = vacuum held)
    def _emat(nm, rgb, e):
        m_ = UsdShade.Material.Define(st, f"/World/Looks/{nm}")
        s_ = UsdShade.Shader.Define(st, f"/World/Looks/{nm}/S")
        s_.CreateIdAttr("UsdPreviewSurface")
        s_.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        s_.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        if e > 0:
            s_.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[c_ * e for c_ in rgb]))
        m_.CreateSurfaceOutput().ConnectToSource(s_.ConnectableAPI(), "surface")
        return m_

    LED_M = {"off": _emat("U1Off", (0.22, 0.22, 0.24), 0.0), "seek": _emat("U1Seek", (1.0, 0.65, 0.05), 1.0),
             "hold": _emat("U1Hold", (0.1, 1.0, 0.3), 1.0)}
    _hx, _hy, _t = PAD[0] / 2 + 0.012, PAD[1] / 2 + 0.012, 0.022
    _ring = [((2 * _hx + _t, _t, 0.05), (0, _hy, 0.0)), ((2 * _hx + _t, _t, 0.05), (0, -_hy, 0.0)),
             ((_t, 2 * _hy - _t, 0.05), (_hx, 0, 0.0)), ((_t, 2 * _hy - _t, 0.05), (-_hx, 0, 0.0)),
             ((0.10, 0.06, 0.10), (0, -(PAD[1] / 2 + 0.03), -0.08))]  # beacon on the housing top (local -y = world up)
    LED_B = []
    for i, (sz_, c_) in enumerate(_ring):
        vis_cube(f"/World/EOAT/led{i}", sz_, c_, LED_M["off"])
        LED_B.append(UsdShade.MaterialBindingAPI(st.GetPrimAtPath(f"/World/EOAT/led{i}/g")))
    scam = UsdGeom.Camera.Define(st, "/World/ShowCam")
    scam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200))
    SHOW_EYE = np.array([float(v) for v in os.environ.get("U1_SHOW_EYE", "-3.4,-6.4,2.6").split(",")])
    SHOW_TGT = np.array([float(v) for v in os.environ.get("U1_SHOW_TGT", "-0.3,-1.6,1.0").split(",")])
    scam.CreateFocalLengthAttr(float(os.environ.get("U1_SHOW_FOCAL", "17")))
    scam.CreateHorizontalApertureAttr(20.955)
    scam.CreateVerticalApertureAttr(20.955 * RES[1] / RES[0])
    _z = SHOW_EYE - SHOW_TGT
    _z /= np.linalg.norm(_z)
    _x = np.cross([0, 0, 1], _z)
    _x /= np.linalg.norm(_x)
    _M = np.eye(4)
    _M[:3, 0], _M[:3, 1], _M[:3, 2], _M[:3, 3] = _x, np.cross(_z, _x), _z, SHOW_EYE
    UsdGeom.Xformable(scam.GetPrim()).AddTransformOp().Set(Gf.Matrix4d(_M.T.tolist()))
    SHOW = dict(base=np.eye(4), q=np.zeros(6), ok=0, fail=0)
    SHOW["base"][:3, :3] = _R.from_euler("z", 90, degrees=True).as_matrix()
    SHOW["base"][:3, 3] = BASE_P


def project(p):
    """World point -> pixel (pinhole, same intrinsics as the USD camera)."""
    pc = np.linalg.inv(CM) @ np.r_[p, 1.0]
    fpx = FOCAL / HA * RES[0]
    return RES[0] / 2 + fpx * pc[0] / -pc[2], RES[1] / 2 - fpx * pc[1] / -pc[2]


for _ in range(10):
    app.update()
SimulationManager.setup_simulation(dt=1.0 / 120)
_surface_gripper.acquire_surface_gripper_interface().set_write_to_usd(True)
grip = GripperView(paths="/World/SG", max_grip_distance=[0.03], coaxial_force_limit=[LIMIT], shear_force_limit=[LIMIT],
                   retry_interval=[0.0])
ids = [b["id"] for b in B]
boxes = RigidPrim([f"/World/Boxes/b{i}" for i in ids], reset_xform_op_properties=True)
eoat = RigidPrim("/World/EOAT", reset_xform_op_properties=True)
R_PAD = Rotation.from_euler("x", -90, degrees=True)  # pad normal (+z local) -> +y world (into the wall)
PARK = np.array([0.0, -1.6, 2.2])


def pose7(p, R=R_PAD):
    q = R.as_quat()
    return np.asarray(p, float)[None], np.array([[q[3], q[0], q[1], q[2]]])


if SHOW:
    PARK = np.array([0.0, -0.95, 1.55])  # arm-reachable park pose (showcase only; picks are unchanged)
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

    SHOW["ks"] = LulaKinematicsSolver(robot_description_path=os.path.join(LULA, "rmpflow", "kuka_kr210_robot_description.yaml"),
                                      urdf_path=os.path.join(LULA, "kr210.urdf"))
    _RE = np.eye(4)
    _RE[:3, :3] = Rotation.from_euler("y", 90, degrees=True).as_matrix()   # tool0 +X = flange normal -> EOAT +Z
    _OFF = np.eye(4)
    _OFF[2, 3] = -0.20                                                      # EOAT origin -> flange face
    SHOW["tool"] = _OFF @ np.linalg.inv(_RE)
    SHOW["q"] = np.array([0.0, -0.6, 0.9, 0.0, 0.6, 0.0])

    def _T(R, p):
        M = np.eye(4)
        M[:3, :3], M[:3, 3] = R, p
        return M

    def pose_arm():
        q_ = eoat.get_world_poses()
        p_, qq = q_[0].numpy()[0], q_[1].numpy()[0]
        Ew = _T(Rotation.from_quat([qq[1], qq[2], qq[3], qq[0]]).as_matrix(), p_)
        tb = np.linalg.inv(SHOW["base"]) @ Ew @ SHOW["tool"]
        x_, y_, z_, w_ = Rotation.from_matrix(tb[:3, :3]).as_quat()
        q, ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=SHOW["q"],
                                                      position_tolerance=2e-3, orientation_tolerance=2e-2)
        if not ok:  # retry from a neutral seed (warm start can trap the solver after a fast move)
            q, ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]),
                                                          warm_start=np.array([0.0, -0.6, 0.9, 0.0, 0.6, 0.0]),
                                                          position_tolerance=3e-3, orientation_tolerance=3e-2)
        if ok:
            q = np.asarray(q, float)
            SHOW["q"] = SHOW["q"] + (q - SHOW["q"] + np.pi) % (2 * np.pi) - np.pi
            SHOW["ok"] += 1
        else:
            SHOW["fail"] += 1
        for ln in ("base_link", "link_1", "Link1", "link_2", "link_3", "link_4", "link_5", "link_6", "tool0"):
            pr = st.GetPrimAtPath(f"/World/Show/arm/{ln}")
            if pr.IsValid():
                pp, RR = SHOW["ks"].compute_forward_kinematics(ln, SHOW["q"])
                x = UsdGeom.Xformable(pr)
                x.ClearXformOpOrder()
                x.AddTransformOp().Set(Gf.Matrix4d(_T(np.asarray(RR), np.asarray(pp)).T.tolist()))

eoat.set_world_poses(*pose7(PARK))
app_utils.play()
app.update()
tl = omni.timeline.get_timeline_interface()
t0 = tl.get_current_time()
frames_dir = args.frames
fcount = [0]
rgb = None
if frames_dir or args.obs or args.datagen or args.vla or SHOW:
    import omni.replicator.core as rep

    rp = rep.create.render_product("/World/ShowCam" if SHOW else "/World/Cam", resolution=RES)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(rp)
    rgb_pip = None
    if SHOW and os.environ.get("U1_PIP"):  # close-up camera that follows the pad (picture-in-picture)
        pcam = UsdGeom.Camera.Define(st, "/World/PadCam")
        pcam.CreateClippingRangeAttr(Gf.Vec2f(0.03, 50))
        pcam.CreateFocalLengthAttr(float(os.environ.get("U1_PIP_FOCAL", "18")))
        pcam.CreateHorizontalApertureAttr(20.955)
        pcam.CreateVerticalApertureAttr(20.955 * 9 / 16)
        UsdGeom.Xformable(pcam.GetPrim()).AddTransformOp()
        rp2 = rep.create.render_product("/World/PadCam", resolution=(960, 540))
        rgb_pip = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_pip.attach(rp2)
        SHOW["pip_s"] = None
    rep.orchestrator.set_capture_on_play(False)


def place_padcam():
    p_ = eoat.get_world_poses()[0].numpy()[0]
    off = np.array([float(v) for v in os.environ.get("U1_PIP_OFF", "0.70,-0.75,0.40").split(",")])
    if p_[1] > -1.2:  # inside / at the container: look from the container centre side, never through its wall
        off[0] = -np.sign(p_[0]) * abs(off[0]) if abs(p_[0]) > 0.25 else abs(off[0])
        off[0] = np.clip(p_[0] + off[0], -1.0, 1.0) - p_[0]
    s_ = np.r_[p_ + off, p_ + np.array([0, 0.10, -0.03])]
    if os.environ.get("U1_PIP_EYE"):  # fixed eye, track the pad only (no wall/arm crossings)
        s_[:3] = [float(v) for v in os.environ["U1_PIP_EYE"].split(",")]
        s_[3:] = p_
    SHOW["pip_s"] = s_ if SHOW["pip_s"] is None else 0.85 * SHOW["pip_s"] + 0.15 * s_  # smooth follow
    eye, tgt = SHOW["pip_s"][:3], SHOW["pip_s"][3:]
    z = eye - tgt
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, np.cross(z, x), z, eye
    UsdGeom.Xformable(st.GetPrimAtPath("/World/PadCam")).GetOrderedXformOps()[0].Set(Gf.Matrix4d(M.T.tolist()))


def snap():
    rep.orchestrator.step(rt_subframes=int(os.environ.get("U1_SUB", "4")), delta_time=0.0, pause_timeline=False)
    return np.ascontiguousarray(rgb.get_data()[:, :, :3])


def servo_to(p, dt=1 / 60):
    q = eoat.get_world_poses()
    cp = q[0].numpy()[0]
    cq = q[1].numpy()[0]
    R = Rotation.from_quat([cq[1], cq[2], cq[3], cq[0]])
    v = (np.asarray(p) - cp) / dt
    R_t = SHOW["R_cmd"] if (SHOW and SHOW.get("R_cmd") is not None) else R_PAD
    w = (R_t * R.inv()).as_rotvec() / dt
    eoat.set_velocities(linear_velocities=v[None], angular_velocities=w[None])


def cur_R():
    cq = eoat.get_world_poses()[1].numpy()[0]
    return Rotation.from_quat([cq[1], cq[2], cq[3], cq[0]])


# showcase only: place on the side conveyor with the pad turned to face -x (arm reaches out sideways, no folding)
R_PLACE = Rotation.from_euler("z", 90, degrees=True) * R_PAD


FAST = not (frames_dir or args.obs or args.datagen or args.vla or SHOW)
if SHOW:
    frames_dir = args.show


def step(n=1, target=None):
    for _ in range(n):
        if target is not None:
            servo_to(target)
        if FAST:  # physics only (no Hydra render) when no images are needed: 2 x 1/120 s = one 1/60 s update
            SimulationManager.step()
            SimulationManager.step()
        else:
            app.update()
        every = args.show_every if SHOW else 2
        if frames_dir and fcount[0] % every == 0:
            import imageio.v2 as imageio

            if SHOW:
                pose_arm()
                _st = "hold" if grip.get_gripped_objects()[0] else SHOW.get("phase", "off")
                if _st != SHOW.get("led"):
                    for b_ in LED_B:
                        b_.Bind(LED_M[_st])
                    SHOW["led"] = _st
                if rgb_pip is not None:
                    place_padcam()
            imageio.imwrite(os.path.join(frames_dir, f"{fcount[0] // every:05d}.png"), snap())
            if SHOW and rgb_pip is not None:
                os.makedirs(os.path.join(frames_dir, "pip"), exist_ok=True)
                imageio.imwrite(os.path.join(frames_dir, "pip", f"{fcount[0] // every:05d}.png"),
                                np.ascontiguousarray(rgb_pip.get_data()[:, :, :3]))
        fcount[0] += 1


def move(a, b, secs, R_end=None):
    n = max(2, int(secs * 60))
    sl_ = None
    if SHOW:  # showcase: orientation follows the same smooth profile (default: back to the pick orientation)
        R0 = SHOW["R_cmd"] if SHOW.get("R_cmd") is not None else R_PAD
        R1 = R_end if R_end is not None else R_PAD
        sl_ = Slerp([0.0, 1.0], Rotation.concatenate([R0, R1]))
    for i in range(1, n + 1):
        u = i / n
        u = u * u * (3 - 2 * u)
        if sl_ is not None:
            SHOW["R_cmd"] = sl_(u)
        step(1, a + (b - a) * u)


if frames_dir:
    os.makedirs(frames_dir, exist_ok=True)
step(150, PARK)  # settle 2.5 s
p0 = boxes.get_world_poses()[0].numpy().copy()
settle_moved = [ids[k] for k in range(len(ids)) if np.linalg.norm(p0[k] - np.array([B[k]["x"], B[k]["d"] / 2, B[k]["z"]])) > 0.03]
print(f"[u1] seed {args.seed}: {len(ids)} boxes, settle moved {settle_moved}", flush=True)

if args.obs:
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont

    for _ in range(3):
        snap()
    img = snap()
    imageio.imwrite(os.path.join(HERE, "obs" + SFX, f"wall_{args.seed}_raw.png"), img)
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    fnt = ImageFont.truetype(r"C:\Windows\Fonts\arialbd.ttf", 22)
    table = []
    q0 = boxes.get_world_poses()[1].numpy()
    for k, b in enumerate(B):
        c = p0[k]
        R = Rotation.from_quat([q0[k][1], q0[k][2], q0[k][3], q0[k][0]])
        # front-face centre and top/bottom from the settled pose
        corners = np.array([[sx * b["w"] / 2, sy * b["d"] / 2, sz * b["h"] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        wc = R.apply(corners) + c
        front = c + R.apply([0, -b["d"] / 2, 0])
        u, v = project(front)
        table.append(dict(id=b["id"], x=round(float(front[0]), 3), z=round(float(front[2]), 3), w=round(b["w"], 3),
                          h=round(b["h"], 3), top=round(float(wc[:, 2].max()), 3), bottom=round(float(wc[:, 2].min()), 3),
                          lean_deg=round(float(R.as_euler("xyz", degrees=True)[1]), 1), px=[round(u), round(v)]))
        tw = dr.textlength(str(b["id"]), font=fnt)
        dr.rounded_rectangle((u - tw / 2 - 5, v - 14, u + tw / 2 + 5, v + 14), 6, fill=(255, 255, 255), outline=(20, 20, 20), width=2)
        dr.text((u - tw / 2, v - 13), str(b["id"]), font=fnt, fill=(10, 10, 10))
    im.save(os.path.join(HERE, "obs" + SFX, f"wall_{args.seed}.png"))
    json.dump(dict(seed=args.seed, boxes=table, settle_moved=settle_moved), open(os.path.join(HERE, "obs" + SFX, f"wall_{args.seed}_table.json"), "w"), indent=1)
    print("[u1] obs saved", flush=True)


gone = set()
LOG = []


def front_of(k):
    P = boxes.get_world_poses()[0].numpy()
    Q = boxes.get_world_poses()[1].numpy()
    R = Rotation.from_quat([Q[k][1], Q[k][2], Q[k][3], Q[k][0]])
    return P[k], R, P[k] + R.apply([0, -B[k]["d"] / 2, 0])


def in_wall(k):
    P = boxes.get_world_poses()[0].numpy()
    return ids[k] not in gone and P[k][2] > -0.5 and np.linalg.norm(P[k] - p0[k]) <= 0.25


def live_table():
    """Detection table of the boxes still standing in the wall (same fields as the obs table)."""
    out = []
    for k, b in enumerate(B):
        if not in_wall(k):
            continue
        c, R, front = front_of(k)
        corners = np.array([[sx * b["w"] / 2, sy * b["d"] / 2, sz * b["h"] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        wc = R.apply(corners) + c
        u, v = project(front)
        out.append(dict(id=b["id"], x=float(front[0]), z=float(front[2]), w=b["w"], h=b["h"], top=float(wc[:, 2].max()),
                        bottom=float(wc[:, 2].min()), px=[float(u), float(v)]))
    return out


def pick(bid, name):
    if bid not in ids or bid in gone:
        LOG.append(dict(id=bid, result="skip_invalid"))
        return
    k = ids.index(bid)
    if not in_wall(k):
        LOG.append(dict(id=bid, result="skip_displaced"))
        return
    standing = {ids[j] for j in range(len(ids)) if in_wall(j)}  # only boxes still in the wall can newly fall
    _, _, front = front_of(k)
    pre = front + np.array([0, -0.45, 0])
    front[2] = max(front[2], PAD[1] / 2 + 0.012)  # keep the pad edge off the floor; the ray still hits the face
    face = front + np.array([0, -PAD[2] - 0.004, 0])
    cur = eoat.get_world_poses()[0].numpy()[0]
    move(cur, pre, 0.6)
    if SHOW:
        SHOW["phase"] = "seek"
    move(pre, face, 0.4)
    grip.apply_gripper_action([0.5])
    step(8, face)
    if SHOW:
        SHOW["phase"] = "off"
    held = bool(grip.get_gripped_objects()[0])
    gm = None
    if held and GRIP_P and B[k].get("kind") == "bag":
        u_ = GRNG.random()
        if u_ < GRIP_P[0]:
            gm = "model_no_grip"
            grip.apply_gripper_action([-0.5])
            step(4, face)
            held = False
        elif u_ < GRIP_P[0] + GRIP_P[1]:
            gm = "model_drop"
    before = boxes.get_world_poses()[0].numpy().copy()
    if gm == "model_drop":
        mid = face + np.array([0, -0.35, 0.01])
        move(face, mid, 0.45)
        grip.apply_gripper_action([-0.5])
        step(2, mid)
        move(mid, face + np.array([0, -0.70, 0.02]), 0.45)
    else:
        move(face, face + np.array([0, -0.70, 0.02]), 0.9)
    held_after = bool(grip.get_gripped_objects()[0])
    hold_p = face + np.array([0, -0.70, 0.02])
    if SHOW and held_after:  # showcase: place the case on the take-away conveyor, then let the belt carry it out
        b_ = B[k]
        # pad turned to face -x: the gripped (front) face now looks +x, so the pad sits on the +x side of the box
        tgt = np.array([CONV_X + b_["d"] / 2 + PAD[2] + 0.004, CONV_Y, CONV_TOP + b_["h"] / 2 + 0.03])
        via = np.array([tgt[0] + 0.35, hold_p[1] - 0.15, max(hold_p[2], tgt[2]) + 0.20])
        move(hold_p, via, 0.7, R_PLACE)
        move(via, tgt, 0.6, R_PLACE)
        grip.apply_gripper_action([-0.5])
        step(4, tgt)
        hold_p = tgt + np.array([0.30, 0, 0.20])
        for _ in range(45):
            boxes.set_velocities(linear_velocities=np.array([[0.0, -1.1, 0.0]]), angular_velocities=np.zeros((1, 3)), indices=[k])
            step(1, hold_p)
    grip.apply_gripper_action([-0.5])
    step(2, hold_p)
    if held_after:
        far = np.array([[5 + len(gone) * 0.9, -2.5, 0.4]])
        boxes.set_world_poses(positions=far, indices=[k])
        boxes.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)), indices=[k])
        gone.add(bid)
    step(40, hold_p)  # let the wall react
    Pa = boxes.get_world_poses()[0].numpy()
    moved = [ids[j] for j in range(len(ids)) if ids[j] in standing and ids[j] not in gone and ids[j] != bid and np.linalg.norm(Pa[j] - before[j]) > 0.05]
    fell = [j for j in moved if Pa[ids.index(j)][2] < before[ids.index(j)][2] - 0.10]
    LOG.append(dict(id=bid, result="picked" if held_after else ("grip_lost" if held else "no_grip"), disturbed=moved, fell=fell, kind=B[k].get("kind", "box"), grip_model=gm))
    print(f"[u1] {name} pick {bid}: {LOG[-1]['result']} disturbed={moved} fell={fell}", flush=True)


def finish(name, extra=None):
    P = boxes.get_world_poses()[0].numpy()
    left = [ids[j] for j in range(len(ids)) if ids[j] not in gone]
    on_floor = [i for i in left if P[ids.index(i)][2] < 0.3 and np.linalg.norm(P[ids.index(i)] - p0[ids.index(i)]) > 0.1]
    res = dict(seed=args.seed, method=name, n_boxes=len(ids), picked=len(gone),
               collapse_events=sum(1 for e in LOG if e.get("fell")), boxes_fell=len({b for e in LOG for b in e.get("fell", [])}),
               left=left, on_floor=on_floor, log=LOG)
    res.update(extra or {})
    json.dump(res, open(os.path.join(HERE, "results" + SFX, f"{name}_{args.seed}.json"), "w"), indent=1)
    print(f"[u1] RESULT {name} seed {args.seed}: picked {len(gone)}/{len(ids)} collapse_events {res['collapse_events']} "
          f"fell {res['boxes_fell']}", flush=True)


if args.plan:
    plan = json.load(open(args.plan, encoding="utf-8"))
    name = plan.get("method", os.path.splitext(os.path.basename(args.plan))[0])
    for n_, bid in enumerate(plan["sequence"]):
        if args.max_picks and n_ >= args.max_picks:
            break
        pick(bid, name)
        if SC and LOG and LOG[-1].get("id") == bid and LOG[-1]["result"] == "no_grip":
            pick(bid, name)  # scenario runs: one re-grip attempt, like a real cell
    if SHOW:
        print(f"[u1] show IK ok {SHOW['ok']} fail {SHOW['fail']}", flush=True)
    else:
        finish(name, dict(decision_latency_s=plan.get("decision_latency_s")))

if args.datagen:  # VLA training data: raw image -> rule expert's next box (pixel target), executed physically
    import sys as _sys

    import imageio.v2 as imageio

    _sys.path.insert(0, HERE)
    from u1_rule import plan as rule_plan

    od = os.path.join(args.datagen, f"ep_{args.seed}")
    os.makedirs(od, exist_ok=True)
    acts, states, skip = [], [], set()
    for t in range(70):
        tab = live_table()
        if not tab:
            break
        cand = [b for b in rule_plan(tab) if b not in skip]
        if not cand:
            break
        nxt = cand[0]
        row = next(r for r in tab if r["id"] == nxt)
        for _ in range(2):
            snap()
        img = snap()
        imageio.imwrite(os.path.join(od, f"{t:04d}.jpg"), img[::2, ::2], quality=92)  # 640x360
        acts.append([row["px"][0] / RES[0], row["px"][1] / RES[1]])
        states.append([len(tab) / 45.0, t / 45.0])
        pick(nxt, "datagen")
        if LOG and LOG[-1]["id"] == nxt and LOG[-1]["result"] != "picked":
            skip.add(nxt)  # unreachable / lost grip: the expert moves on instead of retrying forever
    np.savez(os.path.join(od, "data.npz"), action=np.array(acts, np.float32), state=np.array(states, np.float32))
    print(f"[u1] DATAGEN seed {args.seed}: {len(acts)} samples", flush=True)

if args.vla:  # closed loop with an external policy process (file handshake)
    import time as _time

    import imageio.v2 as imageio

    os.makedirs(args.vla, exist_ok=True)
    name = os.environ.get("U1_VLA_NAME", "vla")
    invalid = 0
    lat = []
    for t in range(70):
        tab = live_table()
        if not tab or invalid >= 3:
            break
        for _ in range(2):
            snap()
        img = snap()
        obs = os.path.join(args.vla, f"obs_{args.seed}_{t:03d}.jpg")
        imageio.imwrite(obs + ".tmp.jpg", img[::2, ::2], quality=92)
        json.dump(dict(state=[len(tab) / 45.0, t / 45.0]), open(obs + ".json", "w"))
        os.replace(obs + ".tmp.jpg", obs)
        actf = obs.replace("obs_", "act_").replace(".jpg", ".json")
        t0_ = _time.perf_counter()
        while not os.path.exists(actf):
            _time.sleep(0.05)
        _time.sleep(0.05)
        a = json.load(open(actf))
        lat.append(_time.perf_counter() - t0_)
        u, v = a["u"] * RES[0], a["v"] * RES[1]
        cand = min(tab, key=lambda r: (r["px"][0] - u) ** 2 + (r["px"][1] - v) ** 2)
        dist = ((cand["px"][0] - u) ** 2 + (cand["px"][1] - v) ** 2) ** 0.5
        if dist > 120:
            invalid += 1
            LOG.append(dict(id=None, result="no_box_at_target", px=[u, v]))
            continue
        pick(cand["id"], name)
    finish(name, dict(decision_latency_s=float(np.mean(lat)) if lat else None))
app_utils.stop()
app.close()

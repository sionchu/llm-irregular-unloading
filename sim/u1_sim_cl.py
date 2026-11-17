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
ap.add_argument("--closed", default="", help="closed-loop planner name (u1_planners.PLANNERS)")
ap.add_argument("--look", type=int, default=0, help="physics lookahead over the top-K candidates")
ap.add_argument("--agent", default="", help="IK agent: opus | ikrule")
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
if args.show or args.agent:
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

CATCH = os.environ.get("U1_CATCH", "") == "1"
CATCH_Y0, CATCH_Y1, CATCH_HW, CATCH_TOP = -0.04, -1.0, 0.72, 0.10   # belt from the wall face to the base front, half width, top z
CONVEYED = {}
if CATCH:
    static_box("/World/Catch/belt", (2 * CATCH_HW, CATCH_Y0 - CATCH_Y1, 0.04), (0, (CATCH_Y0 + CATCH_Y1) / 2, CATCH_TOP - 0.02),
               mat("CatchBelt", (0.07, 0.07, 0.08), 0.85))
    for sx_ in (-1, 1):
        vis_cube(f"/World/Catch/frame{int(sx_ > 0)}", (0.05, CATCH_Y0 - CATCH_Y1, 0.10), (sx_ * (CATCH_HW + 0.025), (CATCH_Y0 + CATCH_Y1) / 2, CATCH_TOP - 0.05),
                 mat("CatchFrame", (0.55, 0.57, 0.60), 0.4, 0.7))
    for i_ in range(9):
        vis_cube(f"/World/Catch/roller{i_}", (2 * CATCH_HW, 0.03, 0.03), (0, CATCH_Y0 - 0.05 - i_ * (CATCH_Y0 - CATCH_Y1 - 0.1) / 8, CATCH_TOP - 0.055),
                 mat("CatchRoller", (0.35, 0.36, 0.38), 0.3, 0.8))
    vis_cube("/World/Catch/lip", (2 * CATCH_HW, 0.03, 0.03), (0, CATCH_Y0 - 0.015, CATCH_TOP + 0.005), mat("SafetyYellowC", (0.95, 0.75, 0.05), 0.5))

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
    if args.show and kind == "box":  # textured cardboard (Isaac Simple_Warehouse box props, scaled to the physics box; visual only)
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
        vis_cube(p + "/vis", (b["w"], b["d"], b["h"]), (0, 0, 0), mat(f"Bag{b['id']}", (0.90 * tone, 0.91 * tone, 0.93 * tone), 0.22))
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
    st.GetPrimAtPath("/World/Looks/Container/S").GetAttribute("inputs:diffuseColor").Set(Gf.Vec3f(0.80, 0.81, 0.83))
    st.GetPrimAtPath("/World/Looks/Container/S").GetAttribute("inputs:metallic").Set(0.6)
    st.GetPrimAtPath("/World/Looks/Container/S").GetAttribute("inputs:roughness").Set(0.45)
    st.GetPrimAtPath("/World/Looks/Backstop/S").GetAttribute("inputs:diffuseColor").Set(Gf.Vec3f(0.62, 0.63, 0.65))
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
        _vis(f"/World/Show/convGuard{int(sx > 0)}", (0.04, _clen, 0.07), (CONV_X + sx * 0.33, _cy, CONV_TOP - 0.015), mat("ConvBlue", (0.15, 0.35, 0.62), 0.4, 0.3))
    for k in range(5):
        yy = CONV_Y0 - 0.2 - k * (_clen - 0.4) / 4
        for sx in (-1, 1):
            _vis(f"/World/Show/convLeg{k}_{int(sx > 0)}", (0.05, 0.05, CONV_TOP - 0.06), (CONV_X + sx * 0.28, yy, (CONV_TOP - 0.06) / 2), steel2)
    _vis("/World/Show/convMotor", (0.22, 0.22, 0.22), (CONV_X - 0.42, CONV_Y1 + 0.3, CONV_TOP - 0.14), mat("ConvBlue", (0.15, 0.35, 0.62), 0.4, 0.3))
    if os.environ.get("U1_MAST", "0") == "1":  # camera mast sat in the arm's sweep (visual only) -> off by default
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

    def _arm_pts(q):
        # points along the arm (base frame -> world), 8 samples per link segment
        ps = [np.asarray(SHOW["ks"].compute_forward_kinematics(ln, q)[0]) for ln in ("link_2", "link_3", "link_4", "link_5", "link_6", "tool0")]
        out = []
        for a_, b_ in zip(ps[:-1], ps[1:]):
            for u_ in np.linspace(0, 1, 8):
                out.append((SHOW["base"] @ np.r_[a_ + (b_ - a_) * u_, 1.0])[:3])
        return np.array(out)

    def _arm_in_conv(q, r=0.12):
        # arm centre line within r of the belt volume (belt top 0.80, x CONV_X +-0.35, y from CONV_Y0 back to CONV_Y1)
        p_ = _arm_pts(q)
        inx = np.abs(p_[:, 0] - CONV_X) < 0.35 + r
        iny = (p_[:, 1] < CONV_Y0 + r) & (p_[:, 1] > CONV_Y1 - r)
        inz = p_[:, 2] < CONV_TOP + r
        return bool(np.any(inx & iny & inz))

    def _arm_path_ok(poses):
        # poses: [(p, Rotation)] for the EOAT; True if every pose solves with tight tolerances and no link enters the belt
        qw = SHOW["q"].copy()
        for p_, R_ in poses:
            Ew = _T(R_.as_matrix(), p_)
            tb = np.linalg.inv(SHOW["base"]) @ Ew @ SHOW["tool"]
            x_, y_, z_, w_ = Rotation.from_matrix(tb[:3, :3]).as_quat()
            q, ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=qw,
                                                          position_tolerance=2e-3, orientation_tolerance=2e-2)
            if not ok:
                return False
            q = np.asarray(q, float)
            qw = qw + (q - qw + np.pi) % (2 * np.pi) - np.pi
            if _arm_in_conv(qw):
                return False
        return True

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
        if not ok:
            _best = None
            for _sd in SHOW.setdefault("seeds", [np.array(v) for v in (
                    [0.0, -0.6, 0.9, 0.0, 0.6, 0.0], [0.0, -1.0, 1.2, 0.0, -0.2, 0.0], [0.0, -0.3, 0.3, 0.0, 1.2, 0.0],
                    [0.8, -0.6, 0.9, 0.0, 0.6, 0.0], [-0.8, -0.6, 0.9, 0.0, 0.6, 0.0], [1.6, -0.8, 1.0, 0.0, 0.5, 0.0],
                    [-1.6, -0.8, 1.0, 0.0, 0.5, 0.0], [0.0, -0.9, 1.6, 0.0, -1.0, 0.0], [0.0, -0.6, 0.9, 1.5, 0.6, -1.5],
                    [0.0, -0.6, 0.9, -1.5, 0.6, 1.5])]):
                _q, _ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=_sd,
                                                               position_tolerance=3e-3, orientation_tolerance=3e-2)
                if _ok:
                    _q = np.asarray(_q, float)
                    _d = float(np.linalg.norm((_q - SHOW["q"] + np.pi) % (2 * np.pi) - np.pi))
                    if _best is None or _d < _best[0]:
                        _best = (_d, _q)
            if _best is not None:
                q, ok = _best[1], True
        if ok and _arm_in_conv(np.asarray(q, float)):  # this branch puts a link into the conveyor: look for another branch
            _alt = None
            for _sd in SHOW.get("seeds") or [np.array(v) for v in ([0.0, -0.6, 0.9, 0.0, 0.6, 0.0], [0.0, -1.0, 1.2, 0.0, -0.2, 0.0], [0.0, -0.3, 0.3, 0.0, 1.2, 0.0], [0.8, -0.6, 0.9, 0.0, 0.6, 0.0], [-0.8, -0.6, 0.9, 0.0, 0.6, 0.0], [0.0, -0.9, 1.6, 0.0, -1.0, 0.0], [0.0, -0.6, 0.9, 1.5, 0.6, -1.5], [0.0, -0.6, 0.9, -1.5, 0.6, 1.5])]:
                _q, _ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=_sd,
                                                               position_tolerance=3e-3, orientation_tolerance=3e-2)
                if _ok and not _arm_in_conv(np.asarray(_q, float)):
                    _q = np.asarray(_q, float)
                    _d = float(np.linalg.norm((_q - SHOW["q"] + np.pi) % (2 * np.pi) - np.pi))
                    if _alt is None or _d < _alt[0]:
                        _alt = (_d, _q)
            if _alt is not None:
                q = _alt[1]
        lvl = 0 if ok else 1
        if not ok:  # keep the arm on the pad: accept a small orientation error rather than freezing the arm in place
            q, ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=SHOW["q"],
                                                          position_tolerance=1e-2, orientation_tolerance=0.35)
            lvl = 1 if ok else 2
        for _ot in (1.0, 3.2):  # last resort: reach the pad position, accept a wrist orientation error (arm never left behind)
            if ok:
                break
            q, ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=SHOW["q"],
                                                          position_tolerance=1e-2, orientation_tolerance=_ot)
            lvl = 3 if ok else 2
        if ok:
            q = np.asarray(q, float)
            SHOW["q"] = SHOW["q"] + (q - SHOW["q"] + np.pi) % (2 * np.pi) - np.pi
            SHOW["ok"] += 1
        else:
            SHOW["fail"] += 1
        pt_, _ = SHOW["ks"].compute_forward_kinematics("tool0", SHOW["q"])
        err_ = float(np.linalg.norm(np.asarray(pt_) - tb[:3, 3])) * 1000
        SHOW.setdefault("diag", []).append([fcount[0] // max(1, args.show_every), lvl, round(err_, 1), int(_arm_in_conv(SHOW["q"]))])
        if len(SHOW["diag"]) % 20 == 0 and frames_dir:
            json.dump(SHOW["diag"], open(os.path.join(frames_dir, "armdiag.json"), "w"))
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


R_GRASP = None


def servo_to(p, dt=1 / 60):
    q = eoat.get_world_poses()
    cp = q[0].numpy()[0]
    cq = q[1].numpy()[0]
    R = Rotation.from_quat([cq[1], cq[2], cq[3], cq[0]])
    v = (np.asarray(p) - cp) / dt
    R_t = SHOW["R_cmd"] if (SHOW and SHOW.get("R_cmd") is not None) else (R_GRASP if R_GRASP is not None else R_PAD)
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
if SHOW and os.environ.get("U1_SHOW_FROM"):
    FAST, frames_dir = True, None


def catch_update():
    if "gone" not in globals():  # wall still settling: belt idle
        return
    P = boxes.get_world_poses()[0].numpy()
    V = boxes.get_velocities()[0].numpy()
    on = [k for k in range(len(ids)) if ids[k] not in gone and ids[k] not in CONVEYED and CATCH_Y1 < P[k][1] < CATCH_Y0
          and abs(P[k][0]) < CATCH_HW and P[k][2] < CATCH_TOP + 0.45]
    if not on:
        return
    out = [k for k in on if P[k][1] < CATCH_Y1 + 0.25]
    mv = [k for k in on if k not in out]
    if mv:
        boxes.set_velocities(linear_velocities=np.array([[0.0, -0.8, min(0.0, V[k][2])] for k in mv]),
                             angular_velocities=np.zeros((len(mv), 3)), indices=mv)
    for k in out:
        CONVEYED[ids[k]] = round(float(p0[k][2] - CATCH_TOP), 2)  # drop proxy: start height of the item centre above the belt
        boxes.set_world_poses(positions=np.array([[5 + len(CONVEYED) * 0.9, -1.4, 0.4]]), indices=[k])
        boxes.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)), indices=[k])


def step(n=1, target=None):
    for _ in range(n):
        if target is not None:
            servo_to(target)
        if CATCH:
            catch_update()
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


def drive_base(y, secs=1.0):
    if not (SHOW and frames_dir) or abs(SHOW["base"][1, 3] - y) < 1e-3:
        return show_base(y)
    pose_arm()
    q = SHOW["q"].copy()
    pt, RR = SHOW["ks"].compute_forward_kinematics("tool0", q)
    Tl = _T(np.asarray(RR), np.asarray(pt)) @ np.linalg.inv(SHOW["tool"])
    y0 = SHOW["base"][1, 3]
    n = max(2, int(secs * 60))
    for i in range(1, n + 1):
        uu = i / n
        uu = uu * uu * (3 - 2 * uu)
        show_base(y0 + (y - y0) * uu)
        T_ = SHOW["base"] @ Tl
        SHOW["R_cmd"] = Rotation.from_matrix(T_[:3, :3])
        step(1, T_[:3, 3])


def move_q(b, R_end, secs):
    """Showcase: joint-space (PTP) move of the display arm to pad pose (b, R_end); the pad follows the arm's FK.
    Avoids Cartesian moves through wrist flips, where the arm cannot follow the pad."""
    cur = eoat.get_world_poses()[0].numpy()[0]
    if not (SHOW and frames_dir and os.environ.get("U1_PTP") == "1"):
        return move(cur, b, secs, R_end)
    pose_arm()
    Ew = _T(R_end.as_matrix(), np.asarray(b, float))
    tb = np.linalg.inv(SHOW["base"]) @ Ew @ SHOW["tool"]
    x_, y_, z_, w_ = Rotation.from_matrix(tb[:3, :3]).as_quat()
    best = None
    for sd in [SHOW["q"]] + [np.array(v) for v in [[0.0, -0.6, 0.9, 0.0, 0.6, 0.0], [0.0, -1.0, 1.2, 0.0, -0.2, 0.0], [0.0, -0.3, 0.3, 0.0, 1.2, 0.0],
              [0.8, -0.6, 0.9, 0.0, 0.6, 0.0], [-0.8, -0.6, 0.9, 0.0, 0.6, 0.0], [1.6, -0.8, 1.0, 0.0, 0.5, 0.0],
              [-1.6, -0.8, 1.0, 0.0, 0.5, 0.0], [0.0, -0.9, 1.6, 0.0, -1.0, 0.0], [0.0, -0.6, 0.9, 1.5, 0.6, -1.5],
              [0.0, -0.6, 0.9, -1.5, 0.6, 1.5]]]:
        q, ok = SHOW["ks"].compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=sd,
                                                      position_tolerance=2e-3, orientation_tolerance=2e-2)
        if ok:
            q = np.asarray(q, float)
            dq = (q - SHOW["q"] + np.pi) % (2 * np.pi) - np.pi
            if best is None or np.linalg.norm(dq) < np.linalg.norm(best):
                best = dq
    if best is None:
        print("[u1] move_q: no IK, cartesian fallback", flush=True)
        return move(cur, b, secs, R_end)
    q0 = SHOW["q"].copy()
    Tinv = np.linalg.inv(SHOW["tool"])
    n = max(2, int(secs * 60))
    for i in range(1, n + 1):
        uu = i / n
        uu = uu * uu * (3 - 2 * uu)
        pt, RR = SHOW["ks"].compute_forward_kinematics("tool0", q0 + best * uu)
        T_ = SHOW["base"] @ _T(np.asarray(RR), np.asarray(pt)) @ Tinv
        SHOW["R_cmd"] = Rotation.from_matrix(T_[:3, :3])
        step(1, T_[:3, 3])
    SHOW["R_cmd"] = R_end


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


CLOSED = False


def in_wall(k):
    P = boxes.get_world_poses()[0].numpy()
    if CLOSED:  # anything still inside the container and reachable from the door counts (re-perceived)
        return ids[k] not in gone and ids[k] not in CONVEYED and P[k][2] > -0.2 and P[k][1] > -0.75 and not (CATCH and P[k][1] < CATCH_Y0 and P[k][2] < 0.8)
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
    front = pick_point(k) if CLOSED else front_of(k)[2]
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
    if CATCH:
        res.update(catch=True, conveyed=dict(CONVEYED), n_conveyed=len(CONVEYED),
                   unloaded_total=len(gone) + len(CONVEYED), left=[i for i in left if i not in CONVEYED])
    json.dump(res, open(os.path.join(HERE, "results" + SFX, f"{name}_{args.seed}.json"), "w"), indent=1)
    print(f"[u1] RESULT {name} seed {args.seed}: picked {len(gone)}/{len(ids)} collapse_events {res['collapse_events']} "
          f"fell {res['boxes_fell']}", flush=True)



# ---------------- closed loop (u1_sim_cl.py only): re-perceive after every pick, re-plan, optional physics lookahead -----
def _corners(k):
    b = B[k]
    c, R, _ = front_of(k)
    cn = np.array([[sx * b["w"] / 2, sy * b["d"] / 2, sz * b["h"] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return c, R, R.apply(cn) + c


def pick_point(k):
    """Centre of the item face that looks most toward the robot (-y), whatever the item's pose (r2: r1 used an
    edge band of hull corners, which put the pad on an edge of slightly yawed boxes -> no_grip)."""
    b = B[k]
    c, R, _ = front_of(k)
    half = np.array([b["w"], b["d"], b["h"]]) / 2
    n, hh = max(((s_ * np.eye(3)[i], half[i]) for i in range(3) for s_ in (-1, 1)), key=lambda nh: -R.apply(nh[0])[1])
    return c + R.apply(n * hh)


def live_table_cl():
    out = []
    for k, b in enumerate(B):
        if not in_wall(k):
            continue
        c, R, wc = _corners(k)
        pp = pick_point(k)
        tilt = float(np.degrees(np.arccos(np.clip(R.apply([0, 0, 1])[2], -1, 1))))
        out.append(dict(id=b["id"], kind=b.get("kind", "box"), x=float(c[0]), z=float(pp[2]), w=float(np.ptp(wc[:, 0])),
                        h=float(np.ptp(wc[:, 2])), top=float(wc[:, 2].max()), bottom=float(wc[:, 2].min()), tilt_deg=tilt,
                        displaced=bool(np.linalg.norm(c - p0[k]) > 0.25)))
    return out


def _snapshot():
    P = boxes.get_world_poses()
    E = eoat.get_world_poses()
    return P[0].numpy().copy(), P[1].numpy().copy(), E[0].numpy().copy(), E[1].numpy().copy(), set(gone), len(LOG)


def _restore(s):
    global GRNG
    grip.apply_gripper_action([-0.5])
    boxes.set_world_poses(positions=s[0], orientations=s[1])
    boxes.set_velocities(linear_velocities=np.zeros((len(ids), 3)), angular_velocities=np.zeros((len(ids), 3)))
    eoat.set_world_poses(positions=s[2], orientations=s[3])
    eoat.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))
    gone.clear()
    gone.update(s[4])
    del LOG[s[5]:]
    step(2, s[2][0])


def trial_cost(bid):
    """Physics lookahead on the CURRENT sim state (ground-truth twin = optimistic upper bound for a real digital twin).
    Bag grip outcome is drawn from a separate RNG so the trial cannot see the real draw."""
    global GRNG
    s = _snapshot()
    real_rng, GRNG = GRNG, np.random.default_rng(TRIAL_RNG.integers(1 << 30))
    pick(bid, "trial")
    e = LOG[-1]
    cost = 2 * len(e.get("fell", [])) + len(e.get("disturbed", [])) + (0 if e["result"] == "picked" else 3)
    GRNG = real_rng
    _restore(s)
    return cost


if args.closed:
    import collections as _co
    import sys as _sys
    import time as _time

    _sys.path.insert(0, HERE)
    from u1_planners import PLANNERS, STAB_REV

    CLOSED = True
    planner = PLANNERS[args.closed]
    K = args.look
    TRIAL_RNG = np.random.default_rng(args.seed * 31 + 7)
    name = f"cl_{args.closed}" + (f"_look{K}" if K else "")
    fails = _co.Counter()
    dts, n_trials = [], 0
    for t in range(160):
        tab = live_table_cl()
        t0_ = _time.perf_counter()
        cand = [i for i in planner(tab) if fails[i] < 2]
        if not cand:
            break
        bid = cand[0]
        if K and len(cand) > 1:
            costs = [(trial_cost(c_), r_) for r_, c_ in enumerate(cand[:K])]
            n_trials += len(costs)
            bid = cand[min(costs)[1]]
        dts.append(_time.perf_counter() - t0_)
        pick(bid, name)
        if LOG[-1]["result"] != "picked":
            fails[bid] += 1
    finish(name, dict(planner=args.closed, stab_rev=STAB_REV if args.closed == "stab" else None, look_k=K, trials=n_trials,
                      decision_latency_s=float(np.mean(dts)) if dts else None, grip_model_env=os.environ.get("U1_GRIP", "")))


# ---------------- IK agent (u1_sim_cl.py --agent opus|ikrule) ----------------------------------------------------------
# Same closed loop as E1 (re-perceive after every pick), but each pick is a GRASP = item + face (front | top) + offset on
# that face + mobile-base position. A grasp is executable only if KR210 IK (Lula, official Isaac KR210 config) solves the
# stage/pre/contact/retreat poses and the pad sweep does not hit other items or the container. 'opus' = Claude Opus agent
# that calls these checks as tools (MCP, file handshake with this process); 'ikrule' = same tools, fixed code policy.
AG_TRACE = []
HANDED = set()
ROOF_Z, WALL_X = 2.69, 1.15
BASE_YS = (-2.8, -2.3, -1.8)


def _pad_R(n):
    z = -np.asarray(n, float) / np.linalg.norm(n)
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = ref - z * (ref @ z)
    x /= np.linalg.norm(x)
    return Rotation.from_matrix(np.column_stack([x, np.cross(z, x), z]))


def _face_frame(k, face):
    b = B[k]
    c, R, _ = front_of(k)
    half = np.array([b["w"], b["d"], b["h"]]) / 2
    key = (lambda nh: -R.apply(nh[0])[1]) if face == "front" else (lambda nh: R.apply(nh[0])[2])
    n_loc, hh = max(((s_ * np.eye(3)[i], half[i]) for i in range(3) for s_ in (-1, 1)), key=key)
    i_n = int(np.argmax(np.abs(n_loc)))
    o = [i for i in range(3) if i != i_n]
    n = R.apply(n_loc)
    t1, t2 = R.apply(np.eye(3)[o[0]]), R.apply(np.eye(3)[o[1]])
    if abs(t1[2]) > abs(t2[2]):  # t1 = the more horizontal tangent (u = left/right), t2 = the other (v)
        t1, t2, o = t2, t1, o[::-1]
    if t1[0] < 0:
        t1 = -t1
    if t2[2] < 0 or (abs(t2[2]) < 0.3 and t2[1] < 0):
        t2 = -t2
    return c + n * hh, n, t1, t2, (half[o[0]], half[o[1]])


def _obb(k):
    b = B[k]
    c, R, _ = front_of(k)
    return c, R, np.array([b["w"], b["d"], b["h"]]) / 2


_PADPTS = np.array([[x, y, z] for x in np.linspace(-PAD[0] / 2, PAD[0] / 2, 4) for y in np.linspace(-PAD[1] / 2, PAD[1] / 2, 4)
                    for z in (-0.12, PAD[2])])


def _sweep_hits(path, Rg, skip_k):
    """Pad+housing volume sampled along the path vs other items' boxes (+1 cm) and the container walls/roof."""
    pts = []
    for a, b_ in zip(path[:-1], path[1:]):
        for u in np.linspace(0, 1, 10):
            pts.append(Rg.apply(_PADPTS) + (a + (b_ - a) * u))
    P = np.concatenate(pts)
    hits = []
    for j in range(len(ids)):
        if j == skip_k or ids[j] in gone or ids[j] in HANDED:
            continue
        c, R, h = _obb(j)
        loc = R.inv().apply(P - c)
        if np.any(np.all(np.abs(loc) < h + 0.01, axis=1)):
            hits.append(ids[j])
    if np.any((np.abs(P[:, 0]) > WALL_X) & (P[:, 1] > -0.8)):
        hits.append("container_wall")
    if np.any((P[:, 2] > ROOF_Z) & (P[:, 1] > -0.8)):
        hits.append("container_roof")
    if np.any(P[:, 2] < -0.01):
        hits.append("floor")
    return hits


def _ik(pos, Rg, base_y):
    Bm = np.eye(4)
    Bm[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    Bm[:3, 3] = [0.0, base_y, 0.62]
    Ew = np.eye(4)
    Ew[:3, :3], Ew[:3, 3] = Rg.as_matrix(), pos
    tb = np.linalg.inv(Bm) @ Ew @ AG_TOOL
    x_, y_, z_, w_ = Rotation.from_matrix(tb[:3, :3]).as_quat()
    for seed in (AG_Q.get(base_y), np.array([0.0, -0.6, 0.9, 0.0, 0.6, 0.0]), np.array([0.0, -1.0, 1.4, 0.0, -0.4, 0.0])):
        if seed is None:
            continue
        q, ok = AG_KS.compute_inverse_kinematics("tool0", tb[:3, 3], np.array([w_, x_, y_, z_]), warm_start=seed,
                                                 position_tolerance=3e-3, orientation_tolerance=3e-2)
        if ok:
            AG_Q[base_y] = np.asarray(q, float)
            return True
    return False


RETREAT = os.environ.get("U1_RETREAT", "")  # "" | lift | slow | liftslow
LIFT_H = float(os.environ.get("U1_LIFT_H", "0.04"))
_NOLIFT = [False]


def grasp_plan(bid, face, du=0.0, dv=0.0):
    k = ids.index(bid)
    p, n, t1, t2, ext = _face_frame(k, face)
    du, dv = float(np.clip(du, -ext[0], ext[0])), float(np.clip(dv, -ext[1], ext[1]))
    pt = p + du * t1 + dv * t2
    Rg = _pad_R(n)
    contact = pt + n * (PAD[2] + 0.004)
    if face == "front":
        contact[2] = max(contact[2], PAD[1] / 2 + 0.012)
        pre = pt + n * 0.40
        path = [pre, contact]
        retreat = [contact + np.array([0, -0.70, 0.02])]
        if "lift" in RETREAT and not _NOLIFT[0]:  # lift-then-pull: raise the item off what it sits on before sliding it out
            l_ = contact + np.array([0, -0.02, min(LIFT_H, max(0.0, ROOF_Z - 0.13 - contact[2]))])
            retreat = [l_, l_ + np.array([0, -0.68, 0])]
    else:
        pre = pt + n * 0.40
        pre[2] = min(pre[2], ROOF_Z - 0.13)
        stage = np.array([pt[0], -1.15, pre[2]])
        path = [stage, pre, contact]
        lift = contact + np.array([0, 0, max(0.0, min(0.15, ROOF_Z - 0.13 - contact[2]))])
        retreat = [lift, np.array([lift[0], -1.15, lift[2]])]
    g = np.array([[a, b_] for a in np.linspace(-PAD[0] / 2, PAD[0] / 2, 7) for b_ in np.linspace(-PAD[1] / 2, PAD[1] / 2, 7)])
    W = Rg.apply(np.c_[g, np.zeros(len(g))]) + pt - p
    cover = float(np.mean((np.abs(W @ t1) <= ext[0]) & (np.abs(W @ t2) <= ext[1])))
    return dict(k=k, face=face, du=du, dv=dv, point=pt, normal=n, R=Rg, path=path, retreat=retreat, cover=cover, ext=ext)


def _loads_on(k):
    c, R, h = _obb(k)
    top = c[2] + float(np.abs(R.as_matrix()) @ h @ np.array([0, 0, 1]))
    out = []
    for j in range(len(ids)):
        if j == k or ids[j] in gone or ids[j] in HANDED or not in_wall(j):
            continue
        cj, Rj, hj = _obb(j)
        ej = np.abs(Rj.as_matrix()) @ hj
        ek = np.abs(R.as_matrix()) @ h
        if abs((cj[2] - ej[2]) - top) < 0.06 and abs(cj[0] - c[0]) < ej[0] + ek[0] - 0.02 and abs(cj[1] - c[1]) < ej[1] + ek[1] - 0.02:
            out.append(ids[j])
    return out


def check_grasp(bid, face="front", du=0.0, dv=0.0, base_y=None):
    r = _check_grasp0(bid, face, du, dv, base_y)
    if not r.get("ok") and "lift" in RETREAT and "ik" in r and not r.get("sweep_collisions"):
        _NOLIFT[0] = True
        try:
            r2 = _check_grasp0(bid, face, du, dv, base_y)
        finally:
            _NOLIFT[0] = False
        if r2.get("ok"):
            r2["lift"] = False
            return r2
    return r


def _check_grasp0(bid, face="front", du=0.0, dv=0.0, base_y=None):
    if bid not in ids or bid in gone or bid in HANDED or not in_wall(ids.index(bid)):
        return dict(ok=False, error="no such item in the container")
    if face not in ("front", "top"):
        return dict(ok=False, error="face must be front or top")
    g = grasp_plan(bid, face, du, dv)
    hits = _sweep_hits(g["path"], g["R"], g["k"])
    ys = [float(base_y)] if base_y is not None else list(BASE_YS)
    ik = None
    for y in ys:
        y = float(np.clip(y, -2.8, -1.8))
        r = [_ik(p_, g["R"], y) for p_ in g["path"] + g["retreat"]]
        if all(r):
            ik = dict(base_y=y, ok=True)
            break
        ik = dict(base_y=y, ok=False, failed=[nm for nm, ok_ in zip((["stage"] if face == "top" else []) + ["pre", "contact"] +
                                                                    (["lift", "out"] if face == "top" else ["retreat"]), r) if not ok_])
    k = g["k"]
    c, R, _ = _obb(k)
    return dict(ok=bool(ik["ok"] and not hits), item=bid, face=face, du=round(g["du"], 3), dv=round(g["dv"], 3),
                face_half_size_m=[round(float(e), 3) for e in g["ext"]], face_normal=[round(float(v), 2) for v in g["normal"]],
                pad_coverage=round(g["cover"], 2), ik=ik, sweep_collisions=hits, items_resting_on_it=_loads_on(k),
                contact_point=[round(float(v), 3) for v in g["point"]])


def scene():
    out = []
    for k, b in enumerate(B):
        if not in_wall(k) or ids[k] in HANDED:
            continue
        c, R, h = _obb(k)
        ext = np.abs(R.as_matrix()) @ h
        _, nf, *_ = _face_frame(k, "front")
        _, nt, *_ = _face_frame(k, "top")
        out.append(dict(id=b["id"], kind=b.get("kind", "box"), center=[round(float(v), 2) for v in c],
                        size_wdh=[round(b["w"], 2), round(b["d"], 2), round(b["h"], 2)],
                        bbox_min_z=round(float(c[2] - ext[2]), 2), bbox_max_z=round(float(c[2] + ext[2]), 2),
                        tilt_deg=round(float(np.degrees(np.arccos(np.clip(R.apply([0, 0, 1])[2], -1, 1)))), 0),
                        front_face_toward_robot=round(float(-nf[1]), 2), top_face_up=round(float(nt[2]), 2),
                        mass_kg=round(b["mass"], 1), displaced=bool(np.linalg.norm(c - p0[k]) > 0.25),
                        resting_on_it=_loads_on(k), failed_grips=int(AG_FAILS[b["id"]])))
    return out


def show_base(y):
    if not SHOW or abs(SHOW["base"][1, 3] - y) < 1e-6:
        return
    dy = y - SHOW["base"][1, 3]
    for nm in ("base", "baseTop", "bumperF", "bumperR", "wheel0", "wheel1", "wheel2", "wheel3", "lidarF", "beacon", "mast", "cam3d", "arm"):
        pr = st.GetPrimAtPath(f"/World/Show/{nm}")
        if pr.IsValid():
            op = UsdGeom.Xformable(pr).GetOrderedXformOps()[0]
            v = op.Get()
            op.Set(Gf.Vec3d(v[0], v[1] + dy, v[2]))
    SHOW["base"][1, 3] = y


def pick_g(g, name):
    """Execute a checked grasp (same pad/gripper/grip model as pick(); orientation and path from the grasp)."""
    global R_GRASP
    k = g["k"]
    bid = ids[k]
    standing = {ids[j] for j in range(len(ids)) if in_wall(j)}
    cur = eoat.get_world_poses()[0].numpy()[0]
    R_GRASP = g["R"]
    if g.get("base_y") is not None:
        drive_base(g["base_y"])
        cur = eoat.get_world_poses()[0].numpy()[0]
    path = g["path"]
    first = path[0] + np.array([0, -0.25, 0]) if g["face"] == "top" else path[0]
    _ph("approach", bid)
    move_q(first, g["R"], float(os.environ.get("U1_APPROACH_S", "0.6")))
    step(20, first)  # let the pad turn
    prev = first
    for p_ in path:
        move(prev, p_, 0.5 if p_ is not path[-1] else 0.4, g["R"])
        prev = p_
    _ph("grip", bid)
    grip.apply_gripper_action([0.5])
    step(8, prev)
    held = bool(grip.get_gripped_objects()[0])
    gm = None
    if held and GRIP_P and B[k].get("kind") == "bag":
        u_ = GRNG.random()
        if u_ < GRIP_P[0]:
            gm = "model_no_grip"
            grip.apply_gripper_action([-0.5])
            step(4, prev)
            held = False
        elif u_ < GRIP_P[0] + GRIP_P[1]:
            gm = "model_drop"
    before = boxes.get_world_poses()[0].numpy().copy()
    _ph("lift" if "lift" in RETREAT else "pull", bid)
    for i_, p_ in enumerate(g["retreat"]):
        move(prev, p_, (2.4 if "slow" in RETREAT else 0.9) if i_ == len(g["retreat"]) - 1 else 0.6, g["R"])
        if gm == "model_drop" and i_ == 0:
            grip.apply_gripper_action([-0.5])
        prev = p_
    held_after = bool(grip.get_gripped_objects()[0])
    if SHOW and held_after and os.environ.get("U1_PLACE", "1") == "1":
        # showcase: set the item down on the take-away conveyor (turn 90 deg about z so it hangs on the belt side)
        bp_ = boxes.get_world_poses()[0].numpy()[k].copy()
        Rz_ = Rotation.from_euler("z", 90, degrees=True)
        off_ = Rz_.apply(bp_ - prev)  # box centre relative to the pad after the turn
        # release height: a side-held pad (0.36 x 0.30) hangs ~0.18 below the box centre -> keep it above the belt lip
        side_ = g["face"] != "top"
        def _variant(yaw_, dy_, zc_):
            Rz_ = Rotation.from_euler("z", yaw_, degrees=True)
            Rp_ = Rz_ * g["R"]
            btgt = np.array([CONV_X, CONV_Y + dy_, CONV_TOP + max(B[k]["h"] / 2, 0.24 if side_ else 0.0) + 0.04])
            tgt = btgt - Rz_.apply(bp_ - prev)
            # pad housing must stay above the belt: lowest housing corner >= belt top + 3 cm
            cz_ = min((Rp_.apply([sx * PAD[0] / 2, sy * PAD[1] / 2, sz * 0.12]) + tgt)[2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))
            if cz_ < CONV_TOP + 0.03:
                tgt = tgt + np.array([0, 0, CONV_TOP + 0.03 - cz_])
            z_safe = max(prev[2], tgt[2] + zc_)
            up_ = np.array([prev[0], prev[1], z_safe])
            above_ = np.array([tgt[0], tgt[1], z_safe])
            sl_ = Slerp([0.0, 1.0], Rotation.concatenate([g["R"], Rp_]))
            poses = [(prev + (up_ - prev) * u, g["R"]) for u in np.linspace(0.2, 1, 4)]
            poses += [(up_ + (above_ - up_) * u, sl_(u)) for u in np.linspace(0.1, 1, 8)]
            poses += [(above_ + (tgt - above_) * u, Rp_) for u in np.linspace(0.2, 1, 4)]
            poses += [(tgt + np.array([0, 0, 0.35]) * u, Rp_) for u in (0.5, 1.0)]
            return tgt, Rp_, z_safe, up_, above_, poses

        pick_ = None
        for v_ in [(y_, d_, z_) for y_ in (90, -90, 0, 180) for d_ in (0.0, -0.35, -0.7) for z_ in (0.45, 0.30, 0.65)]:
            if _arm_path_ok(_variant(*v_)[5]):
                pick_ = v_
                break
        tgt, Rp_, z_safe, up_, above_, _ = _variant(*(pick_ or (90, 0.0, 0.45)))
        print(f"[u1] place {bid}: {'yaw %d dy %.2f carry %.2f' % pick_ if pick_ else 'no clean path (last variant)'}", flush=True)
        _PLACE_LOG.append(dict(item=bid, variant=pick_))
        _ph("transfer", bid)
        if z_safe - prev[2] > 0.02:
            move(prev, up_, 0.5, g["R"])
        move(up_, above_, 0.9, Rp_)
        _ph("place", bid)
        move(above_, tgt, 0.6, Rp_)  # 3) vertical set-down
        grip.apply_gripper_action([-0.5])
        step(4, tgt)
        _ph("release", bid)
        prev = tgt + np.array([0.0, 0, 0.35])
        move(tgt, prev, 0.4, Rp_)  # 4) straight up off the belt
        for _ in range(45):
            boxes.set_velocities(linear_velocities=np.array([[0.0, -1.1, 0.0]]), angular_velocities=np.zeros((1, 3)), indices=[k])
            step(1, prev)
    grip.apply_gripper_action([-0.5])
    step(2, prev)
    if held_after:
        boxes.set_world_poses(positions=np.array([[5 + len(gone) * 0.9, -2.5, 0.4]]), indices=[k])
        boxes.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)), indices=[k])
        gone.add(bid)
    hold_p = prev
    R_GRASP = None
    step(40, hold_p)
    Pa = boxes.get_world_poses()[0].numpy()
    moved = [ids[j] for j in range(len(ids)) if ids[j] in standing and ids[j] not in gone and ids[j] != bid and np.linalg.norm(Pa[j] - before[j]) > 0.05]
    fell = [j for j in moved if Pa[ids.index(j)][2] < before[ids.index(j)][2] - 0.10]
    LOG.append(dict(id=bid, result="picked" if held_after else ("grip_lost" if held else "no_grip"), disturbed=moved, fell=fell,
                    kind=B[k].get("kind", "box"), grip_model=gm, face=g["face"], du=g["du"], dv=g["dv"], base_y=g.get("base_y")))
    print(f"[u1] {name} pick {bid} {g['face']}: {LOG[-1]['result']} disturbed={moved} fell={fell}", flush=True)


_EVS = []
_PH = []
_PLACE_LOG = []


def _ph(name, item=None):
    """Motion phase marker for the video side panel: phases.json = [{f, phase, item}] in output frame numbers."""
    if not frames_dir or not SHOW:
        return
    _PH.append(dict(f=fcount[0] // max(1, args.show_every), phase=name, item=item))
    json.dump(_PH, open(os.path.join(frames_dir, "phases.json"), "w"))


def _EVW(t, d_, f0):
    if not frames_dir:
        return
    _EVS.append(dict(t=t, f0=f0, f1=fcount[0] // max(1, args.show_every), action=d_.get("action"), item=d_.get("item"),
                     face=d_.get("face"), reason=(d_.get("reason") or "")[:240], result=LOG[-1].get("result") if LOG else None,
                     fell_now=LOG[-1].get("fell", []) if LOG else [],
                     picked=sum(1 for e in LOG if e.get("result") == "picked"),
                     handed=sum(1 for e in LOG if e.get("result") == "handoff"),
                     fell=len({b for e in LOG for b in e.get("fell", [])})))
    json.dump(_EVS, open(os.path.join(frames_dir, "events.json"), "w"), indent=1, default=str)


def handoff(bid, reason=""):
    k = ids.index(bid)
    boxes.set_world_poses(positions=np.array([[5 + len(HANDED) * 0.9, -3.8, 0.4]]), indices=[k])
    boxes.set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)), indices=[k])
    HANDED.add(bid)
    step(30, eoat.get_world_poses()[0].numpy()[0])
    LOG.append(dict(id=bid, result="handoff", kind=B[k].get("kind", "box"), reason=reason[:200]))
    print(f"[u1] handoff {bid}: {reason[:80]}", flush=True)


def ikrule_decide(tab):
    """Fixed policy with the same tools: E1 rule order; front grasp, top grasp if the front face does not look at the
    robot (|n_y| < 0.7) or nothing executable from the front; first executable base position; skip if none."""
    for bid in [i for i in PLANNERS["rule"](tab) if AG_FAILS[i] < 2]:
        k = ids.index(bid)
        _, nf, *_ = _face_frame(k, "front")
        faces = ["front", "top"] if -nf[1] >= 0.7 else ["top", "front"]
        for f in faces:
            r = check_grasp(bid, f)
            if r["ok"]:
                return dict(action="execute", item=bid, face=f, du=0.0, dv=0.0, base_y=r["ik"]["base_y"])
    return dict(action="finish")


AG_PROMPT = """You are the grasp-planning agent of a truck-unloading robot: KUKA KR210 L150 on a mobile base at a container door, with one
front suction pad 36 x 30 cm (it can be turned to grip a top face from above). The container mouth is at y = -0.8, items are
inside at y >= 0 (x left/right within +-1.15 m, z up, roof at 2.69 m). The load is irregular: cartons, plastic bags, upright and
lying tubes; items lean, are pushed back or have fallen inside after earlier picks.

Tools (MCP server 'cell'): get_scene, check_grasp(item, face, du, dv, base_y), execute_grasp(...), hand_off(item, reason), finish().
A grasp = item + face ('front' = the face looking most toward the robot, 'top' = the face looking most up) + offset du/dv in
metres on that face (du = left/right, dv = the other in-face direction, 0/0 = face centre) + mobile-base y (-2.8 far .. -1.8 close
to the door; omit it to let check_grasp try -2.8, -2.3, -1.8). check_grasp runs KR210 inverse kinematics on every waypoint and a pad
sweep collision check; execute_grasp refuses grasps that do not pass.

Goal for the whole container: unload as many items as possible by robot; an item that falls on the floor costs more than a hand-off
to a person, and a hand-off costs more than a slightly slower pick. Physics you should reason about: pulling an item out from the
front drags what rests on it (items_resting_on_it) and can topple leaning neighbours; items that lean or lie on their side often
have no good front face; bags and tubes slip; a pad that hangs over a face edge (pad_coverage < 0.6) grips poorly.

This turn: decide the ONE next action. Usually: get_scene, then check_grasp on 1-4 promising grasps, then execute_grasp (or hand_off
an item that no grasp can take safely, or finish if nothing is left that the robot should take). Keep it short.
Recent outcomes (newest last): {hist}"""


AG_PROMPT_V2 = AG_PROMPT.replace(
    """have no good front face; bags and tubes slip; a pad that hangs over a face edge (pad_coverage < 0.6) grips poorly.""",
    """have no good front face; bags and tubes slip; pad_coverage is the share of the pad on the face (low values are a risk mainly for
bags, tubes and heavy items).
A catch conveyor covers the container floor in front of the wall (y -1.0 .. 0, x within +-0.72): items that fall onto it are carried
out and count as unloaded, so a short fall onto it is cheap; a fall from high up (item above ~1.5 m) or outside the belt (near the side
walls) risks damage or loss. Score = items unloaded (robot picks + caught by the belt), minus damage, minus hand-offs.
Hand off only when no grasp passes or the item would likely drop from height; a person's trip is expensive.""")


AG_PROMPT_V3 = AG_PROMPT.replace(
    """have no good front face; bags and tubes slip; a pad that hangs over a face edge (pad_coverage < 0.6) grips poorly.""",
    """have no good front face; bags and tubes slip; pad_coverage is the share of the pad on the face (low values are a risk mainly for
bags, tubes and heavy items).
There is no catch device: every item that falls to the floor counts as damage. Score = robot picks - 2 x fallen items - hand-offs.
Before a front pull, look at items_resting_on_it: take the items on top first (top grasp) or pick so nothing is dragged off.
Hand off only when no grasp passes or every passing grasp would likely make something fall; a person's trip is expensive.""")


AG_PREVIEW_NOTE = """
Extra tool preview_grasp(item, face, du, dv, base_y): runs that grasp in a physics twin of the current wall and reports
would_fall / would_disturb, then restores the wall. Use it on your best 1-3 passing candidates when items rest on or lean
against the target, then execute the one with the fewest would_fall."""


def _agp():
    return _agp0() + (AG_PREVIEW_NOTE if PREVIEW else "")


def _agp0():
    v = os.environ.get("U1_PROMPT", "")
    return AG_PROMPT_V3 if v == "v3" else (AG_PROMPT_V2 if CATCH else AG_PROMPT)


PREVIEW = os.environ.get("U1_PREVIEW", "") == "1"
PREV_RNG = np.random.default_rng(args.seed * 53 + 11)
PREV_DRIFT = []


def preview_grasp(bid, face="front", du=0.0, dv=0.0, base_y=None):
    """Try the grasp in the sim, see what moves and falls, then put everything back (ground-truth twin)."""
    global GRNG
    r = check_grasp(bid, face, du, dv, base_y)
    if not r.get("ok"):
        return dict(r, ok=False, error="grasp does not pass IK/collision; not previewed")
    snap = _snapshot()
    real_rng, GRNG = GRNG, np.random.default_rng(PREV_RNG.integers(1 << 30))
    _NOLIFT[0] = ("lift" in RETREAT) and not r.get("lift", True)
    g = grasp_plan(bid, face, r["du"], r["dv"])
    _NOLIFT[0] = False
    g["base_y"] = r["ik"]["base_y"]
    try:
        pick_g(g, "preview")
        e = dict(LOG[-1])
    finally:
        GRNG = real_rng
        _restore(snap)
    step(10, snap[2][0])
    P = boxes.get_world_poses()[0].numpy()
    live = [k for k in range(len(ids)) if ids[k] not in gone]
    drift = float(max(np.linalg.norm(P[k] - snap[0][k]) for k in live)) if live else 0.0
    PREV_DRIFT.append(round(drift, 4))
    print(f"[u1] preview {bid} {face}: {e['result']} fell={e['fell']} drift={drift:.4f}", flush=True)
    return dict(ok=True, item=bid, face=face, du=r["du"], dv=r["dv"], base_y=r["ik"]["base_y"], result=e["result"],
                would_fall=e["fell"], would_disturb=e["disturbed"],
                note="simulated outcome on the current state; real grip of bags/tubes is uncertain")


def opus_decide(t):
    import subprocess as _sp
    import time as _time
    io_dir = os.path.join(HERE, "agent_io", f"{SC or 'base'}_{args.seed}")
    os.makedirs(io_dir, exist_ok=True)
    for f_ in os.listdir(io_dir):
        try:
            os.remove(os.path.join(io_dir, f_))
        except OSError:
            pass
    cfg = os.path.join(io_dir, "..", f"mcp_{SC or 'base'}_{args.seed}.json")
    json.dump({"mcpServers": {"cell": {"command": "python", "args": [os.path.join(HERE, "u1_mcp.py"), io_dir] + (["preview"] if PREVIEW else [])}}}, open(cfg, "w"))
    hist = "; ".join(f"{e['id']}:{e['result']}" + (f"(fell {e['fell']})" if e.get("fell") else "") for e in LOG[-8:]) or "none yet"
    cmd = [AG_CLAUDE, "-p", _agp().replace("{hist}", hist), "--model", AG_MODEL, "--effort", AG_EFFORT, "--output-format", "json",
           "--mcp-config", cfg, "--strict-mcp-config", "--setting-sources", "project", "--tools", "",
           "--allowedTools", "mcp__cell__get_scene,mcp__cell__check_grasp,mcp__cell__execute_grasp,mcp__cell__hand_off,mcp__cell__finish" + (",mcp__cell__preview_grasp" if PREVIEW else ""),
           "--max-turns", "14", "--no-session-persistence",
           "--system-prompt", "You are the grasp-planning layer of a robot cell. Use only the cell tools. End after one execute_grasp, hand_off or finish."]
    if args.agent == "astra":
        _m = os.path.join(HERE, "u1_mcp.py").replace("\\", "/")
        _io = io_dir.replace("\\", "/")
        _srv = f'mcp_servers={{cell={{command="python",args=["{_m}","{_io}"{',"preview"' if PREVIEW else ''}],default_tools_approval_mode="approve"}}}}'
        cmd = AG_CODEX + ["exec", "--skip-git-repo-check", "--json", "--sandbox", "read-only", "-m", AG_MODEL,
               "-c", 'approval_policy="never"', "-c", f'model_reasoning_effort="{AG_EFFORT}"', "-c", _srv,
               "You are the grasp-planning layer of a robot cell. Use only the cell MCP tools (no shell, no files). "
               "End after one execute_grasp, hand_off or finish.\n\n" + _agp().replace("{hist}", hist)]
    t0_ = _time.perf_counter()
    _of, _ef = os.path.join(io_dir, f"_out_{t}.txt"), os.path.join(io_dir, f"_err_{t}.txt")
    _ofh, _efh = open(_of, "w", encoding="utf-8"), open(_ef, "w", encoding="utf-8")
    pr = _sp.Popen(cmd, stdout=_ofh, stderr=_efh, stdin=_sp.DEVNULL, cwd=io_dir)

    def _killtree():
        _sp.run(["taskkill", "/T", "/F", "/PID", str(pr.pid)], stdout=_sp.DEVNULL, stderr=_sp.DEVNULL)
    decision, calls, t_dec = None, [], None
    while True:
        for f_ in sorted(os.listdir(io_dir)):
            if not (f_.startswith("req_") and f_.endswith(".json")):
                continue
            rp = os.path.join(io_dir, f_)
            try:
                req = json.load(open(rp))
            except Exception:
                continue
            os.remove(rp)
            tool, a = req.get("tool"), req.get("args") or {}
            try:
                if tool == "get_scene":
                    res = dict(items=scene(), picked_so_far=len(gone), handed_off=len(HANDED))
                elif tool == "check_grasp":
                    res = check_grasp(int(a["item"]), a.get("face", "front"), float(a.get("du", 0)), float(a.get("dv", 0)), a.get("base_y"))
                elif tool == "preview_grasp" and PREVIEW:
                    res = preview_grasp(int(a["item"]), a.get("face", "front"), float(a.get("du", 0)), float(a.get("dv", 0)), a.get("base_y"))
                elif tool == "execute_grasp":
                    if decision:
                        res = dict(ok=False, error="a decision was already made this turn; stop now")
                    else:
                        r = check_grasp(int(a["item"]), a.get("face", "front"), float(a.get("du", 0)), float(a.get("dv", 0)), a.get("base_y"))
                        if r["ok"]:
                            decision = dict(action="execute", item=int(a["item"]), face=a.get("face", "front"), du=r["du"], dv=r["dv"], base_y=r["ik"]["base_y"])
                            res = dict(ok=True, note="accepted; it runs after you end this turn. Stop now.")
                        else:
                            res = dict(r, ok=False, error="grasp does not pass IK/collision; not executed")
                elif tool == "hand_off":
                    decision = decision or dict(action="handoff", item=int(a["item"]), reason=str(a.get("reason", "")))
                    res = dict(ok=True, note="accepted. Stop now.")
                elif tool == "finish":
                    decision = decision or dict(action="finish")
                    res = dict(ok=True)
                else:
                    res = dict(ok=False, error="unknown tool")
            except Exception as e:
                res = dict(ok=False, error=repr(e)[:300])
            calls.append(dict(tool=tool, args=a, ok=res.get("ok")))
            json.dump(res, open(rp.replace("req_", "resp_") + ".tmp", "w"))
            os.replace(rp.replace("req_", "resp_") + ".tmp", rp.replace("req_", "resp_"))
        if decision is not None and t_dec is None:
            t_dec = _time.perf_counter()
        if pr.poll() is not None:
            break
        if (t_dec is not None and _time.perf_counter() - t_dec > 20) or _time.perf_counter() - t0_ > 600:
            _killtree()
            break
        _time.sleep(0.05)
    try:
        pr.wait(timeout=10)
    except Exception:
        pass
    _ofh.close()
    _efh.close()
    out = open(_of, encoding="utf-8", errors="replace").read()
    err = open(_ef, encoding="utf-8", errors="replace").read()
    dt = _time.perf_counter() - t0_
    rec = dict(t=t, latency_s=round(dt, 1), calls=calls, decision=decision)
    if args.agent == "astra":
        u = {}
        msgs = []
        for ln in out.splitlines():
            try:
                ev = json.loads(ln)
            except Exception:
                continue
            if ev.get("type") == "turn.completed":
                u = ev.get("usage") or {}
            it = ev.get("item") or {}
            if ev.get("type") == "item.completed" and it.get("type") == "agent_message":
                msgs.append(it.get("text", ""))
        rec.update(cost_usd=None, tok_in=u.get("input_tokens"), tok_cached=u.get("cached_input_tokens"), tok_out=u.get("output_tokens"),
                   result=(msgs[-1] if msgs else "")[:300], stderr=None if decision else err[-300:])
        AG_TRACE.append(rec)
        print(f"[u1] agent t{t} {dt:.1f}s calls {len(calls)} -> {decision}", flush=True)
        return decision
    try:
        j = json.loads(out)
        u = j.get("usage") or {}
        rec.update(cost_usd=j.get("total_cost_usd"), turns=j.get("num_turns"), is_error=j.get("is_error"), result=(j.get("result") or "")[:300],
                   tok_out=u.get("output_tokens"))
    except Exception:
        rec.update(stdout=out[-300:], stderr=err[-300:])
    AG_TRACE.append(rec)
    print(f"[u1] agent t{t} {dt:.1f}s calls {len(calls)} -> {decision}", flush=True)
    return decision


if args.agent:
    import collections as _co
    import shutil
    import sys as _sys

    _sys.path.insert(0, HERE)
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
    from u1_planners import PLANNERS

    _LULA = r"C:\isaacsim\extsDeprecated\isaacsim.robot_motion.motion_generation\motion_policy_configs\Kuka\kr210"
    AG_KS = LulaKinematicsSolver(robot_description_path=os.path.join(_LULA, "rmpflow", "kuka_kr210_robot_description.yaml"),
                                 urdf_path=os.path.join(_LULA, "kr210.urdf"))
    _RE = np.eye(4)
    _RE[:3, :3] = Rotation.from_euler("y", 90, degrees=True).as_matrix()
    _OFF = np.eye(4)
    _OFF[2, 3] = -0.20
    AG_TOOL = _OFF @ np.linalg.inv(_RE)
    AG_Q = {}
    AG_CLAUDE = os.path.expanduser(r"~\AppData\Roaming\npm\node_modules\@anthropic-ai\claude-code\bin\claude.exe")
    AG_MODEL = os.environ.get("U1_AG_MODEL", "gpt-6-astra" if args.agent == "astra" else "claude-opus-5-5")
    AG_EFFORT = os.environ.get("U1_AG_EFFORT", "low")
    AG_CODEX = [shutil.which("node"), os.path.expandvars(r"%APPDATA%\npm\node_modules\@openai\codex\bin\codex.js")]
    CLOSED = True
    AG_FAILS = _co.Counter()
    name = f"agent_{args.agent}" + os.environ.get("U1_TAG", "")
    errs = 0
    for t in range(160):
        tab = live_table_cl()
        tab = [r for r in tab if r["id"] not in HANDED]
        if not [r for r in tab if AG_FAILS[r["id"]] < 2]:
            break
        if args.agent == "replay":
            if t == 0:
                _RP = [r_.get("decision") for r_ in json.load(open(os.environ["U1_REPLAY"]))]
                _F0, _F1 = int(os.environ.get("U1_SHOW_FROM", "0")), int(os.environ.get("U1_SHOW_TO", "9999"))
            if t >= len(_RP) or t > _F1:
                break
            if SHOW and t == _F0 and frames_dir is None:
                FAST, frames_dir = False, args.show
                os.makedirs(frames_dir, exist_ok=True)
                step(30, eoat.get_world_poses()[0].numpy()[0])
            d_ = _RP[t]
            _EV0 = fcount[0] // max(1, args.show_every)
        else:
            try:
                d_ = ikrule_decide(tab) if args.agent == "ikrule" else opus_decide(t)
            except Exception as _e:
                import traceback as _tb
                print("[u1] agent exception " + repr(_e)[:200] + " | " + _tb.format_exc().replace("\n", " / ")[-600:], flush=True)
                d_ = None
        if args.agent == "ikrule":
            AG_TRACE.append(dict(t=t, decision=d_))
        if d_ is None:
            errs += 1
            if errs >= 3:
                break
            continue
        errs = 0
        if d_["action"] == "finish":
            break
        if d_["action"] == "handoff":
            if d_["item"] in ids and d_["item"] not in gone and d_["item"] not in HANDED:
                handoff(d_["item"], d_.get("reason", ""))
            if args.agent == "replay":
                _EVW(t, d_, _EV0)
            continue
        _NOLIFT[0] = ("lift" in RETREAT) and not check_grasp(d_["item"], d_["face"], d_["du"], d_["dv"], d_["base_y"]).get("lift", True)
        g_ = grasp_plan(d_["item"], d_["face"], d_["du"], d_["dv"])
        _NOLIFT[0] = False
        g_["base_y"] = d_["base_y"]
        pick_g(g_, name)
        if LOG[-1]["result"] != "picked":
            AG_FAILS[d_["item"]] += 1
        if args.agent == "replay":
            _EVW(t, d_, _EV0)
    json.dump(AG_TRACE, open(os.path.join(HERE, "results" + SFX, f"{name}_{args.seed}_trace.json"), "w"), indent=1, default=str)
    finish(name, dict(agent=args.agent, model=AG_MODEL if args.agent != "ikrule" else None, effort=AG_EFFORT if args.agent != "ikrule" else None,
                      handed_off=sorted(HANDED), n_handoff=len(HANDED),
                      agent_cost_usd=sum((r.get("cost_usd") or 0) for r in AG_TRACE),
                      decision_latency_s=float(np.mean([r["latency_s"] for r in AG_TRACE if "latency_s" in r])) if args.agent != "ikrule" and AG_TRACE else None,
                      grip_model_env=os.environ.get("U1_GRIP", ""), preview=PREVIEW, preview_drift_max=max(PREV_DRIFT) if PREV_DRIFT else None,
                      n_preview=len(PREV_DRIFT)))


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

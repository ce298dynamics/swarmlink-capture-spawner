"""
============================================================================
===[ EDITOR SCRIPT: IMPORT A SCANNED PERSON (OR ANY PROP) FROM FBX ]===
============================================================================
Imports an FBX as a spawnable StaticMesh, imports the textures beside it, builds
a Material from them, and assigns it -- so AirSim can spawn the result directly.

Written against RenderPeople "posed people" downloads (free, no registration,
https://renderpeople.com/free-3d-people/) but nothing here is specific to them:
any FBX with a sibling texture folder works.

USAGE
-----
Point FBX_PATH at your download, either by environment variable:

    set AIRSIM_PROP_FBX=C:\\path\\to\\rp_posed_00178_29.fbx
    UE4Editor-Cmd.exe <your>.uproject -run=pythonscript -script="ue_import_person.py"

...or by editing FBX_PATH below.  Run with the EDITOR CLOSED -- UE locks the
project.  Afterwards restart the editor, press Play, then check the spawn name:

    py probe_env.py --grep <asset name>

WHY STATIC AND NOT SKELETAL
---------------------------
AirSim's asset registry admits only UStaticMesh and UBlueprint
(AirBlueprintLib.cpp:241), so a SkeletalMesh never appears in simListAssets()
and can never be spawned.  import_as_skeletal is therefore forced False.

A posed scan has no rig anyway, and that is the point: a RIGGED character
carries its animation set's posture.  The stock UE mannequin's animations all
sit on a rifle-carry rig, so every pose holds the arms up at chest height.  A
scan is simply the pose the person was photographed in.

WHY THE TEXTURES NEED THEIR OWN PASS
------------------------------------
The FBX import brings in the MESH ONLY.  RenderPeople FBX files reference their
maps from a sibling tex/ folder and the importer does not follow it, so the scan
lands with no material and renders flat grey -- which defeats the whole purpose
of using a photoreal human for perception data.  This script imports the maps
itself and wires them up.

The normals map MUST be flagged TC_NORMALMAP or it renders as a blue sheen
instead of surface detail.

WHY IT RUNS HEADLESS
--------------------
This is pure asset work -- no level, no world -- so a commandlet is fine.
Anything touching EditorLevelLibrary is NOT: under -run=pythonscript there is no
editor world, and the first such call dereferences null and kills the process
with EXCEPTION_ACCESS_VIOLATION rather than raising.
"""

import os
import unreal

# --- configure ---------------------------------------------------------------
# Environment variable wins; otherwise edit this default.
FBX_PATH = os.environ.get("AIRSIM_PROP_FBX", r"C:\path\to\your_model.fbx")

# Where assets land inside the project.  /Game maps to <project>/Content.
DEST = "/Game/Props/People"

# Texture suffix -> (material input, is normal map, sRGB).  RenderPeople's naming;
# adjust for other vendors.
MAPS = [
    ("basecolor", unreal.MaterialProperty.MP_BASE_COLOR, False, True),
    ("normals",   unreal.MaterialProperty.MP_NORMAL,     True,  False),
    ("roughness", unreal.MaterialProperty.MP_ROUGHNESS,  False, False),
    ("specular",  unreal.MaterialProperty.MP_SPECULAR,   False, False),
]

_lines = []


def log(msg):
    unreal.log("[import-person] %s" % msg)
    _lines.append(str(msg))


def _import(tools, path, dest):
    """One import task.  Returns the created asset's object path, or None."""
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", path)
    task.set_editor_property("destination_path", dest)
    task.set_editor_property("automated", True)       # suppress modal dialogs
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    return task


def import_mesh(tools, fbx, dest):
    task = _import(tools, fbx, dest)
    opts = unreal.FbxImportUI()
    opts.set_editor_property("import_mesh", True)
    opts.set_editor_property("import_as_skeletal", False)   # see header
    opts.set_editor_property("import_animations", False)
    opts.set_editor_property("import_materials", True)
    opts.set_editor_property("import_textures", True)
    opts.set_editor_property("mesh_type_to_import",
                             unreal.FBXImportType.FBXIT_STATIC_MESH)
    sm = opts.static_mesh_import_data
    sm.set_editor_property("combine_meshes", True)          # one asset, one spawn
    sm.set_editor_property("generate_lightmap_u_vs", True)
    sm.set_editor_property("auto_generate_collision", True)
    task.set_editor_property("options", opts)
    tools.import_asset_tasks([task])
    made = list(task.get_editor_property("imported_object_paths") or [])
    return made[0] if made else None


def find_textures(fbx):
    """Locate the maps beside the FBX: ./tex/<base>_<suffix>.* or ./<base>_...

    Returns {suffix: path}.  Missing maps are simply skipped -- only basecolor
    is required.
    """
    base = os.path.splitext(os.path.basename(fbx))[0]
    root = os.path.dirname(fbx)
    found = {}
    for suffix, _prop, _isn, _srgb in MAPS:
        for folder in (os.path.join(root, "tex"), root):
            if not os.path.isdir(folder):
                continue
            for ext in (".jpg", ".jpeg", ".png", ".tga"):
                p = os.path.join(folder, "%s_%s%s" % (base, suffix, ext))
                if os.path.isfile(p):
                    found[suffix] = p
                    break
            if suffix in found:
                break
    return found


def build_material(tools, base, textures, dest):
    mat_name = "M_%s" % base
    mat_pkg = "%s/%s" % (dest, mat_name)
    if unreal.EditorAssetLibrary.does_asset_exist(mat_pkg):
        unreal.EditorAssetLibrary.delete_asset(mat_pkg)
    mat = tools.create_asset(mat_name, dest, unreal.Material,
                             unreal.MaterialFactoryNew())
    if mat is None:
        log("FAILED: could not create material")
        return None
    y = -400
    for suffix, prop, _isn, _srgb in MAPS:
        tex = textures.get(suffix)
        if tex is None:
            continue
        node = unreal.MaterialEditingLibrary.create_material_expression(
            mat, unreal.MaterialExpressionTextureSample, -400, y)
        node.set_editor_property("texture", tex)
        # colour and normal are 3-channel; roughness/specular are scalar
        channel = "RGB" if suffix in ("basecolor", "normals") else "R"
        unreal.MaterialEditingLibrary.connect_material_property(node, channel, prop)
        log("  wired %-10s via %s" % (suffix, channel))
        y += 250
    unreal.MaterialEditingLibrary.recompile_material(mat)
    unreal.EditorAssetLibrary.save_asset(mat_pkg)
    return mat


def main():
    try:
        if not os.path.isfile(FBX_PATH):
            log("FAILED: no FBX at %r" % FBX_PATH)
            log("  set AIRSIM_PROP_FBX, or edit FBX_PATH at the top of this file")
            return
        base = os.path.splitext(os.path.basename(FBX_PATH))[0]
        tools = unreal.AssetToolsHelpers.get_asset_tools()
        for d in (DEST, "%s/Textures" % DEST):
            if not unreal.EditorAssetLibrary.does_directory_exist(d):
                unreal.EditorAssetLibrary.make_directory(d)

        # ---- 1. the mesh ----
        log("importing %s (%.1f MB)" % (os.path.basename(FBX_PATH),
                                        os.path.getsize(FBX_PATH) / 1048576.0))
        mesh_path = import_mesh(tools, FBX_PATH, DEST)
        if not mesh_path:
            log("FAILED: mesh import produced nothing")
            return
        mesh = unreal.load_asset(mesh_path.split('.')[0])
        log("mesh: %s (%s)" % (mesh_path, type(mesh).__name__))

        # ---- 2. the textures the FBX importer did not follow ----
        srcs = find_textures(FBX_PATH)
        if not srcs:
            log("WARNING: no textures found beside the FBX -- it will render grey")
        textures = {}
        for suffix, _prop, is_normal, srgb in MAPS:
            if suffix not in srcs:
                continue
            p = _import(tools, srcs[suffix], "%s/Textures" % DEST)
            tools.import_asset_tasks([p])
            made = list(p.get_editor_property("imported_object_paths") or [])
            if not made:
                log("  texture import failed: %s" % suffix)
                continue
            tex = unreal.load_asset(made[0].split('.')[0])
            if tex is None:
                continue
            if is_normal:
                # without this a normal map renders as a blue sheen
                tex.set_editor_property(
                    "compression_settings",
                    unreal.TextureCompressionSettings.TC_NORMALMAP)
            tex.set_editor_property("srgb", srgb)
            unreal.EditorAssetLibrary.save_asset(made[0].split('.')[0])
            textures[suffix] = tex
            log("  imported %s" % suffix)

        # ---- 3. material, and assign it ----
        if "basecolor" in textures:
            mat = build_material(tools, base, textures, DEST)
            if mat is not None:
                mats = mesh.get_editor_property("static_materials")
                if mats:
                    mats[0].material_interface = mat
                    mesh.set_editor_property("static_materials", mats)
                    unreal.EditorAssetLibrary.save_asset(mesh_path.split('.')[0])
                    log("material assigned to slot 0")
                else:
                    log("WARNING: mesh reports no material slots")

        log("")
        log("DONE. Spawn name is the BARE asset name: %r" % base)
        log("  (the registry is keyed by name, not by /Game/... path)")
        log("Restart the editor, press Play, then: py probe_env.py --grep %s"
            % base[:12])
        log("Then measure it before placing -- scan pivots are often NOT at the")
        log("feet, so a prop can hover. maze_props.py's catalogue carries a")
        log("per-prop z offset for exactly that.")
    except Exception:
        import traceback
        log("EXCEPTION\n%s" % traceback.format_exc())
    finally:
        # commandlet stdout is unreliable; leave a file next to the FBX
        try:
            out = os.path.join(os.path.dirname(FBX_PATH) or ".",
                               "import_report.txt")
            open(out, "w").write("\n".join(_lines))
        except Exception:
            pass


main()

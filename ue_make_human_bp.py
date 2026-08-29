"""
============================================================================
===[ EDITOR SCRIPT: POSED HUMAN BLUEPRINTS ]===
============================================================================
Builds Blueprint actors holding the tutorial character's skeletal mesh pinned to
a chosen animation, so AirSim can spawn a human in a pose other than the default
weapon-ready idle.

    BP_Human_Walk   posed by Tutorial_Walk_Fwd   (mid-stride)
    BP_Human_Idle   posed by Tutorial_Idle       (the default, for comparison)

Those are the ONLY two animations the engine ships for this skeleton, so two
poses is the hard ceiling with stock content.  Real pose variety needs posed
human static meshes (Fab "scanned people" and similar), which need none of this.

WHY A BLUEPRINT AT ALL
----------------------
AirSim has no animation API: simSpawnObject takes a transform and a scale, and
the client exposes only simSetObjectPose / simSetVehiclePose / simSetCameraPose.
Nothing selects a skeletal pose at runtime.

Driving the actor with repeated simSetObjectPose calls does not help, and this
was measured rather than assumed: TutorialTPP_AnimBlueprint blends idle->walk on
VELOCITY, and a teleport registers no velocity, so frames captured before and
during a 10 m slide showed an identical pose.  The pose has to be baked in.

HOW THIS WORKS, AND THE FOUR DEAD ENDS BEHIND IT
------------------------------------------------
It DUPLICATES the engine's TutorialCharacter Blueprint and retunes the copy's
mesh component on the class default object.  That looks indirect, but every more
obvious route is unavailable in UE 4.27:

  * unreal.SubobjectDataSubsystem (add a component to a Blueprint) is 5.x only.
  * EditorLevelLibrary.convert_actors_to_blueprint does not exist; 4.27 has only
    convert_actors, which swaps actor CLASS and cannot make a Blueprint.
  * unreal.BlueprintEditorLibrary and unreal.KismetEditorUtilities do not exist.
  * bp.generated_class() and get_editor_property("generated_class") both fail --
    the generated class must be loaded by path as <package>.<Name>_C.

Duplicating sidesteps all of it: TutorialCharacter ALREADY has a configured
SkeletalMeshComponent, so nothing has to be constructed.

RUN IT HEADLESS, WITH THE EDITOR CLOSED
---------------------------------------
    UE4Editor-Cmd.exe <path>\\Blocks.uproject ^
        -run=pythonscript -script="<this file>"

This script deliberately touches NO level API.  Under -run=pythonscript there is
no editor world, and the first EditorLevelLibrary call dereferences null and
kills the process with EXCEPTION_ACCESS_VIOLATION rather than raising -- which is
what the earlier spawn-an-actor-then-convert approach hit.  Asset-only work is
fine headless.

Afterwards RESTART the editor and press Play: AirSim builds its asset map once
at SimMode startup, so new assets stay invisible until PIE restarts.  Confirm
with:  py probe_env.py --grep BP_Human
"""

import unreal

OUT_DIR = "/Game/Props/Humans"
SRC_BP = "/Engine/Tutorial/SubEditors/TutorialAssets/Character/TutorialCharacter.TutorialCharacter"
ANIM_DIR = "/Engine/Tutorial/SubEditors/TutorialAssets/Character"

POSES = [
    ("BP_Human_Walk", "%s/Tutorial_Walk_Fwd.Tutorial_Walk_Fwd" % ANIM_DIR),
    ("BP_Human_Idle", "%s/Tutorial_Idle.Tutorial_Idle" % ANIM_DIR),
]

_report = []


def log(msg):
    unreal.log("[human-bp] %s" % msg)
    _report.append(str(msg))


def make_one(tools, src, name, anim_path):
    pkg = "%s/%s" % (OUT_DIR, name)

    anim = unreal.load_asset(anim_path)
    if anim is None:
        log("FAILED: animation not found -- %s" % anim_path)
        return False

    if unreal.EditorAssetLibrary.does_asset_exist(pkg):
        unreal.EditorAssetLibrary.delete_asset(pkg)

    bp = tools.duplicate_asset(name, OUT_DIR, src)
    if bp is None:
        log("FAILED: could not duplicate into %s" % pkg)
        return False

    # The generated class is a SEPARATE object at <package>.<Name>_C; neither
    # bp.generated_class() nor get_editor_property("generated_class") exists here.
    gen = unreal.load_object(None, "%s.%s_C" % (pkg, name))
    if gen is None:
        log("FAILED: no generated class for %s" % name)
        return False
    cdo = unreal.get_default_object(gen)
    comp = cdo.get_editor_property("mesh")      # ACharacter::Mesh

    # THE POINT: pin one animation instead of running the AnimBlueprint's
    # velocity-driven blendspace, which a spawned-and-teleported actor never
    # leaves idle in.
    comp.set_editor_property("animation_mode", unreal.AnimationMode.ANIMATION_SINGLE_NODE)
    data = unreal.SingleAnimationPlayData()
    data.set_editor_property("anim_to_play", anim)
    # FSingleAnimationPlayData's flags are bSavedLooping / bSavedPlaying, so
    # Python names them saved_looping / saved_playing -- "looping" is rejected.
    for prop in ("saved_looping", "saved_playing"):
        try:
            data.set_editor_property(prop, True)
        except Exception as exc:
            log("  note: could not set %s (%s)" % (prop, exc))
    comp.set_editor_property("animation_data", data)

    unreal.EditorAssetLibrary.save_asset(pkg)
    log("built %s   pose=%s" % (pkg, anim_path.rsplit('.', 1)[-1]))
    return True


def main():
    src = unreal.load_asset(SRC_BP)
    if src is None:
        log("FAILED: source Blueprint not found -- %s" % SRC_BP)
        return
    if not unreal.EditorAssetLibrary.does_directory_exist(OUT_DIR):
        unreal.EditorAssetLibrary.make_directory(OUT_DIR)

    tools = unreal.AssetToolsHelpers.get_asset_tools()
    built = [n for n, a in POSES if make_one(tools, src, n, a)]
    log("done -- %d of %d built: %s" % (len(built), len(POSES),
                                        ", ".join(built) if built else "none"))
    log("RESTART the editor and press Play, then: py probe_env.py --grep BP_Human")


main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Examples for interacting with Omniverse Engine.
"""

'''Unused Libraries'''
# import omni.kit.commands
# from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools


def main():
    """Setup for the Nvidia examples.
    """
    # setting up import configuration:
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False

    # Get path to extension data:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = ext_manager.get_enabled_extension_id("omni.isaac.urdf")
    extension_path = ext_manager.get_extension_path(ext_id)
    # import URDF
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=extension_path + "/data/urdf/robots/carter/urdf/carter.urdf",
        import_config=import_config,
    )
    # get stage handle
    stage = omni.usd.get_context().get_stage()

    # enable physics
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    # set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    # add ground plane
    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -50), Gf.Vec3f(0.5))

    # add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)

if __name__ == '__main__':
    main()
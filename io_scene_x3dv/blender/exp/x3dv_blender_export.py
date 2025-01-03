# SPDX-License-Identifier: Apache-2.0)
# Copyright 2023 The x3dv-Blender-IO authors.
# Contributors: bart:neeneenee*de, http://www.neeneenee.de/vrml, Campbell Barton
"""
This script exports to X3D format.

Usage:
Run this script from "File->Export" menu.  A pop-up will ask whether you
want to export only selected or all relevant objects.

Known issues:
    Doesn't handle multiple materials (don't use material indices);<br>
    Doesn't handle multiple UV textures on a single mesh (create a mesh for each texture);<br>
    Can't get the texture array associated with material * not the UV ones;
"""

import time

import bpy
import sys
import traceback
from io_scene_x3dv.io.com.x3dv_io_debug import print_console, print_newline
from io_scene_x3dv.blender.com.x3dv import *
import math
import os
import mathutils
import random

from .export_motions import write_animation
from .export_interpolators import write_interpolators, write_obj_interpolators, export_TimeSensor

from bpy_extras.io_utils import create_derived_objects

from .RoundArray import round_array, round_array_no_unit_scale
from .GetSceneScale import getscenescale

from .swap_USEbeforeDEF import swap_USEbeforeDEF, USEdict, DEFdict, USEbeforeDEFdict

import itertools

from .Joints import JOINTS
from .Segments import SEGMENTS
from .Sites import SITES

class Counter:
    def __init__(self):
        self.counter = 0
    def get_id(self):
        self.counter += 1
        return str(self.counter)

counter = Counter()
depth = 0

class NameUsed:
    def __init__(self):
        self.reset()

    def lookup_center(self, DEF):
        if not self.uuid_defs[DEF].center:
            return self.uuid_defs[DEF].location
        else:
            return self.uuid_defs[DEF].center

    # copy centers to correct skeleton
    def is_used(self, DEF, node):
        if DEF in self.uuid_defs.keys() and type(node) == type(self.uuid_defs[DEF]):
            #if isinstance(node, HAnimJoint) and hasattr(node, "center") and node.center:
            #    self.uuid_defs[DEF].center = node.center
            #    print_console('INFO',f"Node uuid {DEF} center {self.uuid_defs[DEF].center}")
            if isinstance(self.uuid_defs[DEF], HAnimJoint) and hasattr( self.uuid_defs[DEF].center, "center") and self.uuid_defs[DEF].center:
                # print_console('INFO',f"Node update {node.DEF} center {node.center} copied from uuid def {self.uuid_defs[DEF].center}")
                node.center = self.uuid_defs[DEF].center
        else:
            self.uuid_defs[DEF] = node
            if isinstance(node, HAnimJoint) and hasattr(node, "center") and node.center:
                # print_console('INFO',f"Node update {node.DEF} center {node.center} copied over uuid def {self.uuid_defs[DEF].center}")
                self.uuid_defs[DEF].center = node.center

    def  reset(self):
        self.uuid_defs = {}
        self.defs = {}

    def DEF_used(self, DEF, node):
        if DEF in self.defs.keys() and type(node) == type(self.defs[DEF]):
                node.USE = self.defs[DEF].DEF
                node.DEF = None
        else:
            self.defs[DEF] = node
            node.DEF = DEF

name_used = NameUsed()

bvh2HAnim = {
"LeftUpLeg" : "l_thigh",
"LeftLeg" : "l_calf",
"LeftFoot" : "l_talus",
"LeftToe" : "l_tarsal_proximal_phalanx_2",
"RightUpLeg" : "r_thigh",
"RightLeg" : "r_calf",
"RightFoot" : "r_talus",
"RightToe" : "r_tarsal_proximal_phalanx_2",
"Hips" : "pelvis",
"Spine" : "sacrum",
"Spine1" : "l5",
"Spine2" : "t12",
"Neck" : "c7",
"Head" : "skull",
"LeftShoulder" : "l_shoulder",
"LeftArm" : "l_upperarm",
"LeftForeArm" : "l_forearm",
"LeftHand" : "l_carpal",
"RightShoulder" : "r_shoulder",
"RightArm" : "r_upperarm",
"RightForeArm" : "r_forearm",
"RightHand" : "r_carpal"
}

def substitute(subs):
    if subs in bvh2HAnim:
        # print_console('INFO',f"converting {subs} to {bvh2HAnim[subs]}")
        subs = bvh2HAnim[subs]
    else:
        pass
        # print_console('INFO',f"{subs} not found")
    return subs.replace(":", "_").replace(" ", "_").replace(".", "_")

JointsSegments = {
"humanoid_root" : "sacrum",
"sacroiliac" : "pelvis",
"Sacroiliac" : "pelvis",
"l_hip" : "l_thigh",
"l_knee" : "l_calf",
"l_talocrural" : "l_talus",
"l_talocalcaneonavicular" : "l_navicular",
"l_cuneonavicular_1" : "l_cuneiform_1",
"l_tarsometatarsal_1" : "l_metatarsal_1",
"l_metatarsophalangeal_1" : "l_tarsal_proximal_phalanx_1",
"l_tarsal_interphalangeal_1" : "l_tarsal_distal_phalanx_1",
"l_cuneonavicular_2" : "l_cuneiform_2",
"l_tarsometatarsal_2" : "l_metatarsal_2",
"l_metatarsophalangeal_2" : "l_tarsal_proximal_phalanx_2",
"l_tarsal_proximal_interphalangeal_2" : "l_tarsal_middle_phalanx_2",
"l_tarsal_distal_interphalangeal_2" : "l_tarsal_distal_phalanx_2",
"l_cuneonavicular_3" : "l_cuneiform_3",
"l_tarsometatarsal_3" : "l_metatarsal_3",
"l_metatarsophalangeal_3" : "l_tarsal_proximal_phalanx_3",
"l_tarsal_proximal_interphalangeal_3" : "l_tarsal_middle_phalanx_3",
"l_tarsal_distal_interphalangeal_3" : "l_tarsal_distal_phalanx_3",
"l_calcaneuscuboid" : "l_calcaneus",
"l_transversetarsal" : "l_cuboid",
"l_tarsometatarsal_4" : "l_metatarsal_4",
"l_metatarsophalangeal_4" : "l_tarsal_proximal_phalanx_4",
"l_tarsal_proximal_interphalangeal_4" : "l_tarsal_middle_phalanx_4",
"l_tarsal_distal_interphalangeal_4" : "l_tarsal_distal_phalanx_4",
"l_tarsometatarsal_5" : "l_metatarsal_5",
"l_metatarsophalangeal_5" : "l_tarsal_proximal_phalanx_5",
"l_tarsal_proximal_interphalangeal_5" : "l_tarsal_middle_phalanx_5",
"l_tarsal_distal_interphalangeal_5" : "l_tarsal_distal_phalanx_5",
"r_hip" : "r_thigh",
"r_knee" : "r_calf",
"r_talocrural" : "r_talus",
"r_talocalcaneonavicular" : "r_navicular",
"r_cuneonavicular_1" : "r_cuneiform_1",
"r_tarsometatarsal_1" : "r_metatarsal_1",
"r_metatarsophalangeal_1" : "r_tarsal_proximal_phalanx_1",
"r_tarsal_interphalangeal_1" : "r_tarsal_distal_phalanx_1",
"r_cuneonavicular_2" : "r_cuneiform_2",
"r_tarsometatarsal_2" : "r_metatarsal_2",
"r_metatarsophalangeal_2" : "r_tarsal_proximal_phalanx_2",
"r_tarsal_proximal_interphalangeal_2" : "r_tarsal_middle_phalanx_2",
"r_tarsal_distal_interphalangeal_2" : "r_tarsal_distal_phalanx_2",
"r_cuneonavicular_3" : "r_cuneiform_3",
"r_tarsometatarsal_3" : "r_metatarsal_3",
"r_metatarsophalangeal_3" : "r_tarsal_proximal_phalanx_3",
"r_tarsal_proximal_interphalangeal_3" : "r_tarsal_middle_phalanx_3",
"r_tarsal_distal_interphalangeal_3" : "r_tarsal_distal_phalanx_3",
"r_calcaneuscuboid" : "r_calcaneus",
"r_transversetarsal" : "r_cuboid",
"r_tarsometatarsal_4" : "r_metatarsal_4",
"r_metatarsophalangeal_4" : "r_tarsal_proximal_phalanx_4",
"r_tarsal_proximal_interphalangeal_4" : "r_tarsal_middle_phalanx_4",
"r_tarsal_distal_interphalangeal_4" : "r_tarsal_distal_phalanx_4",
"r_tarsometatarsal_5" : "r_metatarsal_5",
"r_metatarsophalangeal_5" : "r_tarsal_proximal_phalanx_5",
"r_tarsal_proximal_interphalangeal_5" : "r_tarsal_middle_phalanx_5",
"r_tarsal_distal_interphalangeal_5" : "r_tarsal_distal_phalanx_5",
"vl5" : "l5",
"vl4" : "l4",
"vl3" : "l3",
"vl2" : "l2",
"vl1" : "l1",
"vt12" : "t12",
"vt11" : "t11",
"vt10" : "t10",
"vt9" : "t9",
"vt8" : "t8",
"vt7" : "t7",
"vt6" : "t6",
"vt5" : "t5",
"vt4" : "t4",
"vt3" : "t3",
"vt2" : "t2",
"vt1" : "t1",
"vc7" : "c7",
"vc6" : "c6",
"vc5" : "c5",
"vc4" : "c4",
"vc3" : "c3",
"vc2" : "c2",
"vc1" : "c1",
"skullbase" : "skull",
"l_eyelid_joint" : "l_eyelid",
"r_eyelid_joint" : "r_eyelid",
"l_eyeball_joint" : "l_eyeball",
"r_eyeball_joint" : "r_eyeball",
"l_eyebrow_joint" : "l_eyebrow",
"r_eyebrow_joint" : "r_eyebrow",
"tongue_joint" : "tongue",
"temporomandibular" : "jaw",
"l_sternoclavicular" : "l_clavicle",
"l_acromioclavicular" : "l_scapula",
"l_shoulder" : "l_upperarm",
"l_elbow" : "l_forearm",
"l_radiocarpal" : "l_carpal",
"l_midcarpal_1" : "l_trapezium",
"l_carpometacarpal_1" : "l_metacarpal_1",
"l_metacarpophalangeal_1" : "l_carpal_proximal_phalanx_1",
"l_carpal_interphalangeal_1" : "l_carpal_distal_phalanx_1",
"l_midcarpal_2" : "l_trapezoid",
"l_carpometacarpal_2" : "l_metacarpal_2",
"l_metacarpophalangeal_2" : "l_carpal_proximal_phalanx_2",
"l_carpal_proximal_interphalangeal_2" : "l_carpal_middle_phalanx_2",
"l_carpal_distal_interphalangeal_2" : "l_carpal_distal_phalanx_2",
"l_midcarpal_3" : "l_capitate",
"l_carpometacarpal_3" : "l_metacarpal_3",
"l_metacarpophalangeal_3" : "l_carpal_proximal_phalanx_3",
"l_carpal_proximal_interphalangeal_3" : "l_carpal_middle_phalanx_3",
"l_carpal_distal_interphalangeal_3" : "l_carpal_distal_phalanx_3",
"l_midcarpal_4_5" : "l_hamate",
"l_carpometacarpal_4" : "l_metacarpal_4",
"l_metacarpophalangeal_4" : "l_carpal_proximal_phalanx_4",
"l_carpal_proximal_interphalangeal_4" : "l_carpal_middle_phalanx_4",
"l_carpal_distal_interphalangeal_4" : "l_carpal_distal_phalanx_4",
"l_carpometacarpal_5" : "l_metacarpal_5",
"l_metacarpophalangeal_5" : "l_carpal_proximal_phalanx_5",
"l_carpal_proximal_interphalangeal_5" : "l_carpal_middle_phalanx_5",
"l_carpal_distal_interphalangeal_5" : "l_carpal_distal_phalanx_5",
"r_sternoclavicular" : "r_clavicle",
"r_acromioclavicular" : "r_scapula",
"r_shoulder" : "r_upperarm",
"r_elbow" : "r_forearm",
"r_radiocarpal" : "r_carpal",
"r_midcarpal_1" : "r_trapezium",
"r_carpometacarpal_1" : "r_metacarpal_1",
"r_metacarpophalangeal_1" : "r_carpal_proximal_phalanx_1",
"r_carpal_interphalangeal_1" : "r_carpal_distal_phalanx_1",
"r_midcarpal_2" : "r_trapezoid",
"r_carpometacarpal_2" : "r_metacarpal_2",
"r_metacarpophalangeal_2" : "r_carpal_proximal_phalanx_2",
"r_carpal_proximal_interphalangeal_2" : "r_carpal_middle_phalanx_2",
"r_carpal_distal_interphalangeal_2" : "r_carpal_distal_phalanx_2",
"r_midcarpal_3" : "r_capitate",
"r_carpometacarpal_3" : "r_metacarpal_3",
"r_metacarpophalangeal_3" : "r_carpal_proximal_phalanx_3",
"r_carpal_proximal_interphalangeal_3" : "r_carpal_middle_phalanx_3",
"r_carpal_distal_interphalangeal_3" : "r_carpal_distal_phalanx_3",
"r_midcarpal_4_5" : "r_hamate",
"r_carpometacarpal_4" : "r_metacarpal_4",
"r_metacarpophalangeal_4" : "r_carpal_proximal_phalanx_4",
"r_carpal_proximal_interphalangeal_4" : "r_carpal_middle_phalanx_4",
"r_carpal_distal_interphalangeal_4" : "r_carpal_distal_phalanx_4",
"r_carpometacarpal_5" : "r_metacarpal_5",
"r_metacarpophalangeal_5" : "r_carpal_proximal_phalanx_5",
"r_carpal_proximal_interphalangeal_5" : "r_carpal_middle_phalanx_5",
"r_carpal_distal_interphalangeal_5" : "r_carpal_distal_phalanx_5",
}


# h3d defines
H3D_TOP_LEVEL = 'TOP_LEVEL_TI'
H3D_CAMERA_FOLLOW = 'CAMERA_FOLLOW_TRANSFORM'
H3D_VIEW_MATRIX = 'view_matrix'

HANIM_DEF_PREFIX = ''  # was hanim_

def clamp_color(col):
    return tuple([max(min(c, 1.0), 0.0) for c in col])


def matrix_direction_neg_z(matrix):
    return (matrix.to_3x3() @ mathutils.Vector((0.0, 0.0, -1.0))).normalized()[:]


def prefix_quoted_str(value, prefix):
    return value[0] + prefix + value[1:]


def suffix_quoted_str(value, suffix):
    return value[:-1] + suffix + value[-1:]

def prefix_str(value, prefix):
    return prefix + value


def suffix_str(value, suffix):
    return value + suffix


def bool_as_str(value):
    return ('false', 'true')[bool(value)]


def clean_def(txt):
    # see report [#28256]
    if not txt:
        txt = "None"
    # no digit start
    if txt[0] in "1234567890+-":
        txt = "_" + txt
    return txt.translate({
        # control characters 0x0-0x1f
        # 0x00: "_",
        0x01: "_",
        0x02: "_",
        0x03: "_",
        0x04: "_",
        0x05: "_",
        0x06: "_",
        0x07: "_",
        0x08: "_",
        0x09: "_",
        0x0a: "_",
        0x0b: "_",
        0x0c: "_",
        0x0d: "_",
        0x0e: "_",
        0x0f: "_",
        0x10: "_",
        0x11: "_",
        0x12: "_",
        0x13: "_",
        0x14: "_",
        0x15: "_",
        0x16: "_",
        0x17: "_",
        0x18: "_",
        0x19: "_",
        0x1a: "_",
        0x1b: "_",
        0x1c: "_",
        0x1d: "_",
        0x1e: "_",
        0x1f: "_",

        0x7f: "_",  # 127

        0x20: "_",  # space
        0x22: "_",  # "
        0x27: "_",  # '
        0x23: "_",  # #
        0x2c: "_",  # ,
        0x2e: "_",  # .
        0x5b: "_",  # [
        0x5d: "_",  # ]
        0x5c: "_",  # \
        0x7b: "_",  # {
        0x7d: "_",  # }
        })


def build_hierarchy(objects):
    """ returns parent child relationships, skipping
    """
    objects_set = set(objects)
    par_lookup = {}

    def test_parent(parent):
        while (parent is not None) and (parent not in objects_set):
            parent = parent.parent
        return parent

    for obj in objects:
        par_lookup.setdefault(test_parent(obj.parent), []).append((obj, []))

    for parent, children in par_lookup.items():
        for obj, subchildren in children:
            subchildren[:] = par_lookup.get(obj, [])

    return par_lookup.get(None, [])


# -----------------------------------------------------------------------------
# H3D Functions
# -----------------------------------------------------------------------------

def h3d_shader_glsl_frag_patch(filepath, scene, global_vars, frag_uniform_var_map):
    h3d_file = open(filepath, 'r', encoding='utf-8')
    lines = []

    last_transform = None

    for l in h3d_file:
        if l.startswith("void main(void)"):
            lines.append("\n")
            lines.append("// h3d custom vars begin\n")
            for v in global_vars:
                lines.append("%s\n" % v)
            lines.append("// h3d custom vars end\n")
            lines.append("\n")
        elif l.lstrip().startswith("light_visibility_other("):
            w = l.split(', ')
            last_transform = w[1] + "_transform"  # XXX - HACK!!!
            w[1] = '(view_matrix * %s_transform * vec4(%s.x, %s.y, %s.z, 1.0)).xyz' % (w[1], w[1], w[1], w[1])
            l = ', '.join(w)
        elif l.lstrip().startswith("light_visibility_sun_hemi("):
            w = l.split(', ')
            w[0] = w[0][len("light_visibility_sun_hemi(") + 1:]

            if not h3d_is_object_view(scene, frag_uniform_var_map[w[0]]):
                w[0] = '(mat3(normalize(view_matrix[0].xyz), normalize(view_matrix[1].xyz), normalize(view_matrix[2].xyz)) * -%s)' % w[0]
            else:
                w[0] = ('(mat3(normalize((view_matrix*%s)[0].xyz), normalize((view_matrix*%s)[1].xyz), normalize((view_matrix*%s)[2].xyz)) * -%s)' %
                        (last_transform, last_transform, last_transform, w[0]))

            l = "\tlight_visibility_sun_hemi(" + ", ".join(w)
        elif l.lstrip().startswith("light_visibility_spot_circle("):
            w = l.split(', ')
            w[0] = w[0][len("light_visibility_spot_circle(") + 1:]

            if not h3d_is_object_view(scene, frag_uniform_var_map[w[0]]):
                w[0] = '(mat3(normalize(view_matrix[0].xyz), normalize(view_matrix[1].xyz), normalize(view_matrix[2].xyz)) * -%s)' % w[0]
            else:
                w[0] = ('(mat3(normalize((view_matrix*%s)[0].xyz), normalize((view_matrix*%s)[1].xyz), normalize((view_matrix*%s)[2].xyz)) * %s)' %
                    (last_transform, last_transform, last_transform, w[0]))

            l = "\tlight_visibility_spot_circle(" + ", ".join(w)

        lines.append(l)

    h3d_file.close()

    h3d_file = open(filepath, 'w', encoding='utf-8')
    h3d_file.writelines(lines)
    h3d_file.close()

def h3d_is_object_view(scene, obj):
    camera = scene.camera
    parent = obj.parent
    while parent:
        if parent == camera:
            return True
        parent = parent.parent
    return False

# -----------------------------------------------------------------------------
# Functions for writing output file
# -----------------------------------------------------------------------------
def export(context, x3dv_export_settings):
    """
    file,
    global_matrix,
    depsgraph,
    scene,
    view_layer,
    use_mesh_modifiers=False,
    use_selection=True,
    use_triangulate=False,
    use_normals=False,
    use_hierarchy=True,
    use_h3d=False,
    path_mode='AUTO',
    prefix='',
    name_decorations=True,
    round_precision=16
    ):
    """
    export_settings = x3dv_export_settings
    scene = context.scene
    if not scene:
        scene = bpy.context.scene
    view_layer = context.view_layer
    depsgraph = context.evaluated_depsgraph_get()
    global_matrix = mathutils.Matrix()
    use_h3d = False

    # -------------------------------------------------------------------------
    # Global Setup
    # -------------------------------------------------------------------------
    import bpy_extras
    from bpy_extras.io_utils import unique_name
    from xml.sax.saxutils import quoteattr, escape

    HANIM_DEF_PREFIX = ''
    if export_settings['x3dv_hanim_prefix']:
        HANIM_DEF_PREFIX = export_settings['x3dv_hanim_prefix']

    if export_settings['x3dv_name_decorations']:
        # If names are decorated, the uuid map can be split up
        # by type for efficiency of collision testing
        # since objects of different types will always have
        # different decorated names.
        uuid_cache_object = {}    # object
        uuid_cache_light = {}     # 'LA_' + object.name
        uuid_cache_view = {}      # object, different namespace
        uuid_cache_mesh = {}      # mesh
        uuid_cache_shape = {}     # mesh
        uuid_cache_material = {}  # material
        uuid_cache_texcoords = {} # texture coordinates
        uuid_cache_image = {}     # image
        uuid_cache_world = {}     # world
        CA_ = 'CA_'
        OB_ = 'OB_'
        ME_ = 'ME_'
        SH_ = 'SH_'
        IM_ = 'IM_'
        WO_ = 'WO_'
        MA_ = 'MA_'
        LA_ = 'LA_'
        TX_ = 'TX_'
        TE_ = 'TE_'
        HA_ = 'HA_'
        CO_ = 'CO_'
        NO_ = 'NO_'
        group_ = 'group_'
        LS_ = 'LS_'
        COLOR_ = 'COLOR_'
        FS_ = 'FS_'
    else:
        # If names are not decorated, it may be possible for two objects to
        # have the same name, so there has to be a unified dictionary to
        # prevent uuid collisions.
        uuid_cache = {}
        uuid_cache_object = uuid_cache           # object
        uuid_cache_light = uuid_cache            # 'LA_' + object.name
        uuid_cache_view = uuid_cache             # object, different namespace
        uuid_cache_mesh = uuid_cache             # mesh
        uuid_cache_shape = uuid_cache            # shape
        uuid_cache_material = uuid_cache         # material
        uuid_cache_texcoords = {}                # texture coordinates
        uuid_cache_image = uuid_cache            # image
        uuid_cache_world = uuid_cache            # world
        del uuid_cache
        CA_ = ''
        OB_ = ''
        ME_ = ''
        SH_ = ''
        IM_ = ''
        WO_ = ''
        MA_ = ''
        LA_ = ''
        TX_ = ''
        TE_ = ''
        HA_ = ''
        CO_ = ''
        NO_ = ''
        group_ = ''
        LS_ = ''
        COLOR_ = ''
        FS_ = ''

    _TRANSFORM = '_TRANSFORM'

    def setUSEDEF(prefix, name, node, x3d_oid=None):
        if name is None:
            name = ""
        if name.startswith(prefix):
            name = name[len(prefix):]
        if node is not None:
            node.name = substitute(name)
            pname = prefix+substitute(name)
            if x3d_oid is not None and x3d_oid > 0:
                node.USE = pname
            else:
                name_used.DEF_used(pname, node)
            name_used.is_used(pname, node)
        else:
            raise f"Null node";
        return node

    # store files to copy
    copy_set = set()

    # store names of newly created meshes, so we dont overlap
    mesh_name_set = set()

    # fw = file.write
    base_src = export_settings['x3dv_blender_directory']  # os.path.dirname(bpy.data.filepath)
    base_dst = export_settings['x3dv_filedirectory']  # os.path.dirname(file.name)
    # filename_strip = os.path.splitext(os.path.basename(file.name))[0]
    gpu_shader_cache = {}

    #if use_h3d:
    #    import gpu
    #    gpu_shader_dummy_mat = bpy.data.materials.new('X3D_DYMMY_MAT')
    #    gpu_shader_cache[None] = gpu.export_shader(scene, gpu_shader_dummy_mat)
    #    h3d_material_route = []

    # -------------------------------------------------------------------------
    # File Writing Functions
    # -------------------------------------------------------------------------

    def b2xHeader():
        filepath = os.path.basename(export_settings['x3dv_filepath'])
        blender_ver = 'Blender %s' % bpy.app.version_string
        copyright = export_settings['x3dv_copyright']
        if copyright is None:
            copyright = "2024"
        else:
            copyright = "2024"
        hd = head()
        conversionFactor=getscenescale(context.scene)
    
        hd.children.append(component(name='HAnim', level=3))
        if conversionFactor != 1:
              hd.children.append(unit(category='length', conversionFactor=conversionFactor, name=scene.unit_settings.length_unit))
        hd.children.append(meta(content=filepath,name='title'))
        hd.children.append(meta(content=copyright,name='copyright'))
        hd.children.append(meta(content='https://github.com/Web3DConsortium/BlenderX3DSupport (offcial release)',name='reference'))
        hd.children.append(meta(content=blender_ver,name='generator'))
        hd.children.append(meta(content='https://github.com/coderextreme/BlenderX3DSupport/tree/main/io_scene_x3dv (experiemental release)',name='exporter'))

        # looks interesting but wrong place and not in X3D specs v.4
        #if use_h3d:
        #    # outputs the view matrix in glModelViewMatrix field
        #    fw('%s<TransformInfo DEF="%s" outputGLMatrices="true" />\n' % (ident, H3D_TOP_LEVEL))

        return hd

    def b2xFooter():

        #if use_h3d:
        #    # global
        #    for route in h3d_material_route:
        #        fw('%s%s\n' % (ident, route))

        return None

    def b2xViewpoint(obj, matrix, x3d_obj):
        setUSEDEF(CA_, obj.name, x3d_obj)
        loc, rot, scale = matrix.decompose()
        rot = rot.to_axis_angle()
        rot = (*rot[0].normalized(), rot[1])

        # print_console('INFO',f"Camera location {loc[:]}")
        x3d_obj.position = round_array(loc[:])
        x3d_obj.orientation = round_array_no_unit_scale(rot)
        x3d_obj.fieldOfView = obj.data.angle
        return x3d_obj

    def b2xFog(world):
        if world:
            mtype = world.mist_settings.falloff
            mparam = world.mist_settings
        else:
            return None
        fog = Fog()
        if mparam.use_mist:
            fog.fogType = 'LINEAR' if (mtype == 'LINEAR') else 'EXPONENTIAL'
            fog.color = clamp_color(world.horizon_color)
            fog.visibilityRange = mparam.depth
            return fog
        else:
            return None

    def b2xNavigationInfo(has_light, x3d_obj):
        x3d_obj.type = ["EXAMINE", "ANY"]
        x3d_obj.headlight = has_light
        x3d_obj.visibilityLimit = 0.0
        # default x3d_obj.type = ["EXAMINE", "ANY"]
        # default x3d_obj.avatarSize = [0.25, 1.6, 0.75]
        return x3d_obj

    def b2xText(text_obj, x3d_obj):
        size = round((text_obj.scale[0] + text_obj.scale[1] + text_obj.scale[2])/3, 5)
        setUSEDEF(TE_, text_obj.data.name, x3d_obj)
        x3d_obj.string=text_obj.data.body.split("\n")
        x3d_obj.fontStyle=FontStyle(size=size, justify=["MIDDLE","MIDDLE"])
        return x3d_obj

    def b2xInstantiate(x3d_class, x3d_oid, x3d_name, obj_main=None, x3d_matrix=None):
        try:
            if x3d_oid == 0:
                x3d_obj = eval(x3d_class)(DEF=x3d_name)
            else:
                x3d_obj = eval(x3d_class)(USE=x3d_name)
        except NameError:
                x3d_obj = None
        except:
                if len(x3d_name) <= 0:
                    x3d_obj = eval(x3d_class)()
                else:
                    x3d_obj = eval(x3d_class)(DEF=x3d_name)
        # print_console('INFO',f"Class {x3d_class} instance #{x3d_oid} DEF/USE={x3d_name} {x3d_obj}")
        return x3d_obj

    def b2xTransform(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix, x3d_obj):
        loc, rot, sca = x3d_matrix.decompose()
        rot = rot.to_axis_angle()
        rot = (*rot[0], rot[1])

        x3d_obj.translation = round_array(loc[:])
        x3d_obj.scale = round_array_no_unit_scale(sca[:])
        x3d_obj.rotation = round_array_no_unit_scale(rot)
        return x3d_obj


    def b2xSpotLight(obj, matrix, light, world, lite):
        # note, light_id is not re-used
        setUSEDEF(LA_, obj.name, lite)

        if world and 0:
            ambi = world.ambient_color
            amb_intensity = ((ambi[0] + ambi[1] + ambi[2]) / 3.0) / 2.5
            del ambi
        else:
            amb_intensity = 0.0

        # compute cutoff and beamwidth
        lamp = obj.data  # added by John
        intensity = min(lamp.energy / 1.75, 1.0)
        beamWidth = lamp.spot_size * 0.37
        # beamWidth=((lamp.spotSize*math.pi)/180.0)*.37
        cutOffAngle = beamWidth * 1.3

        orientation = matrix_direction_neg_z(matrix)

        location = round_array(matrix.to_translation()[:])

        radius = lamp.cutoff_distance * math.cos(beamWidth)
        lite.radius = radius
        lite.ambientIntensity = amb_intensity
        lite.intensity = intensity
        lite.color = clamp_color(light.color)
        lite.beamWidth= beamWidth
        lite.cutOffAngle = cutOffAngle
        lite.direction = orientation
        lite.location = location
        
        return lite

    def b2xDirectionalLight(obj, matrix, light, world, lite):
        # note, light_id is not re-used
        setUSEDEF(LA_, obj.name, lite)

        if world and 0:
            ambi = world.ambient_color
            # ambi = world.amb
            amb_intensity = ((float(ambi[0] + ambi[1] + ambi[2])) / 3.0) / 2.5
        else:
            ambi = 0
            amb_intensity = 0.07 #was 0, turned it on a bit

        intensity = min(light.energy / 1.75, 1.0)

        # orientation = matrix_direction_neg_z(matrix)
        loc, orientation, sca = matrix.decompose()
        orientation = orientation.to_axis_angle()
        orientation = (*orientation[0], orientation[1])


        lite.ambientIntensity = amb_intensity
        lite.intensity = intensity
        lite.on = True
        lite.global_ = True #the way we export wrapping in a transform means to get light we need it global
        lite.color = clamp_color(light.color)
        lite.direction = orientation[:-1]
        # print_console('INFO',f"Orientation {orientation}")
        return lite

    def b2xPointLight( obj, matrix, light, world, lite):
        # note, light_id is not re-used
        setUSEDEF(LA_, obj.name, lite)

        if world and 0:
            ambi = world.ambient_color
            # ambi = world.amb
            amb_intensity = ((float(ambi[0] + ambi[1] + ambi[2])) / 3.0) / 2.5
        else:
            ambi = 0.0
            amb_intensity = 0.0

        intensity = min(light.energy / 1.75, 1.0)
        location = round_array(matrix.to_translation()[:])

        lite.radius = light.cutoff_distance
        lite.ambientIntensity = amb_intensity
        lite.intensity = intensity
        lite.color = clamp_color(light.color)
        lite.location = location
        
        return lite

    def b2xHAnimNode(obj, matrix, name, tag, x3d_obj, segment_name=None, skinCoordIndex=None, skinCoordWeight=None, motions=None):
        if (name == "root" or name == "humanoid_root") and tag == "HAnimHumanoid":
            name = "humanoid"
        setUSEDEF(HANIM_DEF_PREFIX, name, x3d_obj)

        if tag == "HAnimJoint":  # For blender bones
            #print_console('INFO',f"Exporting joint {tag} {obj.name}")
            try:
                obj_matrix = obj.matrix_local
                if obj_matrix is not None:
                    loc, rot, sca = obj_matrix.decompose()
                    rot = rot.to_axis_angle()
                    rot = (*rot[0], rot[1])
                #center = matrix.to_translation()[:]
                center = obj.head_local
                chicenter = obj.tail_local
            except AttributeError:
                center = [0, 0, 0]
                chicenter = [0, 0, 0]
                print(f"No such attribute")
                pass
            # eul = mathutils.Euler((math.radians(90.0), 0.0, 0.0), 'XZY')
            # eul2 = mathutils.Euler((0.0, 0.0, math.radians(180.0)), 'XZY')

            vec = mathutils.Vector((center[0], center[1], center[2]))  # in Blender space
            #vec.rotate(eul)
            # vec.rotate(eul2)
            center = round_array(vec)  # in X3D space

            chivec = mathutils.Vector((chicenter[0], chicenter[1], chicenter[2]))  # in Blender space
            #chivec.rotate(eul)
            chicenter = round_array(chivec)  # in X3D space
            #print_console('INFO',f"center {center[:]}")
            x3d_obj.center=center[:]
            if not x3d_obj.center:
              x3d_obj.center = obj_matrix.to_translation()[:]
            if skinCoordIndex is not None:
               x3d_obj.skinCoordIndex = skinCoordIndex
            if skinCoordWeight is not None:
               x3d_obj.skinCoordWeight = round_array_no_unit_scale(skinCoordWeight)
            # print_console('INFO',f"Exporting type {tag} {name} DEF={x3d_obj.DEF} name={x3d_obj.name} USE={x3d_obj.USE}")
        elif tag == "HAnimJoint2": # for Blender empties
            obj_matrix = obj.matrix_local
            if obj_matrix is not None:
                loc, rot, sca = obj_matrix.decompose()
                rot = rot.to_axis_angle()
                rot = (*rot[0], rot[1])
            x3d_obj.center=round_array_no_unit_scale(loc[:]),
            x3d_obj.children=[]

            if skinCoordIndex is not None:
                x3d_obj.skinCoordIndex = skinCoordIndex
            if skinCoordWeight is not None:
                x3d_obj.skinCoordWeight = round_array_no_unit_scale(skinCoordWeight)
            if not x3d_obj.center:
                x3d_obj.center = loc[:]
              # print_console('INFO',f"Exporting2 type {tag} {name} DEF={x3d_obj.DEF} name={x3d_obj.name} USE={x3d_obj.USE} {x3d_obj.center}")
        elif tag == "HAnimSegment":
            # HAnimSegment is not a Transform
            #print_console('INFO',f"Exporting type {tag} {obj.type}")
            #obj_matrix = obj.matrix_local
            #if obj_matrix is not None:
            #    loc, rot, sca = obj_matrix.decompose()
            #    rot = rot.to_axis_angle()
            #    rot = (*rot[0], rot[1])
            x3d_obj.children=[]
            # print_console('INFO',f"Exporting type {tag} {name} DEF={x3d_obj.DEF} name={x3d_obj.name} USE={x3d_obj.USE}")
        elif tag == "HAnimSite":
            #print_console('INFO',f"Exporting type {tag} {obj.type}")
            obj_matrix = obj.matrix_world
            if obj_matrix is not None:
                loc, rot, sca = obj_matrix.decompose()
                rot = rot.to_axis_angle()
                rot = (*rot[0], rot[1])
            x3d_obj.translation=round_array(loc[:]),
            x3d_obj.children=[]
            # print_console('INFO',f"Exporting type {tag} {name} DEF={x3d_obj.DEF} name={x3d_obj.name} USE={x3d_obj.USE}")
        elif tag == "HAnimHumanoid":
            #print_console('INFO',f"Exporting type {tag} {obj.type}")
            if motions and motions[0] is not None:
                #print_console('INFO',f"{motions}")
                x3d_obj.motions=motions
            # print_console('INFO',f"Exporting type {tag} {name} DEF={x3d_obj.DEF} name={x3d_obj.name} USE={x3d_obj.USE}")
        elif tag == "HAnimInterpolators":
            #print_console('INFO',f"Exporting rest interpolators of {tag} {obj.type}")
            children = []
            if obj.type == 'ARMATURE':
                armature = obj
                view_layer.objects.active = armature
                try:
                    bpy.ops.object.mode_set(mode='POSE')
                except:
                    pass
                children = write_interpolators(obj, name, "")
                print_console('INFO',f"received {len(children)} interpolators in HAnimInterpolators")
            return children
        elif tag == "HAnimMotion":
            #print_console('INFO',f"Exporting bvh of {tag} {obj.type}")
            if obj.type == 'ARMATURE':
                armature = obj
                view_layer.objects.active = armature
                try:
                  bpy.ops.object.mode_set(mode='POSE')
                except:
                  pass
                x3d_obj = write_animation(obj)
        else:
            print(f"WARNING:  Unknown HAnim tag {tag} {x3d_obj.DEF}")
        return x3d_obj

    def b2xJoint(joint_parent, joint, matrix, joint_lookup, segment_lookup, armature, skinCoordInfo, displacerCoordInfo):
        # print_console('INFO',f"joint {joint}")
        skinCoordWeight = []
        skinCoordIndex = []
        try:
            if skinCoordInfo[joint.name]:
                skinCoordIndex = skinCoordInfo[joint.name]['indices']
                skinCoordWeight = skinCoordInfo[joint.name]['weights']
        except:
                skinCoordWeight = []

        if joint.name in segment_lookup:
            segment_name = segment_lookup[joint.name]
        else:
            segment_name = "SEGMENT_FOR_"+substitute(joint.name)

        x3d_obj = HAnimJoint()
        # These are bones
        node = b2xHAnimNode(joint, matrix, joint.name, "HAnimJoint", x3d_obj, segment_name=segment_name, skinCoordIndex=skinCoordIndex, skinCoordWeight=skinCoordWeight)

        for joint_child in joint_lookup[joint.name]['joint_children']:
            child = b2xJoint(joint, joint_child.joint, joint_child.joint.matrix_local, joint_lookup, segment_lookup, armature, skinCoordInfo, displacerCoordInfo)
            node.children.append(child)
        return node

    class HAnimNode:
        def __init__(self, name, parent_name, joint, joint_lookup):
            self.name = name
            self.parent_name = parent_name
            self.joint = joint
            joint_lookup[joint.name] = {}
            joint_lookup[joint.name]['joint'] = joint
            joint_lookup[joint.name]['joint_children'] = []
            if parent_name:
                try:
                    joint_lookup[parent_name]['joint_children'].append(self)
                except:
                    joint_lookup[parent_name]['joint_children'].append(self)

    def b2xFindSkinCoordPoint(x3dnode):
        point = None
        children = None
        typestr = type(x3dnode)
        # print_console('INFO', f"type {typestr}")
        typename = str(typestr)
        lastqut = typename.rfind("'")
        lastdot = typename.rfind(".")+1
        acttype = typename[lastdot:lastqut]

        # print_console('INFO', f"name {typename}")
        # print_console('INFO', f"actual type {acttype}")
        if   acttype in [ "Group", "Transform", "ImageTexture", "Appearance" ]:
            children = x3dnode.children
        elif acttype in [ "Shape" ]:
            children = [x3dnode.geometry]
        elif acttype in [ "IndexedFaceSet", "LineSet", "IndexedLineSet", "IndexedTriangleFanSet", "IndexedTriangleSet", "IndexedTriangleStripSet", "PointSet", "TriangleFanSet", "TriangleSet", "TriangleStripSet" ]:
            children = [x3dnode.coord]
        elif acttype in [ "Coordinate" ]:
                # print_console('INFO', f"Setting points {point} + {x3dnode.point}")
                # think about concatenating
                if point is None:
                    point = x3dnode.point
        if children is not None:
            for child in children:
                # think about concatenating
                if point is None:
                    point = b2xFindSkinCoordPoint(child)
        #print_console('INFO', f"Result point {point}")
        return point

    def b2xDEFedCoordinates(x3dnode):
        DEFnodes = []
        children = None
        typestr = type(x3dnode)
        #print_console('INFO', f"type {typestr}")
        typename = str(typestr)
        lastqut = typename.rfind("'")
        lastdot = typename.rfind(".")+1
        acttype = typename[lastdot:lastqut]

        #print_console('INFO', f"name {typename}")
        # print_console('INFO', f"actual type {acttype}")
        if   acttype in [ "Group", "Transform", "ImageTexture", "Appearance" ]:
            children = x3dnode.children
        elif acttype in [ "Shape" ]:
            children = [x3dnode.geometry]
        elif acttype in [ "IndexedFaceSet", "LineSet", "IndexedLineSet", "IndexedTriangleFanSet", "IndexedTriangleSet", "IndexedTriangleStripSet", "PointSet", "TriangleFanSet", "TriangleSet", "TriangleStripSet" ]:
            children = [x3dnode.coord]
        elif acttype in [ "Coordinate" ]:
            #print_console('INFO', f"coordinate {x3dnode}")
            if (x3dnode.DEF):
                DEFnodes = DEFnodes + [x3dnode]
            #print_console('INFO', f"DEFnodes coordinate {DEFnodes}")
        if children is not None:
            for child in children:
                #print_console('INFO', f"Summing nodes {DEFnodes}")
                DEFnodes = DEFnodes + b2xDEFedCoordinates(child)
        return DEFnodes

    def b2xArmature(obj, obj_main, obj_children, obj_matrix, data, world, humanoid):
        armature = obj
        armature_matrix = obj_matrix
        # print_console('INFO', f"Called b2xArmature, exporting {armature.name}, object type {armature.type}")
        segment_lookup = JointsSegments

        # each armature needs their own hierarchy
        joint_lookup = {}
        view_layer.objects.active = armature
        # armature.select_set(True)
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except:
            pass
        #armature_id = quoteattr(HANIM_DEF_PREFIX+armature.parent.name)
        motions = [b2xHAnimNode(armature, armature_matrix, "motions", "HAnimMotion", humanoid)]
        # comment the below out if you want mostions
        motions = None
        humanoid = b2xHAnimNode(armature, armature_matrix, "humanoid", "HAnimHumanoid", humanoid, motions=motions)
        HAnimNode(armature.name, None, armature, joint_lookup)  # populates joint_lookup
        if armature and armature.data and armature.data.bones:
            for joint in armature.data.bones:
                if joint.parent:
                    joint_parent_name = joint.parent.name
                else:
                    joint_parent_name = armature.name
                HAnimNode(joint.name, joint_parent_name, joint, joint_lookup) # populates joint_lookup

            armature_bones = {bone.name for bone in armature.data.bones}
        elif armature:
            def walk_tree(parent_joint, result=None):
                if result is None:
                    result = []
                if hasattr(parent_joint, 'children'):
                    for child_joint in parent_joint.children:
                        if child_joint.name.startswith("HAnimJoint"):
                            result.append(child_joint)
                            HAnimNode(child_joint.name, parent_joint.name, child_joint, joint_lookup) # populates joint_lookup
                            walk_tree(child_joint, result)

                return result
            armature_bones = {joint.name for joint in walk_tree(armature)}
        else:
            armature_bones = []
            print(f"WARNING: No Humanoid")
            
        # get the mapping from group indices to bones
        skinCoordInfo = {}
        displacerCoordInfo = {}
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                if obj.parent == armature:
                    group_to_bone = {i: group.name for i, group in enumerate(obj.vertex_groups)}
        
                    # determine the bone weights associated with each vertex
                    mesh = obj.data
                    for vertex in mesh.vertices: ##in each vertex of the mesh
                        #print_console('INFO','Vertex', vertex.index)
                        for group in vertex.groups: #first loop to calculate the total weight and the space allowed if some are locked
                            group_index = group.group
                            group_bone = group_to_bone[group_index]
                            if group_bone in armature_bones: ##if it's a bone
                                #print_console('INFO','\t', group_bone, group.weight)
                                try:
                                    if skinCoordInfo[group_bone] is None:
                                        skinCoordInfo = {**skinCoordInfo, group_bone : { 'indices' : [], 'weights': []}}
                                except:
                                        skinCoordInfo = {**skinCoordInfo, group_bone : { 'indices' : [], 'weights': []}}
                                skinCoordInfo[group_bone]['indices'].append(vertex.index)
                                skinCoordInfo[group_bone]['weights'].append(group.weight)
                            else:
                                print_console('INFO', f"\tNOT A JOINT, Trying displacer: {group_bone}, {group.weight}")
                                try:
                                    if displacerCoordInfo[group_bone] is None:
                                        displacerCoordInfo = {**displacerCoordInfo, group_bone : { 'indices' : [], 'weights': []}}
                                except:
                                        displacerCoordInfo = {**displacerCoordInfo, group_bone : { 'indices' : [], 'weights': []}}
                                displacerCoordInfo[group_bone]['indices'].append(vertex.index)
                                displacerCoordInfo[group_bone]['weights'].append(group.weight)
        if humanoid is not None:
            humanoid.skeleton = [b2xJoint(obj_main, armature, armature_matrix, joint_lookup, segment_lookup, armature, skinCoordInfo, displacerCoordInfo)]
            # joints should be printed after skeleton in x3d.py. That's why I've picked out skeleton in x3d.py
            humanoid.skeleton[0].name = "humanoid_root"
            humanoid.skeleton[0].DEF = HANIM_DEF_PREFIX+"humanoid_root"
            prefix = HANIM_DEF_PREFIX
            for joint_name in armature_bones:
                if joint_name.startswith(prefix):
                    joint_name = joint_name[len(prefix):]
                pname = prefix+substitute(joint_name)
                node = HAnimJoint(USE=pname)
                humanoid.joints.append(node)
            scale = 1 # getscenescale(scene)
            unit_settings = scene.unit_settings
            length_unit = scene.unit_settings.length_unit 
            # print_console('INFO', f"scene scale is {scale} {unit_settings} {length_unit}")
        else:
            print(f"humanoid is None")
        return humanoid

    def image_get(mat):
        from bpy_extras import node_shader_utils
        if mat.use_nodes:
            mat_wrap = node_shader_utils.PrincipledBSDFWrapper(mat)
            base_color_tex = mat_wrap.base_color_texture
            if base_color_tex and base_color_tex.image:
                return base_color_tex.image
        return None

    def find_texture_nodes_from_material(mtrl):
        nodes = []
        if not mtrl.node_tree:
            return nodes
        for node in mtrl.node_tree.nodes:
            tex_node_types = ["IMAGE", "TEX_IMAGE", "TEX_ENVIRONMENT", "TEXTURE"]
            if node.type not in tex_node_types:
                continue
            if not node.image:
                continue
            nodes.append(node)

        return nodes

    def b2xCoordinate(DEF=None, USE=None, point=None):
        coord = Coordinate(DEF=DEF, USE=USE, point=point)
        return coord

    def b2xIndexedFaceSet(obj, mesh, mesh_name, matrix, world, image_textures, shape, x3d_obj):
        setUSEDEF(FS_, mesh_name, x3d_obj)
        print(f"shape is {shape} obj is {x3d_obj}")
        global counter
        obj_id = unique_name(obj, OB_ + obj.name, uuid_cache_object, clean_func=clean_def, sep="_")
        mesh_id = unique_name(mesh, ME_ + mesh_name, uuid_cache_mesh, clean_func=clean_def, sep="_")
        mesh_id_group = prefix_str(mesh_id, group_)
        mesh_id_coords = prefix_str(mesh_id, 'coords_')
        mesh_id_normals = prefix_str(mesh_id, 'normals_')

        # Be sure tessellated loop triangles are available!
        if export_settings['x3dv_use_triangulate']:
            if not mesh.loop_triangles and mesh.polygons:
                mesh.calc_loop_triangles()

        nodes = []
        use_collnode = bool([mod for mod in obj.modifiers
                             if mod.type == 'COLLISION'
                             if mod.show_viewport])

        if use_collnode:
            coll = Collision(enabled='True')
        else:
            coll = None

        # use _ifs_TRANSFORM suffix so we dont collide with transform node when
        # hierarchys are used.
        # trans = b2xTransform(matrix, suffix_str(obj_id, "_ifs" + _TRANSFORM))
        if mesh.tag:
            pass
        else:
            mesh.tag = True
            is_uv = bool(mesh.uv_layers.active)
            # is_col, defined for each material

            is_coords_written = False

            mesh_materials = mesh.materials[:]
            if not mesh_materials:
                mesh_materials = [None]

            mesh_material_tex = [None] * len(mesh_materials)
            mesh_material_mtex = [None] * len(mesh_materials)
            mesh_material_images = [None] * len(mesh_materials)

            for i, material in enumerate(mesh_materials):
                if material and 'texture_slots' in material:
                    for mtex in material.texture_slots:
                        if mtex:
                            tex = mtex.texture
                            if tex and tex.type in ["IMAGE", "TEX_IMAGE", "TEX_ENVIRONMENT", "TEXTURE"]:
                                image = tex.image
                                if image:
                                    #print_console('INFO',f"Image in Texture {image} use_h3d {use_h3d}")
                                    mesh_material_tex[i] = tex
                                    mesh_material_mtex[i] = mtex
                                    mesh_material_images[i] = image
                                    break
                elif material:
                    mesh_material_images[0] = image_get(material)

            # fast access!
            mesh_vertices = mesh.vertices[:]
            mesh_loops = mesh.loops[:]
            mesh_polygons = mesh.polygons[:]
            mesh_polygons_materials = [p.material_index for p in mesh_polygons]
            mesh_polygons_vertices = [p.vertices[:] for p in mesh_polygons]

            if len(set(mesh_material_images)) > 0:  # make sure there is at least one image
                mesh_polygons_image = [mesh_material_images[material_index] for material_index in mesh_polygons_materials]
            else:
                mesh_polygons_image = [None] * len(mesh_polygons)
            mesh_polygons_image_unique = set(mesh_polygons_image)

            # group faces
            polygons_groups = {}
            for material_index in range(len(mesh_materials)):
                for image in mesh_polygons_image_unique:
                    polygons_groups[material_index, image] = []
            del mesh_polygons_image_unique

            for i, (material_index, image) in enumerate(zip(mesh_polygons_materials, mesh_polygons_image)):
                polygons_groups[material_index, image].append(i)

            # Py dict are sorted now, so we can use directly polygons_groups.items()
            # and still get consistent reproducible outputs.

            is_col = mesh.vertex_colors.active
            mesh_loops_col = mesh.vertex_colors.active.data if is_col else None

            # Check if vertex colors can be exported in per-vertex mode.
            # Do we have just one color per vertex in every face that uses the vertex?
            if is_col:
                def calc_vertex_color():
                    vert_color = [None] * len(mesh.vertices)

                    for i, p in enumerate(mesh_polygons):
                        for lidx in p.loop_indices:
                            l = mesh_loops[lidx]
                            if vert_color[l.vertex_index] is None:
                                vert_color[l.vertex_index] = mesh_loops_col[lidx].color[:]
                            elif vert_color[l.vertex_index] != mesh_loops_col[lidx].color[:]:
                                return False, ()

                    return True, vert_color

                is_col_per_vertex, vert_color = calc_vertex_color()
                del calc_vertex_color

            # If using looptris, we need a mapping poly_index -> loop_tris_indices...
            if export_settings['x3dv_use_triangulate']: 
                polygons_to_loop_triangles_indices = [[] for  i in range(len(mesh_polygons))]
                for ltri in mesh.loop_triangles:
                    polygons_to_loop_triangles_indices[ltri.polygon_index].append(ltri)

            setUSEDEF(SH_, mesh_name, shape)
            for (material_index, image), polygons_group in polygons_groups.items():
                if polygons_group:
                    material = mesh_materials[material_index]

                    is_smooth = False

                    # kludge but as good as it gets!
                    for i in polygons_group:
                        if mesh_polygons[i].use_smooth:
                            is_smooth = True
                            break

                    # UV's and VCols split verts off which effects smoothing
                    # force writing normals in this case.
                    # Also, creaseAngle is not supported for IndexedTriangleSet,
                    # so write normals when is_smooth (otherwise
                    # IndexedTriangleSet can have only all smooth/all flat shading).
                    is_force_normals = export_settings['x3dv_use_triangulate'] and (is_smooth or is_uv or is_col)

                    imt = b2xImageTexture(image, obj, mesh_name, image_textures, None)
                    if imt.url:
                        shape.appearance.texture = MultiTexture()
                        shape.appearance.texture.texture.append(imt)

                    tt = b2xTextureTransform(mesh_material_mtex[material_index], mesh_material_tex[material_index])
                    if tt:
                        shape.appearance.textureTransform = MultiTextureTransform()
                        shape.appearance.textureTransform.textureTransform.append(tt)

                    if material is not None:
                        mat = b2xMaterial(material, world)
                        shape.appearance.material = mat
                    else:
                        print(f"OUCH, Material None for Face Set {x3d_obj}")

                    #/Appearance

                    mesh_loops_uv = mesh.uv_layers.active.uv if is_uv else None

                    #-- IndexedFaceSet or IndexedLineSet
                    if export_settings['x3dv_use_triangulate']:
                        print(f"shape is {shape}")
                        its = x3d_obj
                        # --- Write IndexedTriangleSet Attributes (same as IndexedFaceSet)
                        its.solid = material and material.use_backface_culling
                        if export_settings['x3dv_normals'] or is_force_normals:
                            its.normalPerVertex = True
                        else:
                            # Tell X3D browser to generate flat (per-face) normals
                            its.normalPerVertex = False


                        slot_uv = None
                        slot_col = None
                        def _tuple_from_rounded_iter(it):
                            return tuple(round(v, 5) for v in it)

                        if is_uv and is_col:
                            slot_uv = 0
                            slot_col = 1

                            def vertex_key(lidx):
                                return (
                                    _tuple_from_rounded_iter(mesh_loops_uv[lidx]),
                                    _tuple_from_rounded_iter(mesh_loops_col[lidx].color),
                                )
                        elif is_uv:
                            slot_uv = 0

                            def vertex_key(lidx):
                                return (
                                    _tuple_from_rounded_iter(mesh_loops_uv[lidx]),
                                )
                        elif is_col:
                            slot_col = 0

                            def vertex_key(lidx):
                                return (
                                    _tuple_from_rounded_iter(mesh_loops_col[lidx].color),
                                )
                        else:
                            # ack, not especially efficient in this case
                            def vertex_key(lidx):
                                return None

                        # build a mesh mapping dict
                        vertex_hash = [{} for i in range(len(mesh.vertices))]
                        face_tri_list = [[None, None, None] for i in range(len(mesh.loop_triangles))]
                        vert_tri_list = []
                        totvert = 0
                        totface = 0
                        temp_tri = [None] * 3
                        for pidx in polygons_group:
                            for ltri in polygons_to_loop_triangles_indices[pidx]:
                                for tri_vidx, (lidx, vidx) in enumerate(zip(ltri.loops, ltri.vertices)):
                                    key = vertex_key(lidx)
                                    vh = vertex_hash[vidx]
                                    x3d_v = vh.get(key)
                                    if x3d_v is None:
                                        x3d_v = key, vidx, totvert
                                        vh[key] = x3d_v
                                        # key / original_vertex / new_vertex
                                        vert_tri_list.append(x3d_v)
                                        totvert += 1
                                    temp_tri[tri_vidx] = x3d_v

                                face_tri_list[totface][:] = temp_tri[:]
                                totface += 1

                        del vertex_key
                        del _tuple_from_rounded_iter
                        assert(len(face_tri_list) == len(mesh.loop_triangles))

                        for x3d_f in face_tri_list:
                            its.index.append((x3d_f[0][2], x3d_f[1][2], x3d_f[2][2]))

                        its.coord = b2xCoordinate()
                        setUSEDEF(CO_, mesh_id_coords, its.coord)
                        its.coord.point = []

                        #fw('%s<Coordinate ' % ident)
                        #fw('point="')
                       
                        for x3d_v in vert_tri_list:
                            #fw('%.6f %.6f %.6f ' % mesh_vertices[x3d_v[1]].co[:])
                            its.coord.point.append(round_array(mesh_vertices[x3d_v[1]].co[:]))

                        if export_settings['use_normals'] or is_force_normals:
                            norm = Normal()
                            its.normal = norm
                            #fw('%s<Normal ' % ident)
                            #fw('vector="')
                            for x3d_v in vert_tri_list:
                                #fw('%.6f %.6f %.6f ' % mesh_vertices[x3d_v[1]].normal[:])
                                norm.vector.append(round_array_no_unit_scale(mesh_vertices[x3d_v[1]].normal[:]))

                        if is_uv:
                            its.texCoord = TextureCoordinate()
                            setUSEDEF(TX_, mesh_name, its.texCoord)
                            #fw('%s<TextureCoordinate point="' % ident)
                            for x3d_v in vert_tri_list:
                                #fw('%.4f %.4f ' % x3d_v[0][slot_uv])
                                its.texCoord.point.append(round_array_no_unit_scale(x3d_v[0][slot_uv]))


                        if is_col:
                            #fw('%s<ColorRGBA color="' % ident)
                            rgba = ColorRGBA()
                            its.color = rgba
                            for x3d_v in vert_tri_list:
                                #fw('%.3f %.3f %.3f %.3f ' % x3d_v[0][slot_col])
                                rgba.color.append(x3d_v[0][slot_col])

                        """
                        if use_h3d:
                            # write attributes
                            for gpu_attr in gpu_shader['attributes']:

                                # UVs
                                if gpu_attr['type'] == gpu.CD_MTFACE:
                                    if gpu_attr['datatype'] == gpu.GPU_DATA_2F:
                                        fw('%s<FloatVertexAttribute ' % ident)
                                        fw('name="%s" ' % gpu_attr['varname'])
                                        fw('numComponents="2" ')
                                        fw('value="')
                                        for x3d_v in vert_tri_list:
                                            fw('%.4f %.4f ' % x3d_v[0][slot_uv])
                                        fw('" />\n')
                                    else:
                                        assert(0)

                                elif gpu_attr['type'] == gpu.CD_MCOL:
                                    if gpu_attr['datatype'] == gpu.GPU_DATA_4UB:
                                        pass  # XXX, H3D can't do
                                    else:
                                        assert(0)
                        """

                        #/IndexedTriangleSet

                    else:
                        ifs = x3d_obj
                        ifs.creaseAngle=3.142
                        ifs.ccw=False
                        ifs.convex=True
                        ifs.solid=True

                        print(f"shape is {shape}")
                        # --- Write IndexedFaceSet Attributes (same as IndexedTriangleSet)
                        ifs.solid = material and material.use_backface_culling
                        if is_smooth:
                            # use Auto-Smooth angle, if enabled. Otherwise make
                            # the mesh perfectly smooth by creaseAngle > pi.
                            uas = hasattr(mesh, "use_auto_smooth")
                            asa = hasattr(mesh,"auto_smooth_angle")
                            ifs.creaseAngle = 4.0
                            if uas and asa:
                                ifs.creaseAngle = mesh.auto_smooth_angle

                        if export_settings['x3dv_normals']:
                            # currently not optional, could be made so:
                            ifs.normalPerVertex = True

                        # IndexedTriangleSet assumes true
                        if is_col and not is_col_per_vertex:
                            ifs.colorPerVertex = False

                        # for IndexedTriangleSet we use a uv per vertex so this isn't needed.
                        if is_uv:
                            j = 0
                            for i in polygons_group:
                                num_poly_verts = len(mesh_polygons_vertices[i])
                                #fw('%s -1 ' % ' '.join((str(i) for i in range(j, j + num_poly_verts))))
                                for k in range(j, j + num_poly_verts):
                                    ifs.texCoordIndex.append(k)
                                ifs.texCoordIndex.append(-1)
                                j += num_poly_verts
                            # --- end texCoordIndex

                        if True:
                            for i in polygons_group:
                                poly_verts = mesh_polygons_vertices[i]
                                #fw('%s -1 ' % ' '.join((str(i) for i in poly_verts)))
                                for i in poly_verts:
                                    ifs.coordIndex.append(i)
                                    if export_settings['x3dv_normals']:
                                        ifs.normalIndex.append(i)
                                ifs.coordIndex.append(-1)
                                if export_settings['x3dv_normals']:
                                    ifs.normalIndex.append(-1)
                            # --- end coordIndex


                        # --- Write IndexedFaceSet Elements
                        if True:
                            if is_coords_written:
                                # ifs.coord = b2xCoordinate()
                                setUSEDEF(CO_, mesh_id_coords, ifs.coord, x3d_oid=1)  # we already have a Coordinate
                                if export_settings['x3dv_normals']:
                                    if ifs.normal is None:
                                        ifs.normal = Normal()
                                    setUSEDEF(NO_, mesh_id_normals, ifs.normal, x3d_oid=1)  # 1 for USE
                            else:
                                ifs.coord = b2xCoordinate()
                                setUSEDEF(CO_, mesh_id_coords, ifs.coord)
                                ifs.coord.point = []
                                for v in mesh.vertices:
                                    vco = [ None, None, None ]
                                    vco[0] = v.co[0]
                                    vco[1] = v.co[1]
                                    vco[2] = v.co[2]
                                    loc = round_array(vco)
                                    ifs.coord.point.append(loc[:])

                                if export_settings['x3dv_normals']:
                                    ifs.normal = Normal()
                                    setUSEDEF(NO_, mesh_id_normals, ifs.normal)
                                    ifs.normal.vector = []
                                    for v in mesh.vertices:
                                        ifs.normal.vector.append(round_array_no_unit_scale(v.normal[:]))

                                is_coords_written = True

                        if is_uv:
                            ifs.texCoord = TextureCoordinate()
                            setUSEDEF(TX_, mesh_name, ifs.texCoord)
                            for i in polygons_group:
                                for lidx in mesh_polygons[i].loop_indices:
                                    # TODO fix array out of bounds error
                                    if lidx >= 0 and lidx < len(mesh_loops_uv):
                                        ifs.texCoord.point.append(mesh_loops_uv[lidx].vector)
                                    else:
                                        print_console('INFO',f"ERROR: !!!!!!!!!!!! Array index out of bounds; image {imt.name} mesh {mesh_name} at mesh polygons group {i}, mesh loop index {lidx} should be {len(mesh_loops_uv)} >= 0 and {lidx} < {len(mesh_loops_uv)}")
                                        break
                        if is_col:
                            # Need better logic here, dynamic determination
                            # which of the X3D coloring models fits better this mesh - per face
                            # or per vertex. Probably with an explicit fallback mode parameter.
                            #fw('%s<ColorRGBA color="' % ident)
                            rgba = ColorRGBA()
                            ifs.color = rgba
                            if is_col_per_vertex:
                                for i in range(len(mesh.vertices)):
                                    # may be None,
                                    #fw('%.3f %.3f %.3f %.3f ' % (vert_color[i] or (0.0, 0.0, 0.0, 0.0)))
                                    ifs.color.color.append(vert_color[i] or (0.0, 0.0, 0.0, 0.0))
                            else: # Export as colors per face.
                                # TODO: average them rather than using the first one!
                                for i in polygons_group:
                                    #fw('%.3f %.3f %.3f %.3f ' % mesh_loops_col[mesh_polygons[i].loop_start].color[:])
                                    ifs.color.color.append(mesh_loops_col[mesh_polygons[i].loop_start].color[:])

                        #/IndexedFaceSet
                        # setUSEDEF("", mesh_id, ifs)

                    #/Shape

            #fw('%s<PythonScript DEF="PS" url="object.py" >\n' % ident)
            #fw('%s    <ShaderProgram USE="MA_Material.005" containerField="references"/>\n' % ident)
            #fw('%s</PythonScript>\n' % ident)

            # /Group>

        return coll

    def b2xTextureTransform(material_mtex, material_tex):
        if material_mtex is not None and material_txt is not None:
            loc = material_mtex.offset[:2]
            # mtex_scale * tex_repeat
            sca_x, sca_y = material_mtex.scale[:2]

            sca_x *= material_tex.repeat_x
            sca_y *= material_tex.repeat_y

            # flip x/y is a sampling feature, convert to transform
            if material_tex.use_flip_axis:
                rot = math.pi / -2.0
                sca_x, sca_y = sca_y, -sca_x
            else:
                rot = 0.0

            tt = TextureTransform()
            tt.translation = loc
            tt.scale = (sca_x,sca_y)
            tt.rotation = rot
            return tt

    #def b2xMaterialImage(material):
    #    if material and 'texture_slots' in material:
    #        for mtex in material.texture_slots:
    #            if mtex:
    #                tex = mtex.texture
    #                if tex and tex.type in ["IMAGE", "TEX_IMAGE", "TEX_ENVIRONMENT", "TEXTURE"]:
    #                    image = tex.image
    #    elif material:
    #        image = image_get(material)

    def b2xMaterial(material, world):
        # material_id = unique_name(material, MA_ + material.name, uuid_cache_material, clean_func=clean_def, sep="_")

        # look up material name, use it if available
        mat = Material()
        #setUSEDEF(MA_, material.name, mat)
        mat.DEF = MA_+material.name
        #if material.tag:
        #    pass
        #else:
        material.tag = True
        emit = 0.0 #material.emit
        ambient = 0.0 #material.ambient / 3.0
        diffuseColor = material.diffuse_color[:3]
        if world and 0:
            ambiColor = ((material.ambient * 2.0) * world.ambient_color)[:]
        else:
            ambiColor = 0.0, 0.0, 0.0

        emitColor = tuple(((c * emit) + ambiColor[i]) / 2.0 for i, c in enumerate(diffuseColor))
        shininess = material.specular_intensity
        specColor = tuple((c + 0.001) / (1.25 / (material.specular_intensity + 0.001)) for c in material.specular_color)
        transp = 1.0 - material.diffuse_color[3]

        mat.diffuseColor = round_array(clamp_color(diffuseColor))
        mat.specularColor = clamp_color(specColor)
        mat.emissiveColor = clamp_color(emitColor)
        mat.ambientIntensity = ambient
        mat.shininess = shininess
        mat.transparency = transp
        return mat

    """
    def b2xMaterialH3D(ident, material, world,
                         obj, gpu_shader):
        material_id = unique_name(material, 'MA_' + material.name, uuid_cache_material, clean_func=clean_def, sep="_")

        fw('%s<Material />\n' % ident)
        if material.tag:
            fw('%s<ComposedShader USE=%s />\n' % (ident, material_id))
        else:
            material.tag = True

            # GPU_material_bind_uniforms
            # GPU_begin_object_materials

            #~ CD_MCOL 6
            #~ CD_MTFACE 5
            #~ CD_ORCO 14
            #~ CD_TANGENT 18
            #~ GPU_DATA_16F 7
            #~ GPU_DATA_1F 2
            #~ GPU_DATA_1I 1
            #~ GPU_DATA_2F 3
            #~ GPU_DATA_3F 4
            #~ GPU_DATA_4F 5
            #~ GPU_DATA_4UB 8
            #~ GPU_DATA_9F 6
            #~ GPU_DYNAMIC_LIGHT_DYNCO 7
            #~ GPU_DYNAMIC_LIGHT_DYNCOL 11
            #~ GPU_DYNAMIC_LIGHT_DYNENERGY 10
            #~ GPU_DYNAMIC_LIGHT_DYNIMAT 8
            #~ GPU_DYNAMIC_LIGHT_DYNPERSMAT 9
            #~ GPU_DYNAMIC_LIGHT_DYNVEC 6
            #~ GPU_DYNAMIC_OBJECT_COLOR 5
            #~ GPU_DYNAMIC_OBJECT_IMAT 4
            #~ GPU_DYNAMIC_OBJECT_MAT 2
            #~ GPU_DYNAMIC_OBJECT_VIEWIMAT 3
            #~ GPU_DYNAMIC_OBJECT_VIEWMAT 1
            #~ GPU_DYNAMIC_SAMPLER_2DBUFFER 12
            #~ GPU_DYNAMIC_SAMPLER_2DIMAGE 13
            #~ GPU_DYNAMIC_SAMPLER_2DSHADOW 14

            '''
            inline const char* typeToString( X3DType t ) {
              switch( t ) {
              case     SFFLOAT: return "SFFloat";
              case     MFFLOAT: return "MFFloat";
              case    SFDOUBLE: return "SFDouble";
              case    MFDOUBLE: return "MFDouble";
              case      SFTIME: return "SFTime";
              case      MFTIME: return "MFTime";
              case     SFINT32: return "SFInt32";
              case     MFINT32: return "MFInt32";
              case     SFVEC2F: return "SFVec2f";
              case     MFVEC2F: return "MFVec2f";
              case     SFVEC2D: return "SFVec2d";
              case     MFVEC2D: return "MFVec2d";
              case     SFVEC3F: return "SFVec3f";
              case     MFVEC3F: return "MFVec3f";
              case     SFVEC3D: return "SFVec3d";
              case     MFVEC3D: return "MFVec3d";
              case     SFVEC4F: return "SFVec4f";
              case     MFVEC4F: return "MFVec4f";
              case     SFVEC4D: return "SFVec4d";
              case     MFVEC4D: return "MFVec4d";
              case      SFBOOL: return "SFBool";
              case      MFBOOL: return "MFBool";
              case    SFSTRING: return "SFString";
              case    MFSTRING: return "MFString";
              case      SFNODE: return "SFNode";
              case      MFNODE: return "MFNode";
              case     SFCOLOR: return "SFColor";
              case     MFCOLOR: return "MFColor";
              case SFCOLORRGBA: return "SFColorRGBA";
              case MFCOLORRGBA: return "MFColorRGBA";
              case  SFROTATION: return "SFRotation";
              case  MFROTATION: return "MFRotation";
              case  SFQUATERNION: return "SFQuaternion";
              case  MFQUATERNION: return "MFQuaternion";
              case  SFMATRIX3F: return "SFMatrix3f";
              case  MFMATRIX3F: return "MFMatrix3f";
              case  SFMATRIX4F: return "SFMatrix4f";
              case  MFMATRIX4F: return "MFMatrix4f";
              case  SFMATRIX3D: return "SFMatrix3d";
              case  MFMATRIX3D: return "MFMatrix3d";
              case  SFMATRIX4D: return "SFMatrix4d";
              case  MFMATRIX4D: return "MFMatrix4d";
              case UNKNOWN_X3D_TYPE:
              default:return "UNKNOWN_X3D_TYPE";
            '''
            import gpu

            fw('%s<ComposedShader DEF=%s language="GLSL" >\n' % (ident, material_id))
            ident += '\t'

            shader_url_frag = 'shaders/%s_%s.frag' % (filename_strip, material_id[1:-1])
            shader_url_vert = 'shaders/%s_%s.vert' % (filename_strip, material_id[1:-1])

            # write files
            shader_dir = os.path.join(base_dst, 'shaders')
            if not os.path.isdir(shader_dir):
                os.mkdir(shader_dir)

            # ------------------------------------------------------
            # shader-patch
            field_descr = " <!--- H3D View Matrix Patch -->"
            fw('%s<field name="%s" type="SFMatrix4f" accessType="inputOutput" />%s\n' % (ident, H3D_VIEW_MATRIX, field_descr))
            frag_vars = ["uniform mat4 %s;" % H3D_VIEW_MATRIX]

            # annoying!, we need to track if some of the directional lamp
            # vars are children of the camera or not, since this adjusts how
            # they are patched.
            frag_uniform_var_map = {}

            h3d_material_route.append(
                    '<ROUTE fromNode="%s" fromField="glModelViewMatrix" toNode=%s toField="%s" />%s' %
                    (H3D_TOP_LEVEL, material_id, H3D_VIEW_MATRIX, field_descr))
            # ------------------------------------------------------

            for uniform in gpu_shader['uniforms']:
                if uniform['type'] == gpu.GPU_DYNAMIC_SAMPLER_2DIMAGE:
                    field_descr = " <!--- Dynamic Sampler 2d Image -->"
                    fw('%s<field name="%s" type="SFNode" accessType="inputOutput">%s\n' % (ident, uniform['varname'], field_descr))
                    writeImageTexture(ident + '\t', uniform['image'])
                    fw('%s</field>\n' % ident)

                elif uniform['type'] == gpu.GPU_DYNAMIC_LIGHT_DYNCO:
                    light_obj = uniform['lamp']
                    frag_uniform_var_map[uniform['varname']] = light_obj

                    if uniform['datatype'] == gpu.GPU_DATA_3F:  # should always be true!
                        light_obj_id = unique_name(light_obj, LA_ + light_obj.name, uuid_cache_light, clean_func=clean_def, sep="_")
                        light_obj_transform_id = unique_name(light_obj, light_obj.name, uuid_cache_object, clean_func=clean_def, sep="_")

                        value = '%.6f %.6f %.6f' % (global_matrix * light_obj.matrix_world).to_translation()[:]
                        field_descr = " <!--- Lamp DynCo '%s' -->" % light_obj.name
                        fw('%s<field name="%s" type="SFVec3f" accessType="inputOutput" value="%s" />%s\n' % (ident, uniform['varname'], value, field_descr))

                        # ------------------------------------------------------
                        # shader-patch
                        field_descr = " <!--- Lamp DynCo '%s' (shader patch) -->" % light_obj.name
                        fw('%s<field name="%s_transform" type="SFMatrix4f" accessType="inputOutput" />%s\n' % (ident, uniform['varname'], field_descr))

                        # transform
                        frag_vars.append("uniform mat4 %s_transform;" % uniform['varname'])
                        h3d_material_route.append(
                                '<ROUTE fromNode=%s fromField="accumulatedForward" toNode=%s toField="%s_transform" />%s' %
                                (suffix_quoted_str(light_obj_transform_id, _TRANSFORM), material_id, uniform['varname'], field_descr))

                        h3d_material_route.append(
                                '<ROUTE fromNode=%s fromField="location" toNode=%s toField="%s" /> %s' %
                                (light_obj_id, material_id, uniform['varname'], field_descr))
                        # ------------------------------------------------------

                    else:
                        assert(0)

                elif uniform['type'] == gpu.GPU_DYNAMIC_LIGHT_DYNCOL:
                    # odd  we have both 3, 4 types.
                    light_obj = uniform['lamp']
                    frag_uniform_var_map[uniform['varname']] = light_obj

                    lamp = light_obj.data
                    value = '%.6f %.6f %.6f' % (lamp.color * lamp.energy)[:]
                    field_descr = " <!--- Lamp DynColor '%s' -->" % light_obj.name
                    if uniform['datatype'] == gpu.GPU_DATA_3F:
                        fw('%s<field name="%s" type="SFVec3f" accessType="inputOutput" value="%s" />%s\n' % (ident, uniform['varname'], value, field_descr))
                    elif uniform['datatype'] == gpu.GPU_DATA_4F:
                        fw('%s<field name="%s" type="SFVec4f" accessType="inputOutput" value="%s 1.0" />%s\n' % (ident, uniform['varname'], value, field_descr))
                    else:
                        assert(0)

                elif uniform['type'] == gpu.GPU_DYNAMIC_LIGHT_DYNENERGY:
                    # not used ?
                    assert(0)

                elif uniform['type'] == gpu.GPU_DYNAMIC_LIGHT_DYNVEC:
                    light_obj = uniform['lamp']
                    frag_uniform_var_map[uniform['varname']] = light_obj

                    if uniform['datatype'] == gpu.GPU_DATA_3F:
                        light_obj = uniform['lamp']
                        value = '%.6f %.6f %.6f' % ((global_matrix * light_obj.matrix_world).to_quaternion() * mathutils.Vector((0.0, 0.0, 1.0))).normalized()[:]
                        field_descr = " <!--- Lamp DynDirection '%s' -->" % light_obj.name
                        fw('%s<field name="%s" type="SFVec3f" accessType="inputOutput" value="%s" />%s\n' % (ident, uniform['varname'], value, field_descr))

                        # route so we can have the lamp update the view
                        if h3d_is_object_view(scene, light_obj):
                            light_id = unique_name(light_obj, LA_ + light_obj.name, uuid_cache_light, clean_func=clean_def, sep="_")
                            h3d_material_route.append(
                                '<ROUTE fromNode=%s fromField="direction" toNode=%s toField="%s" />%s' %
                                        (light_id, material_id, uniform['varname'], field_descr))

                    else:
                        assert(0)

                elif uniform['type'] == gpu.GPU_DYNAMIC_OBJECT_VIEWIMAT:
                    frag_uniform_var_map[uniform['varname']] = None
                    if uniform['datatype'] == gpu.GPU_DATA_16F:
                        field_descr = " <!--- Object View Matrix Inverse '%s' -->" % obj.name
                        fw('%s<field name="%s" type="SFMatrix4f" accessType="inputOutput" />%s\n' % (ident, uniform['varname'], field_descr))

                        h3d_material_route.append(
                            '<ROUTE fromNode="%s" fromField="glModelViewMatrixInverse" toNode=%s toField="%s" />%s' %
                                    (H3D_TOP_LEVEL, material_id, uniform['varname'], field_descr))
                    else:
                        assert(0)

                elif uniform['type'] == gpu.GPU_DYNAMIC_OBJECT_IMAT:
                    frag_uniform_var_map[uniform['varname']] = None
                    if uniform['datatype'] == gpu.GPU_DATA_16F:
                        value = ' '.join(['%.6f' % f for v in (global_matrix * obj.matrix_world).inverted().transposed() for f in v])
                        field_descr = " <!--- Object Invertex Matrix '%s' -->" % obj.name
                        fw('%s<field name="%s" type="SFMatrix4f" accessType="inputOutput" value="%s" />%s\n' % (ident, uniform['varname'], value, field_descr))
                    else:
                        assert(0)

                elif uniform['type'] == gpu.GPU_DYNAMIC_SAMPLER_2DSHADOW:
                    pass  # XXX, shadow buffers not supported.

                elif uniform['type'] == gpu.GPU_DYNAMIC_SAMPLER_2DBUFFER:
                    frag_uniform_var_map[uniform['varname']] = None

                    if uniform['datatype'] == gpu.GPU_DATA_1I:
                        if 1:
                            tex = uniform['texpixels']
                            value = []
                            for i in range(0, len(tex) - 1, 4):
                                col = tex[i:i + 4]
                                value.append('0x%.2x%.2x%.2x%.2x' % (col[0], col[1], col[2], col[3]))

                            field_descr = " <!--- Material Buffer -->"
                            fw('%s<field name="%s" type="SFNode" accessType="inputOutput">%s\n' % (ident, uniform['varname'], field_descr))

                            ident += '\t'

                            ident_step = ident + (' ' * (-len(ident) + \
                            fw('%s<PixelTexture \n' % ident)))
                            fw(ident_step + 'repeatS="false"\n')
                            fw(ident_step + 'repeatT="false"\n')

                            fw(ident_step + 'image="%s 1 4 %s"\n' % (len(value), " ".join(value)))

                            fw(ident_step + '/>\n')

                            ident = ident[:-1]

                            fw('%s</field>\n' % ident)

                            #for i in range(0, 10, 4)
                            #value = ' '.join(['%d' % f for f in uniform['texpixels']])
                            # value = ' '.join(['%.6f' % (f / 256) for f in uniform['texpixels']])

                            #fw('%s<field name="%s" type="SFInt32" accessType="inputOutput" value="%s" />%s\n' % (ident, uniform['varname'], value, field_descr))
                            #print_console('INFO','test', len(uniform['texpixels']))
                    else:
                        assert(0)
                else:
                    print_console('INFO',"SKIPPING", uniform['type'])

            file_frag = open(os.path.join(base_dst, shader_url_frag), 'w', encoding='utf-8')
            file_frag.write(gpu_shader['fragment'])
            file_frag.close()
            # patch it
            h3d_shader_glsl_frag_patch(os.path.join(base_dst, shader_url_frag),
                                       scene,
                                       frag_vars,
                                       frag_uniform_var_map,
                                       )

            file_vert = open(os.path.join(base_dst, shader_url_vert), 'w', encoding='utf-8')
            file_vert.write(gpu_shader['vertex'])
            file_vert.close()

            fw('%s<ShaderPart type="FRAGMENT" url=%s />\n' % (ident, quoteattr(shader_url_frag)))
            fw('%s<ShaderPart type="VERTEX" url=%s />\n' % (ident, quoteattr(shader_url_vert)))
            ident = ident[:-1]

            fw('%s</ComposedShader>\n' % ident)
    """

    def b2xImageTexture(image, obj, mesh_name, image_textures, imt):
        try:
            bpy.ops.object.mode_set(mode="EDIT")
        except:
            pass
        if imt is None:
            imt = ImageTexture()
        if mesh_name:
            setUSEDEF(IM_, mesh_name, imt)
        elif image:
            setUSEDEF(IM_, image.name, imt)
        if image is None:
            if image_textures[obj]['png_present']:
                if hasattr(image_textures[obj], 'url'):
                    imt.url = image_textures[obj]['url']

        elif not image.tag:
            image.tag = True
            if not image.filepath:
                uv_dir = os.path.join(base_dst, 'uv')
                if not os.path.isdir(uv_dir):
                    os.mkdir(uv_dir)
                else:
                    #print_console('INFO', f"{uv_dir} folder exists")
                    pass
                png_name = image.name+".png"
                png_url = 'uv/' + png_name  # note forward slash of URL
                png_path = os.path.join(uv_dir, png_name)
                #print_console('INFO', f"UV path is {png_path}")
                image.filepath = png_path

            # collect image paths, can load multiple
            # [relative, name-only, absolute]
            path_mode='AUTO'
            filepath = image.filepath
            filepath_full = bpy.path.abspath(filepath, library=image.library)
            filepath_ref = bpy_extras.io_utils.path_reference(filepath_full, base_src, base_dst, path_mode, "textures", copy_set, image.library)
            filepath_base = os.path.basename(filepath_full)

            images = [
                filepath_ref,
                filepath_base,
            ]
            if path_mode != 'RELATIVE':
                images.append(filepath_full)

            images = [f.replace('\\', '/') for f in images]
            images = [f for i, f in enumerate(images) if f not in images[:i]]

            try:
                # print_console('INFO', f"Saving {image.url}")
                image.save()
                # print_console('INFO', f"Saved {image.url}")
            except:
                pass
            imt.url = ['%s' % escape(f) for f in images]
        try:
            bpy.ops.object.mode_set(mode="OBJECT")
        except:
            pass
        return imt

    def b2xBackground(world):

        if world is None:
            return None

        # note, not re-used
        world_id = unique_name(world, WO_ + world.name, uuid_cache_world, clean_func=clean_def, sep="_")

        # XXX World changed a lot in 2.8... For now do minimal get-it-to-work job.
        # ~ blending = world.use_sky_blend, world.use_sky_paper, world.use_sky_real

        # ~ grd_triple = clamp_color(world.horizon_color)
        # ~ sky_triple = clamp_color(world.zenith_color)
        # ~ mix_triple = clamp_color((grd_triple[i] + sky_triple[i]) / 2.0 for i in range(3))

        blending = (False, False, False)

        grd_triple = clamp_color(world.color)
        sky_triple = clamp_color(world.color)
        mix_triple = clamp_color((grd_triple[i] + sky_triple[i]) / 2.0 for i in range(3))

        bg = Background(DEF= world_id)
        # No Skytype - just Hor color
        if blending == (False, False, False):
            bg.groundColor = grd_triple
            bg.skyColor = grd_triple
        # Blend Gradient
        elif blending == (True, False, False):
            bg.groundColor = (grd_triple + mix_triple)
            bg.skyColor = (sky_triple + mix_triple)
            bg.groundAngle = [1.57]
            bg.skyAngle = [1.57]
        # Blend+Real Gradient Inverse
        elif blending == (True, False, True):
            bg.groundColor = (sky_triple + grd_triple)
            bg.skyColor = (sky_triple + grd_triple + sky_triple)
            bg.groundAngle = [1.57]
            bg.skyAngle = [1.57,3.14159] 
        # Paper - just Zen Color
        elif blending == (False, False, True):
            bg.groundColor = sky_triple
            bg.skyColor = sky_triple
        # Blend+Real+Paper - komplex gradient
        elif blending == (True, True, True):
            bg.groundColor = (sky_triple + grd_triple)
            bg.skyColor = (sky_triple + grd_triple)
            bg.groundAngle = [1.57]
            bg.skyAngle = [1.57]

        # Any Other two colors
        else:
            bg.groundColor = grd_triple
            bg.skyColor = sky_triple

        for tex in bpy.data.textures:
            if tex.type in ["IMAGE", "TEX_IMAGE", "TEX_ENVIRONMENT", "TEXTURE"] and tex.image:
                namemat = tex.name
                pic = tex.image
                basename = bpy.path.basename(pic.filepath)

                if namemat == 'back':
                    bg.backUrl = basename
                elif namemat == 'bottom':
                    bg.bottomkUrl = basename
                elif namemat == 'front':
                    bg.frontUrl = basename
                elif namemat == 'left':
                    bg.leftUrl = basename
                elif namemat == 'right':
                    bg.rightUrl = basename
                elif namemat == 'top':
                    bg.topkUrl = basename

        return bg

    def b2xInterpolators(obj_main, obj_main_id, obj_matrix, prefix):
        interpolators = write_obj_interpolators(obj_main, obj_main_id, obj_matrix, prefix)
        print_console('INFO',f"received {len(interpolators)} interpolators in b2xInterpolators")
        return interpolators

    def b2xShape(x3d_obj=None, geometry=None):
        if x3d_obj is None:
            x3d_obj = Shape(appearance=Appearance(), geometry=geometry)
        else:
            x3d_obj.geometry = geometry
        return x3d_obj

    def b2xLineSet(curve_obj, depsgraph, world, x3d_obj):
        setUSEDEF(LS_, curve_obj.name, x3d_obj)
        print(f"Found line set {x3d_obj}")
        curve_data = curve_obj.data
        obj_for_mesh = curve_obj.evaluated_get(depsgraph)
        try:
            mesh = obj_for_mesh.to_mesh()
        except:
            mesh = None

        is_col = mesh.color_attributes.active
        mesh_loops_col = mesh.color_attributes if is_col else None
        #for color_attr in mesh_loops_col:
        #    print_console('INFO',color_attr.name)
        #print_console('INFO',f"is_col {is_col} mesh {mesh} mesh_loops_col {mesh_loops_col}")
        colors = []
        color = None

        if hasattr(curve_data, "attributes") and "Color" in curve_data.attributes:
            color_attribute = curve_data.attributes["Color"]
            #print_console('INFO',f"color_attribute: {color_attribute}")
            for i in range(len(color_attribute.data)):
                colors.append(color_attribute.data[i].color)
            #print_console('INFO',f"colors from curve_data: {colors}")
        elif is_col:
            for mat in curve_data.materials:
                if mat is not None:
                    colors.append(mat.diffuse_color[:])
                else:
                    print_console('WARNING',f"No material present for curve data converted to LineSet")

        # Get curve points
        points = []
        for spline in curve_data.splines:
            if spline.type == 'BEZIER':
                points.extend([p.co for p in spline.bezier_points])
            else:
                points.extend([p.co for p in spline.points])
        # print_console('INFO',f"end points in curve: {points}")

        # Convert points to X3D format
        x3d_points = MFVec3f([round_array([p.x, p.y, p.z]) for p in points])
        # print_console('INFO',f"end x3d_points in curve: {x3d_points}")

        # compute static colors
        if curve_obj.name.find("Segment") >= 0:   # SkeletonColor
            if not colors:
                colors = [[1, 0, 0, 1], [1, 0, 0, 1]]
        elif "-to-" in curve_obj.name:     # SiteColor
            if not colors:
                colors = [[0, 1, 0, 1], [0, 1, 0, 1]]
        else:
            if not colors:
                colors = [[0, 0, 1, 1], [0, 0, 1, 1]]
        if not colors:
            colors = [[1, 1, 1, 1], [1, 1, 1, 1]]
        last_color = colors[-1]
        print(f"HMMM, last color is {last_color}")

        missing_colors = len(points) - len(colors)
        if missing_colors > 0:
            print(f"MMM, adding {missing_colors}")
            for mc in range(missing_colors):
                colors.append(last_color)
            print(f"MMM, colors {colors}")
        else:
            print(f"MMM, enough {missing_colors}")

        # colorstr = ('_'.join(['_'.join(map(str, sublist)) for sublist in colors]))
        color = ColorRGBA(color=MFColorRGBA(colors))
        setUSEDEF(COLOR_, curve_obj.name, color)


        # Create X3D LineSet structure
        x3d_obj.vertexCount = len(points)
        x3d_obj.coord = Coordinate(point=x3d_points)
        x3d_obj.color = color
        return x3d_obj
    # -------------------------------------------------------------------------
    # blender to x3d Object Hierarchy (recursively called)
    # -------------------------------------------------------------------------
    def b2x_append(parent, x3dnodelist, depth=0):
        # print_console('INFO',f"b2x_append {parent} {x3dnodelist}")
        x3dnode = x3dnodelist
        print(f"{'    ' * depth} parent {type(parent)} x3dnode {type(x3dnode)}")
        if x3dnode is None:
            print_console('WARNING',f"Tried to append null object to {parent}")
        elif isinstance(x3dnode, list):
            x3dnodelist = x3dnode
            # print(f"{'    ' * depth} parent {type(parent)} x3dnode {type(x3dnode)}")
            seen = []
            for x3dnode in x3dnodelist:
                if not x3dnode in seen:
                    seen.append(x3dnode)
                    b2x_append(parent, x3dnode, depth+1)
        elif isinstance(parent, HAnimHumanoid):
            # print(f"{'    ' * depth} parent {type(parent)} x3dnode {type(x3dnode)}")
            if isinstance(x3dnode, HAnimJoint) and  x3dnode.USE:
                parent.joints.append(x3dnode)
            if isinstance(x3dnode, HAnimJoint) :
                parent.skeleton.append(x3dnode)
            elif isinstance(x3dnode, HAnimSegment) and x3dnode.USE:
                parent.segments.append(x3dnode)
            elif isinstance(x3dnode, HAnimSite) and x3dnode.USE:
                parent.sites.append(x3dnode)
            elif isinstance(x3dnode, Shape):
                print_console('WARNING',f"Adding skin twice? {x3dnode}")
                parent.skin.append(x3dnode)
                DEFnodes = b2xDEFedCoordinates(x3dnode)
                USEname = DEFnodes[0].DEF
                parent.skinCoord = b2xCoordinate(USE=USEname)
            elif isinstance(x3dnode, Coordinate):
                print_console('WARNING',f"Adding skinCoord twice? {x3dnode}")
                parent.skinCoord.append(x3dnode)
            elif isinstance(x3dnode, (Transform, Group)):
                b2x_append(parent, x3dnode.children, depth + 1)
            elif isinstance(x3dnode, IndexedFaceSet):
                shape = b2xShape(geometry=x3dnode);
                print_console('WARNING',f"Created shape for skin? {x3dnode}")
                parent.skin.append(shape)
            else:
                print_console('WARNING',f"Do not know where to put child inside {type(parent)} and {type(x3dnode)}")
                # parent.children.append(x3dnode)
        elif isinstance(parent, Shape):
            # print(f"{'    ' * depth} parent {type(parent)} x3dnode {type(x3dnode)}")
            if x3dnode is None:
                print_console('WARNING',f"x3dnode is None for {type(parent)}")
                raise
            if isinstance(x3dnode, (Transform, Group)):
                b2x_append(parent, x3dnode.children, depth + 1)
            elif isinstance(x3dnode, Shape):
                parent.geometry = x3dnode.geometry
                parent.appearance = x3dnode.appearance
            elif isinstance(x3dnode, (Arc2D, ArcClose2D, Box, Circle2D, Cone, Cylinder, Disk2D, ElevationGrid, Extrusion, GeoElevationGrid, IndexedFaceSet, IndexedLineSet, IndexedQuadSet, IndexedTriangleFanSet, IndexedTriangleSet, IndexedTriangleStripSet, LineSet, NurbsCurve, NurbsPatchSurface, NurbsSweptSurface, NurbsSwungSurface, NurbsTrimmedSurface, PointSet, Polyline2D, Polypoint2D, QuadSet, Rectangle2D, Sphere, Text, TriangleFanSet, TriangleSet, TriangleSet2D, TriangleStripSet)):
                parent.geometry = x3dnode
            else:
                parent.appearance = x3dnode
        elif isinstance(x3dnode, (Arc2D, ArcClose2D, Box, Circle2D, Cone, Cylinder, Disk2D, ElevationGrid, Extrusion, GeoElevationGrid, IndexedFaceSet, IndexedLineSet, IndexedQuadSet, IndexedTriangleFanSet, IndexedTriangleSet, IndexedTriangleStripSet, LineSet, NurbsCurve, NurbsPatchSurface, NurbsSweptSurface, NurbsSwungSurface, NurbsTrimmedSurface, PointSet, Polyline2D, Polypoint2D, QuadSet, Rectangle2D, Sphere, Text, TriangleFanSet, TriangleSet, TriangleSet2D, TriangleStripSet)) and not isinstance(parent, Shape):
            #print(f"{'    ' * depth} parent {type(parent)} x3dnode {type(x3dnode)}")
            shape = b2xShape(geometry=x3dnode);
            parent.children.append(shape)
        elif isinstance(parent, Group):
            if not isinstance(x3dnode, HAnimJoint):
                print(f"{'    ' * depth} Appending parent {type(parent)} x3dnode {type(x3dnode)}")
                parent.children.append(x3dnode);
            if isinstance(x3dnode, Shape):
                if x3dnode.geometry is not None:
                    b2x_append(x3dnode, x3dnode.geometry, depth + 1)
                else:
                    print_console('INFO',f"Shape.geometry is null?, {type(parent)} and {type(x3dnode)}")
                if x3dnode.appearance is not None:
                    b2x_append(x3dnode, x3dnode.appearance, depth + 1)
            elif isinstance(x3dnode, Transform):
                pass
            elif x3dnode is None:
                print_console('WARNING',f"Group child is None {type(parent)} and {type(x3dnode)}")
            else:
                print_console('WARNING',f"Fell through for {type(parent)} and {type(x3dnode)}")
        elif isinstance(parent, Transform):
            # print(f"{'    ' * depth} parent {type(parent)} x3dnode {type(x3dnode)}")
            parent.children.append(x3dnode)
        elif parent is not None and parent.children is not None:
            # print_console('INFO',f"DEFAULTED to children?, {type(parent)} and {type(x3dnode)}")
            parent.children.append(x3dnode)
        else:
            print_console('WARNING',f"Couldn't add child, {type(parent)} and {type(x3dnode)}")


    def b2x_object(obj_main_parent, obj_main, obj_children, x3dmodel_scene, image_textures, parent):
        shape = None
        matrix_fallback = mathutils.Matrix()
        world = scene.world
        derived_dict = create_derived_objects(depsgraph, [obj_main])
        derived = derived_dict.get(obj_main)
        #derived_str = str(derived).replace('\n', ' ')
        #print_console('INFO',f"object {obj_main} derived {derived_str}")

        bottom = Group()

        x3d_obj = None
        #if export_settings['x3dv_use_hierarchy']: 
        if True: 
            obj_main_matrix_world = obj_main.matrix_world
            if obj_main_parent:
                obj_main_matrix = obj_main_parent.matrix_world.inverted(matrix_fallback) @ obj_main_matrix_world
            else:
                obj_main_matrix = obj_main_matrix_world
            obj_main_matrix_world_invert = obj_main_matrix_world.inverted(matrix_fallback)

            obj_main_id = unique_name(obj_main, obj_main.name, uuid_cache_object, clean_func=clean_def, sep="_")

            try:
                begin = obj_main_id.index("_")
                x3d_type_str = obj_main_id[:begin]
            except:
                x3d_type_str = "Transform"
                begin = -1
            try:
                if len(obj_children) > 0:
                    end = ""
                    x3d_oid = 0
                else:
                    end = obj_main_id.rindex("_")
                    x3d_oid = int(obj_main_id[end+1:])
            except:
                end = ""
                x3d_oid = 0

            if end  == "":
                x3d_name = obj_main_id[begin+1:]
            else:
                x3d_name = obj_main_id[begin+1:end]

            # Handle EMPTYS
            x3d_matrix = obj_main_matrix if obj_main_parent else global_matrix @ obj_main_matrix

            if obj_main.type == "EMPTY":
                # EMPTYs go in this first section
                if x3d_type_str in ("Sphere", "Box"):
                    x3d_type_str = "IndexedFaceSet"
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                elif x3d_type_str in ("IndexedFaceSet", "IndexedLineSet", "LineSet", "Text"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                elif x3d_type_str in ("HAnimSite", "HAnimSegment"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    x3d_obj = b2xHAnimNode(obj_main, x3d_matrix, x3d_name, x3d_type_str, x3d_obj)
                elif x3d_type_str in ("HAnimHumanoid"):
                    if x3d_name == "root" or x3d_name == "humanoid_root":
                        x3d_name = "humanoid"
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    x3d_obj = b2xHAnimNode(obj_main, x3d_matrix, x3d_name, x3d_type_str, x3d_obj)
                elif x3d_type_str in ("HAnimJoint"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    x3d_obj = b2xHAnimNode(obj_main, x3d_matrix, x3d_name, x3d_type_str+"2", x3d_obj)
                elif x3d_type_str in ("Transform"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    if x3d_obj is not None:
                        x3d_obj = b2xTransform(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix, x3d_obj)
                    else:
                        print(f"WARNING, found weird case2: {x3d_type_str}, {x3d_oid}, {x3d_name}, {obj_main}, {x3d_matrix}")
                elif x3d_type_str in ("Billboard"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    if x3d_obj is not None:
                        x3d_obj.axisOfRotation = axisOfRotation=(0, 0, 0)
                    else:
                        print(f"WARNING, found weird case: {x3d_type_str}, {x3d_oid}, {x3d_name}, {obj_main}, {x3d_matrix}")
                elif x3d_type_str in ("Shape"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    x3d_obj = b2xShape(x3d_obj=x3d_obj)
                elif x3d_type_str in ("TouchSensor"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    x3d_obj.description=obj_main.name[12:].replace('\$', ' ')
                elif x3d_type_str in ("NavigationInfo"):
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                    objects = [obj for obj in view_layer.objects if obj.visible_get(view_layer=view_layer)]
                    x3d_obj = b2xNavigationInfo(any(obj.type == 'LIGHT' for obj in objects), x3d_obj)
                else:
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            elif obj_main.type == "CAMERA":
                x3d_type_str = "Viewpoint"
                x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            elif obj_main.type == "CURVE":
                x3d_type_str = "LineSet"
                x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            elif obj_main.type == "FONT":
                x3d_type_str = "Text"
                x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            elif obj_main.type == "MESH":
                x3d_type_str = "IndexedFaceSet"
                x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            elif obj_main.type == "ARMATURE":
                x3d_type_str = "HAnimHumanoid"
                if x3d_name == "root" or x3d_name == "humanoid_root":
                    x3d_name = "humanoid"
                x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                x3d_obj = b2xHAnimNode(obj_main, x3d_matrix, x3d_name, x3d_type_str, x3d_obj)
                print(f"Found2 humanoid, parent {parent}\n\n\nFound2 x3d_obj {x3d_obj}")
            elif obj_main.type == "LIGHT":
                # Lights go in this section
                data = obj_main.data
                datatype = data.type
                if datatype == 'POINT':
                    x3d_type_str = "PointLight"
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                elif datatype == 'SPOT':
                    x3d_type_str = "SpotLight"
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
                else:   # SUN
                    x3d_type_str = "DirectionalLight"
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            else:
                    x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)

        if x3d_obj is None:
            x3d_type_str = "Transform"
            x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, obj_main_id, obj_main, x3d_matrix)
            x3d_obj = b2xTransform(x3d_type_str, x3d_oid, obj_main_id, obj_main, x3d_matrix, x3d_obj)
            print_console('WARNING',f"OOPS, Couldn't find X3D type, id '{obj_main_id}' type '{x3d_type_str}' {obj_main.type} name '{x3d_name}' begin {begin+1} end {end} int id {x3d_oid}")
            #x3d_obj = b2xInstantiate(x3d_type_str, x3d_oid, x3d_name, obj_main, x3d_matrix)
            #x3d_obj = b2xHAnimNode(obj_main, x3d_matrix, x3d_name, x3d_type_str+"2", x3d_obj)
            # print_console('WARNING',f"OOPS, Couldn't find X3D type, id '{obj_main_id}' type '{x3d_type_str}2' {obj_main.type} name '{x3d_name}' begin {begin+1} end {end} int id {x3d_oid}")
            setUSEDEF("", x3d_name, x3d_obj, x3d_oid=x3d_oid)
            b2x_append(parent, x3d_obj, depth+1)
        else:
            setUSEDEF("", x3d_name, x3d_obj, x3d_oid=x3d_oid)
            b2x_append(parent, x3d_obj, depth+1)
            # print_console('WARNING',f"    CREATED id '{obj_main_id}' type '{x3d_type_str}' {obj_main.type} name '{x3d_name}' begin {begin+1} end {end} int id {x3d_oid} DEF {x3d_obj.DEF} USE {x3d_obj.USE}")


        # Handle other object types
        objects = 0
        seen = []
        for obj, obj_matrix in (() if derived is None else derived):
            if obj in seen:
                continue
            else:
                seen.append(obj)
            after = Group()

            objects += 1
            obj_type = obj.type

            # if export_settings['x3dv_use_hierarchy']:
            if True:
                # make transform node relative
                obj_matrix = obj_main_matrix_world_invert @ obj_matrix
            else:
                obj_matrix = global_matrix @ obj_matrix

            node = None
            if obj_type == 'CAMERA':
                node = b2xViewpoint(obj, obj_matrix, x3d_obj)
            elif obj_type in {'MESH', 'CURVE', 'SURFACE', 'FONT'}:
                if obj_type == 'CURVE':
                    node = b2xLineSet(obj, depsgraph, world, x3d_obj)
                    if node is None:
                        raise "No node"
                    shape = b2xShape(geometry=node)
                    if x3d_obj is None:
                        raise "No x3d_obj"
                    if shape.geometry is None:
                        raise "Couldn't assign"
                    b2x_append(after, shape, depth + 1)
                    # DONE
                    # print_console('INFO',f"materials {obj.data.materials[:]}")
                    for material in obj.data.materials[:]:
                        if material is not None:
                            mat = b2xMaterial(material, world)
                            shape.appearance.material = mat
                            break
                        else:
                            print(f"OUCH, Material None for Line Set {x3d_obj}")
                        # print_console('INFO',f"material {mat.diffuseColor}")
                    me = None # Not really
                    do_remove = False
                elif obj_type == 'FONT':
                    node = b2xText(obj, x3d_obj)
                    if not node:
                        # print_console('INFO',f"Couldn't create {shape} {obj_type}")
                        pass
                    if not isinstance(shape, Shape):
                        shape = b2xShape(geometry=node)
                        b2x_append(after, shape, depth + 1)
                    # print_console('INFO',f"materials {obj.data.materials[:]}")
                    for material in obj.data.materials[:]:
                        if material is not None:
                            mat = b2xMaterial(material, world)
                            shape.appearance.material = mat
                            break
                        else:
                            print(f"OUCH, Material None for Text node {x3d_obj}")
                        # print_console('INFO',f"material {mat.diffuseColor}")
                    me = None # Not really
                    do_remove = False
                elif obj_type == 'SURFACE':
                    obj_for_mesh = obj.evaluated_get(depsgraph) if export_settings['x3dv_use_mesh_modifiers'] else obj
                    try:
                        me = obj_for_mesh.to_mesh()
                        do_remove = True
                    except:
                        me = None
                elif obj_type == 'MESH':
                    me = obj.data
                    do_remove = False
                else:
                    me = obj.data
                    do_remove = False
                # Export all Mesh images
                try:
                    bpy.context.view_layer.objects.active = obj
                    bpy.ops.object.mode_set(mode="EDIT")
                    bpy.ops.mesh.select_all(action='TOGGLE')
                    bpy.ops.mesh.select_all(action='TOGGLE')
                except:
                    pass
                if obj_type in {'MESH', 'CURVE', 'SURFACE', 'FONT'}:
                    obj.select_set(True)
                    uv_dir = os.path.join(base_dst, 'uv')
                    if not os.path.isdir(uv_dir):
                        os.mkdir(uv_dir)
                    else:
                        #print_console('INFO', f"{uv_dir} folder exists")
                        pass
                    png_name = obj.name+".png"
                    png_url = 'uv/' + png_name  # note forward slash of URL
                    png_path = os.path.join(uv_dir, png_name)
                    # print_console('INFO', f"UV path is {png_path} did not call bpy.ops.uv.export_layout()")
                    # bpy.ops.uv.export_layout(filepath=png_path, mode='PNG', size=(4096, 4096), opacity=1)
                    if obj in image_textures:
                        image_textures[obj]['png_present'] = True
                        image_textures[obj]['url'].append(png_url)
                    else:
                        image_textures[obj] = {}
                        image_textures[obj]['png_present'] = True
                        image_textures[obj]['url'] = [png_url]
                try:
                    bpy.ops.object.mode_set(mode="OBJECT")
                    bpy.ops.object.select_all(action='TOGGLE')
                except:
                    pass


                if me is not None:
                    # ensure unique name, we could also do this by
                    # postponing mesh removal, but clearing data - TODO
                    if do_remove:
                        me_name_new = me_name_original = obj.name.rstrip("1234567890").rstrip(".")
                        count = 0
                        while me_name_new in mesh_name_set:
                            me_name_new = "%.17s.%03d" % (me_name_original, count)
                            count += 1
                        mesh_name_set.add(me_name_new)
                        mesh_name = me_name_new
                        del me_name_new, me_name_original, count
                    else:
                        mesh_name = me.name
                    # done

                    if not isinstance(shape, Shape):
                        shape = b2xShape(geometry=x3d_obj)
                        print_console('INFO',f"Created new Shape {shape} MESH/SURFACE")
                    else:
                        print_console('INFO',f"Used old Shape {shape} MESH/SURFACE")

                    coll = b2xIndexedFaceSet(obj, me, mesh_name, obj_matrix, world, image_textures, shape, x3d_obj)
                    if coll:
                        b2x_append(after, coll, depth + 1)
                        # what if after is an IFS TODO
                    b2x_append(after, shape, depth + 1)
                    #print_console('INFO',f"materials {obj.data.materials[:]}")
                    for material in obj.data.materials[:]:
                        if material is not None:
                            mat = b2xMaterial(material, world)
                            shape.appearance.material = mat
                            break
                        else:
                            print(f"OUCH, Material None for IFS {x3d_obj}")
                    #    print_console('INFO',f"material {mat.diffuseColor}")
                    if do_remove:
                        obj_for_mesh.to_mesh_clear()

            elif obj_type == 'LIGHT':
                data = obj.data
                datatype = data.type
                # print_console('INFO','obj type is LIGHT, datatype %s' % datatype)
                if datatype == 'POINT':
                    node = b2xPointLight( obj, obj_matrix, data, world, x3d_obj)
                elif datatype == 'SPOT':
                    node = b2xSpotLight( obj, obj_matrix, data, world, x3d_obj)
                elif datatype == 'SUN':
                    node = b2xDirectionalLight( obj, obj_matrix, data, world, x3d_obj)
                else:
                    node = b2xDirectionalLight( obj, obj_matrix, data, world, x3d_obj)
            elif x3d_type_str == "HAnimHumanoid":
                # obj is ARMATURE or EMPTY
                node = b2xArmature(obj, obj_main, obj_children, obj_matrix, obj.data, world, x3d_obj)
                DEFnodes = []
                point = None
                seen = []
                for obj_child, obj_child_children in obj_children:
                    if not obj_child in seen:
                        seen.append(obj_child)
                        # x3dnodelist is just [ node ], which we appended above
                        for a in  b2x_object(obj_main, obj_child, obj_child_children, x3dmodel_scene, image_textures, node).children:
                            if not a in bottom.children:
                                bottom.children.append(a)
                        if after is not None:
                            for x3dnode in after.children:
                                # if node:
                                #     print_console('INFO', f"appending node {x3dnode} to\n{node.skin}")
                                # print(f"Found {x3dnode}")
                                if isinstance(x3dnode, (Group, LOD, Shape, Switch, Transform, IndexedFaceSet, IndexedTriangleFanSet, LineSet, IndexedLineSet, IndexedQuadSet, IndexedTriangleSet, IndexedTriangleStripSet)):
                                    print(f"Found skin, {x3dnode}")
                                    b2x_append(node, x3dnode, depth + 1)
                                    point = b2xFindSkinCoordPoint(x3dnode)
                                    DEFnodes = b2xDEFedCoordinates(x3dnode)
                                else:
                                    if x3dnode not in bottom.children:
                                        print(f"Found non-skin, {x3dnode}")
                                        print_console('INFO', f"appending node {x3dnode} to bottom {bottom.children}")
                                        bottom.children.append(x3dnode)
                            after = Group()
                if len(DEFnodes) >= 1:
                    DEFname = DEFnodes[0].DEF
                    DEFnodes[0].USE = DEFname 
                    DEFnodes[0].DEF = None
                    DEFnodes[0].point = None  # erase point from this.  VRML output is wrong from x3dv.py
                else:
                    DEFname = "JointsSkinCoordPoint"
                if node:
                    node.skinCoord = b2xCoordinate(DEF=DEFname, point=point)  # TODO skinCoord comes before skin for now
                interpolators = b2xHAnimNode(obj, None, obj.name, "HAnimInterpolators", node)
                if len(interpolators) > 0:
                    bottom.children += interpolators
                    print_console('INFO', f"exported {len(interpolators)} nodes for animations..")
            elif obj_type == 'EMPTY':
                seen = []
                for obj_child, obj_child_children in obj_children:
                    if not obj_child in seen:
                        seen.append(obj_child)
                        for a in b2x_object(obj_main, obj_child, obj_child_children, x3dmodel_scene, image_textures, x3d_obj).children:
                            if not a in bottom.children:
                                bottom.children.append(a)
            else:
                print_console('INFO', "Ignoring [%s], object type [%s], python type [%s], data [%s] not handled yet" % (obj.name,obj_type,type(obj), type(obj.data)))
        # print_console('INFO', f"Number of objects in loop {objects}")
            interpolators = b2xInterpolators(obj, obj_main_id, obj_matrix, "") # prefix was "IN_"
            if len(interpolators) > 0:
                bottom.children += interpolators
                print_console('INFO', f"exported {len(interpolators)} nodes for animations")
            if after:
                for a in after.children:
                    if a not in bottom.children:
                        bottom.children.append(a)
                after = Group()

        print_console('INFO', f"bottom.children {len(bottom.children)}")
        return bottom

    # -------------------------------------------------------------------------
    # Main Export Function
    # -------------------------------------------------------------------------

    def export_main():
        world = scene.world
        image_textures = {}
        name_used.reset()

        x3dmodel = X3D(profile='Immersive',version='4.0')
        x3dmodel.Scene = Scene()
        # tag un-exported IDs
        bpy.data.meshes.tag(False)
        bpy.data.materials.tag(False)
        bpy.data.images.tag(False)

        if export_settings['x3dv_selected']:
            objects = [obj for obj in view_layer.objects if obj.visible_get(view_layer=view_layer)
                       and obj.select_get(view_layer=view_layer)]
        else:
            objects = [obj for obj in view_layer.objects if obj.visible_get(view_layer=view_layer)]

        print_console('INFO', 'starting X3D export to %r...' % export_settings['x3dv_filepath'])

        x3dmodel.head = b2xHeader()

        x3dmodel.Scene.children.append( b2xBackground( world) )
        for time_node in export_TimeSensor():
            x3dmodel.Scene.children.append( time_node )

        fog = b2xFog(world)
        if(fog):
            x3dmodel.Scene.children.append(fog)

        # if export_settings['x3dv_use_hierarchy']: 
        if True: 
            objects_hierarchy = build_hierarchy(objects)
        else:
            objects_hierarchy = ((obj, []) for obj in objects)

#        # skeleton visualization
#        x3dmodel.Scene.children.append(
#            PointLight(location=(0, -10, 0))
#        )
#        x3dmodel.Scene.children.append(
#            PointLight(location=(0, 0, 10))
#        )
#        site_shape = Shape(
#                    geometry=Box(size = (0.05, 0.05, 0.05)),
#                    appearance=Appearance(
#                        material=Material(
#                            diffuseColor = (0, 0, 1),
#                            transparency = 1
#                        ))
#                )
#        setUSEDEF("", "SiteShape", site_shape)
#        x3dmodel.Scene.children.append(
#            Transform( children=[
#                site_shape
#            ])
#        )
#        x3dmodel.Scene.children.append(
#            Transform( children=[
#                Shape(appearance=Appearance(lineProperties=LineProperties(linewidthScaleFactor=5)),
#                      geometry=LineSet(
#                        vertexCount=2,
#                        coord=Coordinate(point=MFVec3f([(0, 0, 0), (0, 0, 0)])),
#                        color=ColorRGBA(
#                            DEF='SegmentLineColor',
#                            color=MFColorRGBA([(0.0, 0.0, 1.0, 1.0), (0.0, 1.0, 0.0, 1.0)])
#                        )
#                ))
#            ])
#        )
#
#        joint_shape = Shape(
#                    geometry=Sphere(radius=0.06),
#                    appearance=Appearance(
#                        DEF="JointAppearance",
#                        material=Material(
#                            diffuseColor = (1, 0.5, 0),
#                            transparency = 1
#                        ))
#                )
#        setUSEDEF("", "JointShape", joint_shape)
#        x3dmodel.Scene.children.append(
#           Transform( children=[
#               joint_shape
#           ])
#        )
        

        x3d_obj = Group()
        x3dmodel.Scene.children.append(x3d_obj)

        seen = []
        for obj_main, obj_main_children in objects_hierarchy:
            if not obj_main in seen:
                seen.append(obj_main)
                children = b2x_object(None, obj_main, obj_main_children, x3dmodel.Scene, image_textures, x3d_obj).children
                #if x3dnodelist:
                #    print_console('INFO', f"appending2 {len(x3dnodelist)} sub-nodes for scene children.")
                #   for x3dnode in x3dnodelist:
                #       if x3dnode not in x3dmodel.Scene.children:
                #           x3dmodel.Scene.children.append(x3dnode)
                if children:
                    print_console('INFO', f"appending {len(children)} sub-nodes for scene children.")
                    for c in children:
                        if c not in x3dmodel.Scene.children:
                            x3dmodel.Scene.children.append(c)

        # swap_USEbeforeDEF(node=x3dmodel.Scene)
        USEdict = {}
        DEFdict = {}
        USEbeforeDEFdict = {}
        return x3dmodel


    x3dmodel = export_main()

    # -------------------------------------------------------------------------
    # global cleanup
    # -------------------------------------------------------------------------

    #if use_h3d:
    #    bpy.data.materials.remove(gpu_shader_dummy_mat)

    # copy all collected files.
    # print_console('INFO',copy_set)
    bpy_extras.io_utils.path_reference_copy(copy_set)

    print_console('INFO', 'finished X3D export to %r' % export_settings['x3dv_filepath']) 
    return x3dmodel

##########################################################
# Callbacks, needed before Main
##########################################################


def gzip_open_utf8(filepath, mode):
    """Workaround for py3k only allowing binary gzip writing"""

    import gzip

    # need to investigate encoding
    file = gzip.open(filepath, mode)
    write_real = file.write

    def write_wrap(data):
        return write_real(data.encode("utf-8"))

    file.write = write_wrap

    return file


def save_OLD(context,x3dv_export_settings):
    """
         filepath,
         *,
         use_selection=True,
         use_mesh_modifiers=False,
         use_triangulate=False,
         use_normals=False,
         use_compress=False,
         use_hierarchy=True,
         use_h3d=False,
         global_matrix=None,
         path_mode='AUTO',
         prefix='',
         name_decorations=True
         ):
    """
    export_settings = x3dv_export_settings
    #bpy.path.ensure_ext(filepath, '.x3dz' if use_compress else '.x3d')

    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')


    if global_matrix is None:
        global_matrix = mathutils.Matrix()
    export_settings['x3dv_global_matrix'] = global_matrix
    x3dstream = export()
    """
           file,
           global_matrix,
           context.evaluated_depsgraph_get(),
           context.scene,
           context.view_layer,
           use_mesh_modifiers=use_mesh_modifiers,
           use_selection=use_selection,
           use_triangulate=use_triangulate,
           use_normals=use_normals,
           use_hierarchy=use_hierarchy,
           use_h3d=use_h3d,
           path_mode=path_mode,
           prefix=prefix,
           name_decorations=name_decorations,
           )
    """
    if export_settings['x3dv_compress']:
        file = gzip_open_utf8(filepath, 'w')
    else:
        file = open(filepath, 'w', encoding='utf-8')

    file.write(stream)
    file.close()

    return {'FINISHED'}

def save(context,export_settings):
    """Start the x3dv export and saves to content file."""

    try:
        bpy.ops.object.mode_set(mode='OBJECT')
    except:
        pass

    original_frame = context.scene.frame_current
    if not export_settings['x3dv_current_frame']:
        context.scene.frame_set(0)

    __notify_start(context)
    start_time = time.time()
    x3dv_round_precision = export_settings['x3dv_round_precision']

    #x3dmodel = blender2x3d(export_settings) #test export of procedurally declared test scene
    x3dmodel = export(context, export_settings)
    format = export_settings['x3dv_format']
    blob = None
    if(format=='X3D'):
        blob = x3dmodel.XML()
    elif(format=='X3DV'):
        blob = x3dmodel.VRML(VRML97=False)
    elif(format=='HTML'):
        blob = x3dmodel.X3DOM()
    elif(format=='JSON'):
        blob = x3dmodel.JSON()
    else:
        print_console('ERROR', "No file format given")
    __write_file(blob,export_settings)


    end_time = time.time()
    __notify_end(context, end_time - start_time)

    if not export_settings['x3dv_current_frame']:
        context.scene.frame_set(int(original_frame))
    return {'FINISHED'}

"""
def blender2x3d(export_settings):
    print_console('INFO','__export(settings)')
    x3dmodel = X3D()
    x3dmodel = X3D(profile='Immersive',version='3.3',
      head=head(
        children=[
        meta(content='HelloWorld.x3d',name='title'),
        meta(content='Simple X3D scene example: Hello World!',name='description'),
        meta(content='30 October 2000',name='created'),
        meta(content='31 October 2019',name='modified'),
        meta(content='Don Brutzman',name='creator'),
        meta(content='HelloWorld.tall.png',name='Image'),
        meta(content='http://en.wikipedia.org/wiki/Hello_world',name='reference'),
        meta(content='https://en.wikipedia.org/wiki/Hello#.22Hello.2C_World.22_computer_program',name='reference'),
        meta(content='https://en.wikipedia.org/wiki/"Hello,_World!"_program',name='reference'),
        meta(content='http://en.wikibooks.org/w/index.php?title=Computer_Programming/Hello_world',name='reference'),
        meta(content='http://www.HelloWorldExample.net',name='reference'),
        meta(content='http://www.web3D.org',name='reference'),
        meta(content='http://www.web3d.org/realtime-3d/news/internationalization-x3d',name='reference'),
        meta(content='http://www.web3d.org/x3d/content/examples/HelloWorld.x3d',name='reference'),
        meta(content='http://X3dGraphics.com/examples/X3dForAdvancedModeling/HelloWorldScenes',name='reference'),
        meta(content='http://X3dGraphics.com/examples/X3dForWebAuthors/Chapter01TechnicalOverview/HelloWorld.x3d',name='identifier'),
        meta(content='http://www.web3d.org/x3d/content/examples/license.html',name='license'),
        meta(content='X3D-Edit 3.3, https://savage.nps.edu/X3D-Edit',name='generator'),
        #  Alternate encodings: VRML97, X3D ClassicVRML Encoding, X3D Compressed Binary Encoding (CBE), X3DOM, JSON 
        meta(content='HelloWorld.wrl',name='reference'),
        meta(content='HelloWorld.x3dv',name='reference'),
        meta(content='HelloWorld.x3db',name='reference'),
        meta(content='HelloWorld.xhtml',name='reference'),
        meta(content='HelloWorld.json',name='reference')]),
      Scene=Scene(
        #  Example scene to illustrate X3D nodes and fields (XML elements and attributes) 
        children=[
        WorldInfo(title='Hello World!'),
        WorldInfo(title="Hello ' apostrophe 1"),
        WorldInfo(title="Hello ' apostrophe 2"),
        WorldInfo(title='Hello " quotation mark 3'),
        WorldInfo(title='Hello " quotation mark 4'),
        MetadataSet(name="items'",
          value=[
          MetadataInteger(name='one',value=[1]),
          MetadataInteger(name='two',value=[2])]),
        Group(
          children=[
          Viewpoint(DEF='ViewUpClose',centerOfRotation=(0,-1,0),description='Hello world!',position=(0,-1,7)),
          #  insert commas to test removal when converted to ttl 
          Transform(DEF='TestWhitespaceCommas',rotation=(0,1,0,3),
            children=[
            Shape(
              geometry=Sphere(),
              appearance=Appearance(
                material=Material(DEF='MaterialLightBlue',diffuseColor=(0.1,0.5,1)),
                texture=ImageTexture(DEF='ImageCloudlessEarth',url=["earth-topo.png","earth-topo.jpg","earth-topo-small.gif","http://www.web3d.org/x3d/content/examples/Basic/earth-topo.png","http://www.web3d.org/x3d/content/examples/Basic/earth-topo.jpg","http://www.web3d.org/x3d/content/examples/Basic/earth-topo-small.gif"])))]),
          Transform(translation=(0,-2,0),
            children=[
            Shape(
              geometry=Text(DEF='TextMessage',string=["Hello","world!"],
                fontStyle=FontStyle(justify=["MIDDLE","MIDDLE"])),
              appearance=Appearance(
                material=Material(USE='MaterialLightBlue')))])])])
    ) # X3D model complete  

    return x3dmodel

"""

def __is_empty_collection(value):
    return (isinstance(value, dict) or isinstance(value, list)) and len(value) == 0


def __write_file(blob, export_settings):
    try:
        print_console('INFO','x3dv_io_export.save_x3d(data)')
        f = open(export_settings['x3dv_filepath'], "w+")
        f.write(blob)
        f.close()

    except AssertionError as e:
        _, _, tb = sys.exc_info()
        traceback.print_tb(tb)  # Fixed format
        tb_info = traceback.extract_tb(tb)
        for tbi in tb_info:
            filename, line, func, text = tbi
            print_console('ERROR', 'An error occurred on line {} in statement {}'.format(line, text))
        print_console('ERROR', str(e))
        raise e


def __notify_start(context):
    print_console('INFO', 'Starting x3dv export')
    context.window_manager.progress_begin(0, 100)
    context.window_manager.progress_update(0)


def __notify_end(context, elapsed):
    print_console('INFO', 'Finished x3dv export in {} s'.format(elapsed))
    context.window_manager.progress_end()
    print_newline()

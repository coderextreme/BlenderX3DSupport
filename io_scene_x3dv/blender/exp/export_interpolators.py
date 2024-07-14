# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# <pep8 compliant>

# Script copyright (C) Campbell Barton
# fixes from Andrea Rugliancich

import bpy
import sys
import os
import random

from x3dv import *
from .RoundArray import round_array, round_array_no_unit_scale
from io_scene_x3dv.io.com.x3dv_io_debug import print_console, print_newline

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
        print(f"converting {subs} to {bvh2HAnim[subs]}")
        subs = bvh2HAnim[subs]
    else:
        #print(f"{subs} not found")
        pass
    return subs.replace(":", "_").replace(" ", "_").replace(".", "_")

#uuid_defs = {}                   # defs
#
#def name_used(DEF):
#    if DEF in uuid_defs.keys():
#        uuid_defs[DEF] = uuid_defs[DEF] + 1
#        return True
#    else:
#        uuid_defs.update({DEF: 1})
#        return False


#def setUSEDEF(prefix, name, node):
#    if name is None:
#        name = ""
#    if name.startswith(prefix):
#        name = name[len(prefix):]
#    node.name = substitute(name)
#    pname = prefix+substitute(name)
#    if name_used(pname):
#        if name == "SiteShape":
#            # create a new empty copy for USE
#            node = type(node)(USE=pname)
#        else:
#            node.USE = pname
#    else:
#        node.DEF = pname
#
#    return node

class NameUsed:
    def __init__(self):
        self.reset()

    def lookup_center(self, DEF):
        return self.uuid_defs[DEF].center

    def is_used(self, DEF, node):
        if DEF in self.uuid_defs.keys() and type(node) == type(self.uuid_defs[DEF]):
            #if isinstance(node, HAnimJoint) and hasattr(node, "center") and node.center:
            #    self.uuid_defs[DEF].center = node.center
            #    print(f"Node uuid {DEF} center {self.uuid_defs[DEF].center}")
            if isinstance(self.uuid_defs[DEF], HAnimJoint) and hasattr( self.uuid_defs[DEF].center, "center") and self.uuid_defs[DEF].center:
                node.center = self.uuid_defs[DEF].center
                print(f"Node {DEF} center {node.center}")
        else:
            self.uuid_defs[DEF] = node
            if isinstance(node, HAnimJoint) and hasattr(node, "center") and node.center:
                self.uuid_defs[DEF].center = node.center
                print(f"Node update {node.DEF} center {node.center} uuid def {self.uuid_defs[DEF].center}")

    def  reset(self):
        self.uuid_defs = {}

name_used = NameUsed()

def setUSEDEF(prefix, name, node):
    if name is None:
        name = ""
    if name.startswith(prefix):
        name = name[len(prefix):]
    node.name = substitute(name)
    pname = prefix+substitute(name)
    node.DEF = pname
    name_used.is_used(pname, node)
    return node

def write_interpolators(obj, name, prefix):  # pass armature object

    root_found = False
    uuid_defs = {}                   # defs

    def ensure_rot_order(rot_order_str):
        if set(rot_order_str) != {'X', 'Y', 'Z'}:
            rot_order_str = "XZY"
        return rot_order_str

    from mathutils import Matrix, Euler
    from math import degrees

    arm = obj.data
    nodes = []
    rotate_mode = 'AXIS_ANGLE'
    root_transform_only=False

    # Build a dictionary of children.
    # None for parentless
    children = {None: []}

    # initialize with blank lists
    for bone in arm.bones:
        children[bone.name] = []

    # keep bone order from armature, no sorting, not esspential but means
    # we can maintain order from import -> export which secondlife incorrectly expects.
    for bone in arm.bones:
        children[getattr(bone.parent, "name", None)].append(bone.name)

    # bone name list in the order that the bones are written
    serialized_names = []

    node_locations = {}

    def write_recursive_nodes(bone_name):
        my_children = children[bone_name]

        bone = arm.bones[bone_name]
        pose_bone = obj.pose.bones[bone_name]
        loc = bone.head_local
        node_locations[bone_name] = loc

        if rotate_mode == "NATIVE":
            rot_order_str = ensure_rot_order(pose_bone.rotation_mode)
        else:
            rot_order_str = rotate_mode

        # make relative if we can
        if bone.parent:
            loc = loc - node_locations[bone.parent.name]

        if my_children:
            # store the location for the children
            # to get their relative offset

            # Write children
            for child_bone in my_children:
                serialized_names.append(child_bone)
                # print(f"writing {child_bone}")
                write_recursive_nodes(child_bone)

        else:
            # Write the bone end.
            loc = bone.tail_local - node_locations[bone_name]
            root_found = True

    if len(children[None]) == 1:
        key = children[None][0]
        serialized_names.append(key)

        print_console('INFO', f"writing only key {key}")
        write_recursive_nodes(key)

    else:
        for child_bone in children[None]:
            serialized_names.append(child_bone)
            # print(f"writing {child_bone}")
            write_recursive_nodes(child_bone)

    class DecoratedBone:
        __slots__ = (
            # Bone name, used as key in many places.
            "name",
            "parent",  # decorated bone parent, set in a later loop
            # Blender armature bone.
            "rest_bone",
            # Blender pose bone.
            "pose_bone",
            # Blender pose matrix.
            "pose_mat",
            # Blender rest matrix (armature space).
            "rest_arm_mat",
            # Blender rest matrix (local space).
            "rest_local_mat",
            # Pose_mat inverted.
            "pose_imat",
            # Rest_arm_mat inverted.
            "rest_arm_imat",
            # Rest_local_mat inverted.
            "rest_local_imat",
            # Last used euler to preserve euler compatibility in between keyframes.
            "prev_euler",
            # Is the bone disconnected to the parent bone?
            "skip_position",
            "rot_order",
            "rot_order_str",
            # Needed for the euler order when converting from a matrix.
            "rot_order_str_reverse",
        )

        _eul_order_lookup = {
            'AXIS_ANGLE': (0, 1, 2),
            'XYZ': (0, 1, 2),
            'XZY': (0, 2, 1),
            'YXZ': (1, 0, 2),
            'YZX': (1, 2, 0),
            'ZXY': (2, 0, 1),
            'ZYX': (2, 1, 0),
        }

        def __init__(self, bone_name):
            self.name = bone_name
            self.rest_bone = arm.bones[bone_name]
            self.pose_bone = obj.pose.bones[bone_name]

            if rotate_mode == "NATIVE":
                self.rot_order_str = ensure_rot_order(self.pose_bone.rotation_mode)
            elif rotate_mode == 'AXIS_ANGLE':
                self.rot_order_str = 'XYZ'
            else:
                self.rot_order_str = rotate_mode

            self.rot_order_str_reverse = self.rot_order_str[::-1]

            self.rot_order = DecoratedBone._eul_order_lookup[self.rot_order_str]

            self.pose_mat = self.pose_bone.matrix

            # mat = self.rest_bone.matrix  # UNUSED
            self.rest_arm_mat = self.rest_bone.matrix_local
            self.rest_local_mat = self.rest_bone.matrix

            # inverted mats
            self.pose_imat = self.pose_mat.inverted()
            self.rest_arm_imat = self.rest_arm_mat.inverted()
            self.rest_local_imat = self.rest_local_mat.inverted()

            self.parent = None
            self.prev_euler = Euler((0.0, 0.0, 0.0), self.rot_order_str_reverse)
            # self.skip_position = ((self.rest_bone.use_connect or root_transform_only) and self.rest_bone.parent)
            self.skip_position = True

        def update_posedata(self):
            self.pose_mat = self.pose_bone.matrix
            self.pose_imat = self.pose_mat.inverted()

        def __repr__(self):
            if self.parent:
                return "[\"%s\" child on \"%s\"]\n" % (self.name, self.parent.name)
            else:
                return "[\"%s\" root bone]\n" % (self.name)

    bones_decorated = [DecoratedBone(bone_name) for bone_name in serialized_names]

    # Assign parents
    bones_decorated_dict = {dbone.name: dbone for dbone in bones_decorated}
    for dbone in bones_decorated:
        parent = dbone.rest_bone.parent
        if parent:
            dbone.parent = bones_decorated_dict[parent.name]
    del bones_decorated_dict
    # finish assigning parents

    scene = bpy.context.scene
    frame_start = scene.frame_start
    frame_end = scene.frame_end
    frame_current = scene.frame_current
    frame_count = frame_end - frame_start + 1
    frame_duration = (1.0 / (scene.render.fps / scene.render.fps_base))

    key_divider = (frame_count - 1) * frame_count / scene.render.fps 

    print_console('INFO', "Frame count: %d\n" % frame_count)
    print_console('INFO', "Frame duration: %.6f\n" % frame_duration)
    print_console('INFO', "Key divider: %.6f\n" % key_divider)
    armature = obj
    numbones = len(armature.pose.bones)
    frame_range = [frame_current, frame_end]
    time_sensor = TimeSensor(cycleInterval=(frame_duration * (frame_end - frame_current)), loop=True, enabled=True)
    clock_name = substitute("X3DV_Clock")
    positionInterpolators = []
    orientationInterpolators = []
    positionRoutes = []
    orientationRoutes = []
    root_found = False
    b = 0
    for dbone in bones_decorated:
        bone = armature.pose.bones[b]
        # print(f"Creating interpolators for {bone.name}")
        pbonename = prefix+substitute(bone.name)
        if bone.name == 'humanoid_root':
            pibonename = substitute(bone.name)+"_PI"
            posInterp = PositionInterpolator()
            setUSEDEF("", pibonename, posInterp)
            positionInterpolators.append(posInterp)
            positionRoutes.append(ROUTE(
                fromNode=clock_name,
                fromField="fraction_changed",
                toNode=pibonename,
                toField="set_fraction"))
            positionRoutes.append(ROUTE(
                fromNode=pibonename,
                fromField="value_changed",
                toNode=pbonename,
                toField="translation"))
            root_found = True

        rotInterp = OrientationInterpolator()
        oibonename = substitute(bone.name)+"_OI"
        setUSEDEF("", oibonename, rotInterp)
        orientationInterpolators.append(rotInterp)
        orientationRoutes.append(ROUTE(
            fromNode=clock_name,
            fromField="fraction_changed",
            toNode=oibonename,
            toField="set_fraction"))
        orientationRoutes.append(ROUTE(
            fromNode=oibonename,
            fromField="value_changed",
            toNode=pbonename,
            toField="rotation"))
        b += 1
    if not root_found:
        print_console('ERROR', "humanoid_root not found in bone data")
    root_found = False
    lasttime = range(int(frame_range[0]), int(frame_range[1]) + 1)[-1]
    keyframe_length = (frame_range[1] - frame_range[0]) / bpy.context.scene.render.fps
    keyframe_time = 0

    skip = False
    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)

        for dbone in bones_decorated:
            dbone.update_posedata()

        b = 0
        for dbone in bones_decorated:
            trans = Matrix.Translation(dbone.rest_bone.head_local)
            itrans = Matrix.Translation(-dbone.rest_bone.head_local)

            if dbone.parent:
                mat_final = dbone.parent.rest_arm_mat @ dbone.parent.pose_imat @ dbone.pose_mat @ dbone.rest_arm_imat
                mat_final = itrans @ mat_final @ trans
                loc = mat_final.to_translation() + (dbone.rest_bone.head_local - dbone.parent.rest_bone.head_local)
            else:
                mat_final = dbone.pose_mat @ dbone.rest_arm_imat
                mat_final = itrans @ mat_final @ trans
                loc = mat_final.to_translation() + dbone.rest_bone.head

            # keep eulers compatible, no jumping on interpolation.
            locign, rot, scaleign = mat_final.decompose()

            # rot = mat_final.to_euler(dbone.rot_order_str_reverse, dbone.prev_euler)
            rot = rot.to_axis_angle()  # convert Quaternion to Axis-Angle
            # print(f"Rotation {rot}")

            if dbone.skip_position:
                skip = True
            else:
                positionInterpolators[b].key.append(round_array_no_unit_scale([keyframe_time / key_divider])[:])
                positionInterpolators[b].keyValue.append(round_array(loc)[:]) # location

            rt = [None, None, None, None]
            rt[0] = rot[0][0]
            rt[1] = rot[0][1]
            rt[2] = rot[0][2]
            rt[3] = rot[1]
            axa = round_array_no_unit_scale(rt)[:]
            oldlen = len(orientationInterpolators[b].keyValue)
            if oldlen > 0:
                oldaxa = orientationInterpolators[b].keyValue[oldlen-1]
            else:
                oldaxa = None
            if frame == lasttime or oldaxa is None or (oldaxa[0] != axa[0] or oldaxa[1] != axa[1] or oldaxa[2] != axa[2] or oldaxa[3] != axa[3]):
                orientationInterpolators[b].key.append(round_array_no_unit_scale([keyframe_time / key_divider])[:])
                orientationInterpolators[b].keyValue.append([axa[0], axa[1], axa[2], axa[3]])
            b += 1
            dbone.prev_euler = rot
        keyframe_time = keyframe_time + keyframe_length

    scene.frame_set(frame_current)
    if not skip:
        print_console('INFO', "humanoid_root found in bone data")
        nodes.append(positionInterpolators[:])
    # print_console('INFO', f"Writing {len(orientationInterpolators)} interpolators {len(orientationRoutes)} routes.")
    nodes.append(orientationInterpolators[:])

    if not skip:
        print_console('INFO', "humanoid_root found in bone data")
        nodes.append(positionRoutes[:])
    nodes.append(orientationRoutes[:])
    return nodes

def export_TimeSensor():
        scene = bpy.context.scene
        nodes = []
        frame_duration = (1.0 / (scene.render.fps / scene.render.fps_base))
        frame_start = scene.frame_start
        frame_end = scene.frame_end
        frame_current = scene.frame_current
        frame_count = frame_end - frame_start + 1

        key_divider = (frame_count - 1) * frame_count / scene.render.fps 

        print_console('INFO', "Frame count: %d\n" % frame_count)
        print_console('INFO', "Frame duration: %.6f\n" % frame_duration)
        print_console('INFO', "Key divider: %.6f\n" % key_divider)

        time_sensor = TimeSensor(cycleInterval=(frame_duration * (frame_end - frame_current)), loop=True, enabled=True)
        clock_name = substitute("X3DV_Clock")
        setUSEDEF("", clock_name, time_sensor)

        activate_sensor = ProximitySensor(size=[ 1000000, 1000000, 1000000 ]) # there are 2 sensors, one for bones, one for objects
        activate_name = substitute("X3DV_Close")
        setUSEDEF("", activate_name, activate_sensor)
        activate_route = ROUTE(
                fromNode=activate_name,
                fromField="enterTime",
                toNode=clock_name,
                toField="startTime")

        nodes.append(time_sensor)
        nodes.append(activate_sensor)
        nodes.append(activate_route)
        return nodes


def write_obj_interpolators(obj, obj_main_id, matrix, prefix):

    animation_data = obj.animation_data
    nodes = []
    scene = bpy.context.scene
    frame_current = scene.frame_current
    if animation_data:
        name = obj_main_id
        frame_start = scene.frame_start
        frame_end = scene.frame_end
        frame_count = frame_end - frame_start + 1
        frame_duration = (1.0 / (scene.render.fps / scene.render.fps_base))

        key_divider = (frame_count - 1) * frame_count / scene.render.fps 

        frame_range = [frame_current, frame_end]
        clock_name = substitute("X3DV_Clock")
        positionInterpolators = []
        orientationInterpolators = []
        positionRoutes = []
        orientationRoutes = []
        print(f"Creating interpolators for {obj_main_id}")
        pobjname = substitute(obj_main_id)

        posInterp = PositionInterpolator()
        piobjname = substitute(obj_main_id)+"transInterp"
        setUSEDEF("", piobjname, posInterp)
        positionInterpolators.append(posInterp)

        positionRoutes.append(ROUTE(
            fromNode=clock_name,
            fromField="fraction_changed",
            toNode=piobjname,
            toField="set_fraction"))
        positionRoutes.append(ROUTE(
            fromNode=piobjname,
            fromField="value_changed",
            toNode=pobjname,
            toField="translation"))

        rotInterp = OrientationInterpolator()
        oiobjname = ""+substitute(obj_main_id)+"RotInterp"
        setUSEDEF("", oiobjname, rotInterp)
        orientationInterpolators.append(rotInterp)

        orientationRoutes.append(ROUTE(
            fromNode=clock_name,
            fromField="fraction_changed",
            toNode=oiobjname,
            toField="set_fraction"))
        orientationRoutes.append(ROUTE(
            fromNode=oiobjname,
            fromField="value_changed",
            toNode=pobjname,
            toField="rotation"))

        lasttime = range(int(frame_range[0]), int(frame_range[1]) + 1)[-1]
        keyframe_length = (frame_range[1] - frame_range[0]) / bpy.context.scene.render.fps
        keyframe_time = 0

        skip = False
        for frame in range(frame_start - 1, frame_end + 1): # X3D starts at 0, Blender starts at 1
            scene.frame_set(frame)

            loc, rot, scale = matrix.decompose()
            rot = rot.to_axis_angle()
            rot = (*rot[0].normalized(), rot[1])

            lo = [None, None, None]
            lo[0] = loc[0]
            lo[1] = loc[1]
            lo[2] = loc[2]
            pos = round_array_no_unit_scale(lo)[:]
            oldlen = len(positionInterpolators[0].keyValue)
            if oldlen > 0:
                oldpos = positionInterpolators[0].keyValue[oldlen-1]
            else:
                oldpos = None
            if frame == lasttime or oldpos is None or  oldpos[0] != pos[0] or oldpos[1] != pos[1] or oldpos[2] != pos[2]:
                # positionInterpolators[0].key.append(round_array_no_unit_scale([keyframe_time / key_divider])[:])
                # positionInterpolators[0].keyValue.append([pos[0], pos[1], pos[2]]) # location
                pass

            rt = [None, None, None, None]
            rt[0] = rot[0]
            rt[1] = rot[1]
            rt[2] = rot[2]
            rt[3] = rot[3]
            axa = round_array_no_unit_scale(rt)[:]
            oldlen = len(orientationInterpolators[0].keyValue)
            if oldlen > 0:
                oldaxa = orientationInterpolators[0].keyValue[oldlen-1]
            else:
                oldaxa = None
            if frame == lasttime or oldaxa is None or oldaxa[0] != axa[0] or oldaxa[1] != axa[1] or oldaxa[2] != axa[2] or oldaxa[3] != axa[3]:
                # orientationInterpolators[0].key.append(round_array_no_unit_scale([keyframe_time / key_divider])[:])
                # orientationInterpolators[0].keyValue.append([axa[0], axa[1], axa[2], axa[3]])
                pass
            action = animation_data.action
            fcurves = action.fcurves
            locarr = ('X', 'Y', 'Z')
            locind = 0
            rotind = 0
            location = [ None, None, None ]
            axis_angle = [ None, None, None, None ]
            obj.rotation_mode = 'AXIS_ANGLE'
            for fc in fcurves:
                if fc.data_path == 'location':
                    for keyframe in fc.keyframe_points:
                        #print(f"frame {frame} keyframe {round(keyframe.co[0], 0)} time {keyframe_time} axis {locind}")
                        if frame == round(keyframe.co[0], 0) and locind < 3:
                            # print(f"keyframe {keyframe}")
                            print(f"frame {frame} index {locind}")
                            print(f"data path {fc.data_path} axis {locarr[locind]}")
                            print(f"keyframe.co[0] frame {keyframe.co[0]}")
                            print(f"keyframe.co[1] value {keyframe.co[1]}")
                            location[locind] = round(keyframe.co[1], 5)
                    locind = locind + 1
                elif fc.data_path.endswith('rotation_axis_angle'):
                    # print(f"Unhandled data_path {fc.data_path}")
                    for keyframe in fc.keyframe_points:
                        #print(f"frame {frame} {round(keyframe.co[0], 0)} {keyframe_time} axis {rotind}")
                        if frame == round(keyframe.co[0], 0) and rotind < 4:
                            print(f"frame {frame} index {rotind}")
                            print(f"keyframe.co[1] value {keyframe.co[1]}")
                            axis_angle[rotind] = round(keyframe.co[1], 5)
                    rotind = rotind + 1
                elif fc.data_path == 'rotation_euler':
                    # print(f"Handled data_path {fc.data_path}")
                    for keyframe in fc.keyframe_points:
                        if frame == round(keyframe.co[0], 0):
                            r = obj.rotation_euler
                            # print(f"euler {r[:]}") # 0=X, 1=Y, 2=Z
                            aa = r.to_quaternion().to_axis_angle()
                            axis_angle = [aa[0][0],aa[0][1],aa[0][2],aa[1]]
                            # print(f"axis angle {axis_angle[:]}") # 0=W, 1=X, 2=Y, 3=Z
                else:
                    print(f"Unhandled data_path {fc.data_path}")

            if location[0] or location[1] or location[2]:
                positionInterpolators[0].key.append(round(frame/250, 5))
                positionInterpolators[0].keyValue.append(location) # location

            if axis_angle[0] or axis_angle[1] or axis_angle[2] or axis_angle[3]:
                orientationInterpolators[0].key.append(round(frame/250, 5))
                orientationInterpolators[0].keyValue.append(axis_angle) # axis_angle

        #coordinateInterpolator = CoordinateInterpolator(DEF=obj_main_id+"_CI")
        #for fc in fcurves:
        #    if fc.data_path == 'location':
        #        for keyframe in fc.keyframe_points:
        #            coordinateInterpolator.key.append(keyframe.co[0])
        #            for coi in keyframe.co:  # not just 3 numbers in a coordinate
        #                print(f"coi {coi}")
        #                coordinateInterpolator.keyValue.append(keyframe.co[:])
        #coordinateRoute = ROUTE(
        #    fromNode=clock_name,
        #    fromField="fraction_changed",
        #    toNode=obj_main_id+"_CI",
        #    toField="set_fraction")
        #print_console('INFO', "Writing 1 coordinate interpolator, 1 route.")
        #nodes.append(coordinateInterpolator)
        #nodes.append(coordinateRoute)


        #if len(positionInterpolators[0].key) == 2 and \
        #    positionInterpolators[0].key[0] == 0.0 and positionInterpolators[0].key[1] == 1.0 and \
        #    positionInterpolators[0].keyValue[0][0] == positionInterpolators[0].keyValue[1][0] and \
        #    positionInterpolators[0].keyValue[0][1] == positionInterpolators[0].keyValue[1][1] and \
        #    positionInterpolators[0].keyValue[0][2] == positionInterpolators[0].keyValue[1][2]:
        #    print("Equal Pos, Removing")
        #else:
        pifound = False
        for pi in positionInterpolators:
            if len(pi.key) > 0 or len(pi.keyValue) > 0:
                nodes.append(pi)
                pifound = True

        #if len(orientationInterpolators[0].key) == 2 and \
        #        orientationInterpolators[0].key[0] == 0.0 and orientationInterpolators[0].key[1] == 1.0 and \
        #        orientationInterpolators[0].keyValue[0][0] == orientationInterpolators[0].keyValue[1][0] and \
        #        orientationInterpolators[0].keyValue[0][1] == orientationInterpolators[0].keyValue[1][1] and \
        #        orientationInterpolators[0].keyValue[0][2] == orientationInterpolators[0].keyValue[1][2] and \
        #        orientationInterpolators[0].keyValue[0][3] == orientationInterpolators[0].keyValue[1][3]:
        #    print("Equal Ori, Removing")
        #else:
        oifound = False
        for oi in orientationInterpolators:
            if len(oi.key) > 0 or len(oi.keyValue) > 0:
                nodes.append(oi)
                oifound = True


        keyframe_time = keyframe_time + keyframe_length

        if pifound:
            print(f"found {len(positionRoutes)} position routes")
            nodes.append(positionRoutes[:])
        if oifound:
            print(f"found {len(orientationRoutes)} orientation routes")
            nodes.append(orientationRoutes[:])

        # print_console('INFO', f"Writing {len(orientationInterpolators)} interpolators {len(orientationRoutes)} routes.")
    scene.frame_set(frame_current)
    return nodes

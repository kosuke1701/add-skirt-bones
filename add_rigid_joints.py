from collections import defaultdict
import re

import bpy
import math
import mathutils

from mmd_tools import utils
import mmd_tools.core.model as mmd_model
from mmd_tools.core import rigid_body

import numpy as np

# NOTE: Please select armature of MMD model before executing this script.

# NOTE: Row and column indices must start from 0.
NAME_PATTERN = "SB_([0-9]+)_([0-9])"
RIGID_PATTERN = "Rigid_SB_{}_{}"
JOINT_PATTERN = "Joint_SB_{}_{}_{}"
RIGID_THICK = 0.2

def get_model():
    obj = bpy.context.active_object
    root = mmd_model.Model.findRoot(obj)
    rig = mmd_model.Model(root)

    return rig

def find_target_bone_matrix(armature):
    bones = armature.pose.bones

    bone_mat = defaultdict(lambda : {})
    for bone_name in bones.keys():
        m = re.search(NAME_PATTERN, bone_name)
        if m:
            i, j = m.groups()
            bone_mat[int(i)][int(j)] = bones[bone_name]
            
    return bone_mat

def vector_average(vecs):
    return sum(vecs, mathutils.Vector((0.0,0.0,0.0))) / len(vecs)

def normalize(vec):
    return vec / (vec @ vec) ** 0.5

def outer(vec1, vec2):
    return mathutils.Vector((
            vec1[1]*vec2[2] - vec1[2]*vec2[1],
            vec1[2]*vec2[0] - vec1[0]*vec2[2],
            vec1[0]*vec2[1] - vec1[1]*vec2[0]
        ))

def compute_width(x1, x2, vec_Z, origin):
    x1 = x1 - origin
    x2 = x2 - origin
    
    x1 = x1 - (vec_Z @ x1) * vec_Z
    x2 = x2 - (vec_Z @ x2) * vec_Z
    
    vec_X = normalize(x1)
    vec_Y = outer(vec_Z, vec_X)
    
    x11, x12, x21, x22 = x1 @ vec_X, x1 @ vec_Y, x2 @ vec_X, x2 @ vec_Y
    A = mathutils.Matrix(
        [[1, 0, -1, 0],
         [0, 1, 0, -1],
         [x11, x12, 0, 0],
         [0, 0, x21, x22]]
    )

    b = mathutils.Vector([-x11+x21, -x12+x22, 0, 0])
    A.invert()
    
    ans = A @ b
    ans = mathutils.Vector(ans[:2])
        
    return (ans @ ans)**0.5, vec_X

def get_bone_center(bone):
    return (bone.bone.head_local + bone.bone.tail_local) / 2

def compute_height_width(self_bone, right_bone, left_bone, vec_Z, origin):
    height = self_bone.bone.length
    
    width1, vec_X = compute_width(get_bone_center(self_bone), get_bone_center(right_bone), vec_Z, origin)
    width2, _ = compute_width(get_bone_center(self_bone), get_bone_center(left_bone), vec_Z, origin)
    
    center = (self_bone.bone.head_local + self_bone.bone.tail_local) / 2
    
    d = center - origin
    e = self_bone.bone.head_local - self_bone.bone.tail_local
    rot_y = normalize(e)
    rot_x = normalize(d - (d@d)/(d@vec_Z)*vec_Z)
    rot_x = normalize(rot_x - (rot_x@rot_y)*rot_y)
    rot_z = outer(rot_x, rot_y)
    rot = self_bone.bone.matrix_local.copy()
    rot[0][0], rot[1][0], rot[2][0] = rot_x
    rot[0][1], rot[1][1], rot[2][1] = rot_y
    rot[0][2], rot[1][2], rot[2][2] = rot_z
    rot = rot.to_euler("YXZ")
    rot.rotate_axis("X", math.pi/2)
    
    return (width1 + width2) / 2, height, center, rot, (rot_x, rot_y, rot_z)
    

# Get list of skirt bones.
bone_mat = find_target_bone_matrix(bpy.context.active_object)
rig = get_model()

#
NUM_I = len(bone_mat)
NUM_J = len(bone_mat[0])

origin = vector_average([bone_mat[i][0].bone.head_local for i in range(NUM_I)])
vec_Z = vector_average([bone_mat[i][NUM_J-1].bone.head_local for i in range(NUM_I)]) - origin
vec_Z = normalize(vec_Z)

rigid_map = defaultdict(dict)
for i in range(NUM_I):
    for j in range(NUM_J):
        width, height, center, rot, _ = compute_height_width(
            bone_mat[i][j], bone_mat[i+1 if i+1<NUM_I else 0][j], bone_mat[i-1 if i-1 >= 0 else NUM_I-1][j],
            vec_Z, origin
        )
        
        rigid = rig.createRigidBody(
            name = RIGID_PATTERN.format(i,j),
            name_e = RIGID_PATTERN.format(i,j),
            location = center,
            rotation = rot,
            size = mathutils.Vector([RIGID_THICK, width, height/2]),
            shape_type = rigid_body.shapeType("BOX"),
            dynamics_type = 1, #MODE_DYNAMIC
            bone = bone_mat[i][j].name
        )
        rigid_map[i][j] = rigid

for i in range(NUM_I):
    for j in range(NUM_J):
        width, height, center, rot, (rot_x, rot_y, rot_z) = compute_height_width(
            bone_mat[i][j], bone_mat[i+1 if i+1<NUM_I else 0][j], bone_mat[i-1 if i-1 >= 0 else NUM_I-1][j],
            vec_Z, origin
        )
        
        # Vertical
        rig.createJoint(
            name = JOINT_PATTERN.format("V", i, j),
            name_e = JOINT_PATTERN.format("V", i, j),
            location = bone_mat[i][j].bone.head_local,
            rotation = rot,
            rigid_a = rigid_map[i][j-1] if j>0 else None,
            rigid_b = rigid_map[i][j] if j>0 else None,
            maximum_location = mathutils.Vector([0,0,0]),
            minimum_location = mathutils.Vector([0,0,0]),
            maximum_rotation = mathutils.Vector([0,0,0]),
            minimum_rotation = mathutils.Vector([0,0,0]),
            spring_linear = mathutils.Vector([0,0,0]),
            spring_angular = mathutils.Vector([0,0,0])
        )
        
        # Horizontal
        rig.createJoint(
            name = JOINT_PATTERN.format("H", i, j),
            name_e = JOINT_PATTERN.format("H", i, j),
            location = center + rot_z * width,
            rotation = rot,
            rigid_a = rigid_map[i][j],
            rigid_b = rigid_map[i+1 if i+1 < NUM_I else 0][j],
            maximum_location = mathutils.Vector([0,0,0]),
            minimum_location = mathutils.Vector([0,0,0]),
            maximum_rotation = mathutils.Vector([0,0,0]),
            minimum_rotation = mathutils.Vector([0,0,0]),
            spring_linear = mathutils.Vector([0,0,0]),
            spring_angular = mathutils.Vector([0,0,0])
        )
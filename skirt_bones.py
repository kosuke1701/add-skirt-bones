import bpy
import bmesh
import mathutils

from collections import defaultdict

# Every N_STEP_* vertices are selected to create bones.
N_STEP_HORI = 3
N_STEP_VERT = 2
UPPER_DIR = mathutils.Vector((0.0,0.0,1.0))
COL_NAME = "SkirtEmpties"
HEAD_NAME = "S"

# Select cloth mesh.
cloth_obj = bpy.context.selected_objects
assert len(cloth_obj)==1
cloth_obj = cloth_obj[0]
cloth_obj_name = cloth_obj.name

# Construct vertex map.
vertices = cloth_obj.data.vertices
print("Found {} vertices.".format(len(vertices)))

edges = cloth_obj.data.edges
print("Found {} edges.".format(len(edges)))

neighbors = defaultdict(set)
for edge in edges:
    from_v, to_v = edge.vertices
    
    neighbors[from_v].add(to_v)
    neighbors[to_v].add(from_v)

def sort_neighbors(ind):
    lst_nei_z = []
    for nei_v in neighbors[ind]:
        lst_nei_z.append(
            ((cloth_obj.matrix_world @ vertices[nei_v].co).dot(UPPER_DIR), nei_v)
        )
    lst_nei_z = sorted(lst_nei_z, reverse=True)
    return lst_nei_z

# Search top vertices
cur_v = 0
while True:
    cur_z = (cloth_obj.matrix_world @ vertices[cur_v].co).dot(UPPER_DIR)
    
    lst_nei_z = sort_neighbors(cur_v)
    
    top_nei_z, top_nei_v = lst_nei_z[0]
    
    if top_nei_z > cur_z:
        cur_v = top_nei_v
    else:
        break
root_v = cur_v
    
lst_top_v = [cur_v]
while True:
    lst_nei_z = sort_neighbors(cur_v)
    
    cands = [_[1] for _ in lst_nei_z[:2]]
    for cand in cands:
        if cand not in lst_top_v:
            lst_top_v.append(cand)
            cur_v = cand
            break
    else:
        break

print("Found {} columns.".format(len(lst_top_v)))

#for top_v in lst_top_v:
#    print(vertices[top_v].co)

# Create list of vertex columns
lst_columns = []
for top_v in lst_top_v[::N_STEP_HORI]:
    lst_column_v = [top_v]
    cur_v = top_v
    while True:
        lst_nei_zv = sort_neighbors(cur_v)
        next_v = lst_nei_zv[-1][1]
        
        lst_column_v.append(next_v)
        if len(neighbors[next_v]) == 3:
            break
        else:
            cur_v = next_v
    
    lst_columns.append(lst_column_v[::N_STEP_VERT])
    
for column in lst_columns:
    print("-"*20)
    for v in column:
        print(vertices[v].co)

print("Number of selected vertices: {}".format(\
    sum(map(len, lst_columns))))
    
# Add empties to positions of selected vertices (except top ones)
collection = bpy.data.collections.new(COL_NAME)
bpy.context.scene.collection.children.link(collection)
layer_collection = bpy.context.view_layer.layer_collection.children[COL_NAME]
bpy.context.view_layer.active_layer_collection = layer_collection

for i_col, columns in enumerate(lst_columns):
    for i_row, ind_v in enumerate(columns):
        if i_row == 0:
            continue
        
        global_pos = cloth_obj.matrix_world @ vertices[ind_v].co
        
        bpy.ops.object.empty_add(radius=0.1, location=global_pos)
        bpy.context.active_object.name = f"{HEAD_NAME}E_{i_col}_{i_row}"
#        bpy.data.collections[COL_NAME].objects.link(
#            bpy.context.active_object)

# Set selected vertices as parents of the empties.
for i_col, columns in enumerate(lst_columns):
    for i_row, ind_v in enumerate(columns):
        if i_row == 0:
            continue
        bpy.ops.object.select_all(action='DESELECT')
        bpy.data.objects[f"{HEAD_NAME}E_{i_col}_{i_row}"].select_set(True)
        
        bpy.context.view_layer.objects.active = cloth_obj
        
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_mode(type="VERT")
        bpy.ops.mesh.select_all(action="DESELECT")
        
        bpy.ops.object.mode_set(mode="OBJECT")
        cloth_obj.data.vertices[ind_v].select = True
        
        bpy.ops.object.mode_set(mode="EDIT")
        
        bpy.ops.object.vertex_parent_set()
        
        bpy.ops.object.mode_set(mode="OBJECT")
        
# Create new armature & create bones
bpy.ops.object.armature_add()
bpy.context.active_object.name = f"{HEAD_NAME}_skelton"

armature = bpy.context.active_object

bpy.ops.object.mode_set(mode="EDIT")

edit_bones = armature.data.edit_bones
edit_bones.remove(armature.data.edit_bones[0])


for i_col, columns in enumerate(lst_columns):
    for i_row in range(len(columns)-1):
        head_pos = vertices[columns[i_row]].co
        tail_pos = vertices[columns[i_row+1]].co
        
        head_pos, tail_pos = [
            armature.matrix_world.inverted() @\
            cloth_obj.matrix_world @ \
            pos for pos in [head_pos, tail_pos]]
        
        b = edit_bones.new(f"{HEAD_NAME}B_{i_col}_{i_row}")
        b.head = (head_pos.x, head_pos.y, head_pos.z)
        b.tail = (tail_pos.x, tail_pos.y, tail_pos.z)
        
        if i_row > 0:
            b.parent = edit_bones["{}B_{}_{}".format(HEAD_NAME, i_col, i_row-1)]
            
bpy.ops.object.mode_set(mode="POSE")

bones = armature.pose.bones
for i_col, columns in enumerate(lst_columns):
    for i_row in range(len(columns)-1):
        bone = bones[f"{HEAD_NAME}B_{i_col}_{i_row}"]
        
        c = bone.constraints.new("IK")
        #print(type(c))
        #bpy.types.KinematicConstraint
        
        c.iterations = 50
        c.target = bpy.data.objects["{}E_{}_{}".format(HEAD_NAME, i_col, i_row+1)]
        c.chain_count = 1

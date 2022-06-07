from enum import Enum
import numpy as np
import pkphysx as px

ratio = 1
scale = 1
lane_ratio = 1000 * ratio
graph_ratio = 1000 * ratio
wall_height = 5500 / lane_ratio

def trans_plist(plist, f):
    if len(plist) == 0: return plist
    if type(plist[0]) == float:
        return f(plist)
    return [trans_plist(lst, f) for lst in plist]

def mm_to_m(p):
    return (p[0] / graph_ratio, p[1] / graph_ratio)

def m_to_mm(p):
    return (p[0] * graph_ratio, p[1] * graph_ratio)

def lane_trans(p):
    return (p[0] / lane_ratio, p[1] / lane_ratio)

def lane_trans_inv(p):
    return (p[0] * lane_ratio, p[1] * lane_ratio)

def createStringShape(self, coords: list, material: px.Material, is_polygon: bool=True) -> px.Shape:
    n = len(coords) - 1 if is_polygon else len(coords)
    vertices = np.empty((n << 1, 3), dtype=self.dtype)
    indices = np.empty((n << 1, 3), dtype=np.int32)
    a, b, c, d = ( None, )*4
    for i in range(len(coords)-1):
        # print(coords[i], coords[i+1])
        xa, xb = i<<1, (i+1)<<1
        xc, xd = xa+1, xb+1
        if i + 1 == n:
            xb -= (n << 1)
            xd -= (n << 1)
        # print(xa, xb, xc, xd)
        #a, b = np.array([*coords[i], 0.], dtype=self.dtype), np.array([*coords[i+1], 0.], dtype=dtype)
        a, b = np.array([*coords[i], -wall_height], dtype=self.dtype)*scale, np.array([*coords[i+1], -wall_height], dtype=self.dtype)*scale
        c, d = a + np.array([0, 0, 2*wall_height]), b + np.array([0, 0, 2*wall_height])
        vertices[xa:xa+2,:] = [a, c]
        indices[xa] = [xa, xc, xb]
        indices[xc] = [xc, xd, xb]
    if not is_polygon:
        vertices[(n-1)<<1:n<<1,:] = [b, d]
    shape = px.Shape.create_triangle_mesh(vertices, indices, material, is_exclusive=True, scale=1.)
    return shape

def lock_2d(actor):
    # https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Joints.html
    # Unlocking translational degrees of freedom allows the origin point of actor1's constraint frame to move along a subset
    # of the axes defined by actor0's constraint frame. For example, unlocking just the X-axis creates the equivalent of
    # a prismatic joint.
    loc, pose = actor.get_global_pose()
    joint = px.D6Joint(actor, loc)
    joint.set_motion(px.D6Axis.X, px.D6Motion.FREE)
    joint.set_motion(px.D6Axis.Y, px.D6Motion.FREE)
    joint.set_motion(px.D6Axis.SWING2, px.D6Motion.FREE)
    joint.set_kinemic_projection(flag=True, tolerance=0.1)
    actor.get_user_data().plane_joint = joint

class FilterType(Enum):
    All = ~np.uint32(0x0)
    Bound = np.uint32(1)
    Wall = np.uint32(1 << 1)
    LaneNode = np.uint32(1 << 2)
    LaneBody = np.uint32(1 << 3)
    Spot = np.uint32(1 << 4)

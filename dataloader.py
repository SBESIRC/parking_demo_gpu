import json
from json.encoder import INFINITY
import geojson

def __round_to(x, d=4, half=.5):
    return (int(x[0] * d + half) / d, int(x[1] * d + half) / d)

def __calc_lane(lanes, origin, lane_filename=None):
    # print(lanes)
    nodes_dict = {}
    nodes = []
    edges = []
    for lane in lanes:
        coords = lane['coordinates']
        x = __round_to((coords[0][0], coords[0][1]))
        lst = -1
        cur = 0
        if nodes_dict.get(x) is None:
            lst = nodes_dict[x] = len(nodes)
            nodes.append({
                'id': lst,
                'pinned': False,
                'location': [x[0]-origin[0], x[1]-origin[1]]
            })
        else:
            lst = nodes_dict[x]
        for n in coords[1:]:
            x = __round_to((n[0], n[1]))
            if nodes_dict.get(x) is None:
                cur = nodes_dict[x] = len(nodes)
                nodes.append({
                    'id': cur,
                    'pinned': False,
                    'location': [x[0]-origin[0], x[1]-origin[1]]
                })
            else:
                cur = nodes_dict[x]
            edges.append({
                'endings': [lst, cur],
                'is_user_defined': True,
                'distortable': True
            })

    res_lanes = {'vertices': nodes, 'edges': edges}
    if lane_filename is not None:
        with open(lane_filename, 'w') as f:
            json.dump(res_lanes, f)
    return res_lanes


def load_geojson(geo_file, lane_filename=None):
    '''@returns: origin, wall, outer_bound, shear_wall, vertices, edges
    origin: [x, y], wall: Polygon, outer_bound: LineString, shear_wall: LineString
    lanes: {'vertices': [{'id':int, 'pinned':bool, 'location':list(2)}],
    'edges': [{'endings':[int, int], 'is_user_defined':bool, 'distortable':bool}]}}]}'''
    graph = geojson.load(geo_file)['features']
    graph_by_layer = {}

    origin = [INFINITY, INFINITY]

    def min_xy(coords, xy):
        if type(coords[0]) == float:
            return [ min(xy[0], coords[0]), min(xy[1], coords[1]) ]
        else:
            for coord in coords:
                xy = min_xy(coord, xy)
            return xy
    
    def minus_xy(coords, xy):
        if type(coords[0]) == float:
            coords[0] -= xy[0]
            coords[1] -= xy[1]
        else:
            for i in range(len(coords)):
                minus_xy(coords[i], xy)

    for g in graph:
        if g.properties.get('Layer') is None:
            continue
        else: layer = g.properties['Layer']
            
        if graph_by_layer.get(layer) is None:
            graph_by_layer[layer] = []

        def push_block(layer, coords, properties, block_type):
            feature = {
                'properties': properties,
                'coordinates': coords,
                'type': block_type
            }
            graph_by_layer[layer].append(feature)

        if g.geometry['type'] == 'GeometryCollection':
            properties = g.properties
            for i, geometry in enumerate(g.geometry['geometries']):
                p = properties.copy()
                p['id'] = i
                push_block(layer, geometry.coordinates, properties, geometry.type)
        else:
            coords = g.geometry['coordinates']
            push_block(layer, coords, g.properties, g.geometry['type'])

    # WALL
    # 地下室轮廓
    # 用地红线
    # _wall = graph_by_layer.get('WALL', []) + graph_by_layer.get('0', [])
    # _outer_bound = graph_by_layer.get('用地红线', [])
    _wall = []
    for name in ['障碍物边缘', 'Obstacle']:
        if graph_by_layer.get(name) is not None:
            _wall = graph_by_layer[name]
            break
    _outer_bound = []
    for name in ['地库边缘', '地库边界', 'Border']:
        if graph_by_layer.get(name) is not None:
            _outer_bound = graph_by_layer[name]
            break

    # get lane
    lanes = []
    for name in ['车道中心线', 'RoadCenter']:
        if graph_by_layer.get(name) is not None:
            lanes = graph_by_layer[name]
            break

    # print(_outer_bound)

    # 去掉坐标偏移
    for g in _outer_bound:
        origin = min_xy(g['coordinates'], origin)

    # for f in (_wall, _outer_bound, _shear_wall):
    #     for g in f:
    #         minus_xy(g['coordinates'], origin)

    res_lane = __calc_lane(lanes, origin, lane_filename)

    return origin, _wall, _outer_bound, res_lane

def load_config(config_file):
    config = json.load(config_file)
    return config

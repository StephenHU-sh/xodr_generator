import matplotlib
matplotlib.use('TkAgg')

import sys
import os
import math
from collections import deque
import numpy as np
import geojson
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import RectangleSelector
import polygon_item
import fig_manipulator
import xodr_exporter
from geom_utils import WorldBox, first, last, is_almost_the_same_pt, curvature, pt_hash, xxyy2xyxy, xyxy2xxyy, clip_xyxy, dist2

import shapely.geometry as sgeom
from shapely.ops import split
from shapely.geometry import LineString, Point, MultiPoint
from collections import OrderedDict
from disjoint_set import DisjointSet


region = None
fig = None
my_map = None
xodr_filename = ""
georef = ""
cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']

class Separator:
  max_id = -1
  def __init__(self, terminals=set()):
    Separator.max_id += 1
    self.id = Separator.max_id
    self.terminals = terminals
    self.heading = None
    self.base_pts = []

    # for direct junctions
    self.road_short_id = None
    self.road_split_from = None
    self.road_merged_to = None

    # for lane spliting/merging junction
    self.junction = None

  def __repr__(self):
    return f"Separator[{self.road_short_id if self.junction is None else self.junction.set_id}]"


class Junction:
  def __init__(self, id):
    self.id = id
    self.connecting_roads = []

  def add_connecting_road(self, road):
    self.connecting_roads.append(road)

  def debug_print(self):
    print(f"{self}")
    print(f"\tConnecting Roads:")
    for road in self.connecting_roads:
      print((f"\t\t[{road.id[6:]}]"))

  def set_id(self):
    return "|".join(sorted([str(j.short_id) for j in self.connecting_roads]))

  def __repr__(self):
    return f"Junction({self.id})[{self.set_id()}]"


class Lane:
  base_x = 0.0
  base_y = 0.0
  base_z = 0.0

  @classmethod
  def set_base(cls, x, y, z):
    cls.base_x = x
    cls.base_y = y
    cls.base_z = z

  def __init__(self, full_id, id, type, poly=None):
    self.road_id = None
    self.full_id = full_id
    self.type = type
    self.id = id
    self.xodr = None
    self.left_bnd = []
    self.right_bnd = []

    if poly is not None:
      self.poly = [(x-Lane.base_x, y-Lane.base_y) for x,y,z in poly]
      self.poly3d = [(x-Lane.base_x, y-Lane.base_y, z-Lane.base_z) for x,y,z in poly]

      # Split the lane polygon into two boundaries
      for idx, pt in enumerate(poly):
        if pt[0] == 1234.0 and pt[1] == 4567.0:
          break
      self.left_bnd = self.poly[:idx]
      self.right_bnd = list(reversed(self.poly[idx+1:-1]))
      self.update_poly()
    
    self.left_bnd_to_recut = []  # [(x1,y1),(x2,y2),...])
    self.right_bnd_to_recut = []
    self.left_bnd_recut_result = None
    self.right_bnd_recut_result = None

    self.clear_topo()

  def clear_topo(self):
    self.left_neighbors = set()
    self.right_neighbors = set()
    self.predecessors = set()
    self.successors = set()
    self.overlapped = set()
    self.reducing = False
    self.growning = False

  def is_fake(self):
    return self.type == "FAKE"

  def remove_fake_lanes_in_topo(self):
    self.left_neighbors = set([lane for lane in self.left_neighbors if not lane.is_fake()])
    self.right_neighbors = set([lane for lane in self.right_neighbors if not lane.is_fake()])
    self.predecessors = set([lane for lane in self.predecessors if not lane.is_fake()])
    self.successors = set([lane for lane in self.successors if not lane.is_fake()])
    self.overlapped = set([lane for lane in self.overlapped if not lane.is_fake()])
    if self.is_fake():
      self.clear_topo()

  def compute_overlapped_lanes(self):
    # Spliting
    to_del = []
    for lane in self.predecessors:
      if self.road_id == lane.road_id:
        self.overlapped.add(lane)
        lane.overlapped.add(self)
        to_del.append(lane)
      elif self.road.original_id == lane.road.original_id:
        to_del.append(lane)
    for lane in to_del:
      self.predecessors.remove(lane)

    # Merging
    to_del = []
    for lane in self.successors:
      if self.road_id == lane.road_id:
        self.overlapped.add(lane)
        lane.overlapped.add(self)
        to_del.append(lane)
      elif self.road.original_id == lane.road.original_id:
        to_del.append(lane)
    for lane in to_del:
      self.successors.remove(lane)

  def recut_bnd(self, base_pt, heading, start_or_end):
    if self.left_bnd_recut_result is None:
      self.left_bnd_recut_result = self.left_bnd_to_recut
      self.right_bnd_recut_result = self.right_bnd_to_recut
      
    d = 60.0
    heading += math.pi / 2
    separator_line = [(base_pt[0] + d * math.cos(heading), base_pt[1] + d * math.sin(heading))]
    separator_line.append((base_pt[0] - d * math.cos(heading), base_pt[1] - d * math.sin(heading)))

    left_bnd_polyline = sgeom.LineString(self.left_bnd_recut_result)
    right_bnd_polyline = sgeom.LineString(self.right_bnd_recut_result)
    separator_polyline = sgeom.LineString(separator_line)

    result = split(left_bnd_polyline, separator_polyline)
    if len(result.geoms) < 2:
      # Failed to cut the boundary. Show debug lines.
      return [self.left_bnd_to_recut, separator_line]
    left1_pts = [result.geoms[0].xy[0], result.geoms[0].xy[1]]
    left2_pts = [result.geoms[1].xy[0], result.geoms[1].xy[1]]
    left_sep_pt = left_bnd_polyline.intersection(separator_polyline)
    if not isinstance(left_sep_pt, Point):
      return [self.left_bnd_to_recut, separator_line]
    assert(isinstance(left_sep_pt, Point))

    if start_or_end == "start":
      self.left_bnd_recut_result = [(left_sep_pt.x, left_sep_pt.y)] + xxyy2xyxy(left2_pts)
    else: # "end"
      self.left_bnd_recut_result = xxyy2xyxy(left1_pts) + [(left_sep_pt.x, left_sep_pt.y)]

    result = split(right_bnd_polyline, separator_polyline)
    if len(result.geoms) < 2:
      # Failed to cut the boundary. Show debug lines.
      return [self.right_bnd_to_recut, separator_line]
    right1_pts = [result.geoms[0].xy[0], result.geoms[0].xy[1]]
    right2_pts = [result.geoms[1].xy[0], result.geoms[1].xy[1]]
    right_sep_pt = right_bnd_polyline.intersection(separator_polyline)
    if not isinstance(right_sep_pt, Point):
      # Failed to cut the boundary. Show debug lines.
      return [self.right_bnd_to_recut, separator_line]
    assert(isinstance(right_sep_pt, Point))

    if start_or_end == "start":
      self.right_bnd_recut_result = [(right_sep_pt.x, right_sep_pt.y)] + xxyy2xyxy(right2_pts)
    else: # "end"
      self.right_bnd_recut_result = xxyy2xyxy(right1_pts) + [(right_sep_pt.x, right_sep_pt.y)]
    
    return []

  def update_poly(self):
    self.poly = self.left_bnd + list(reversed(self.right_bnd))

  def resample_bnd_linear(self, d):
    self.left_bnd = xodr_exporter.resample_linear(self.left_bnd, d)
    self.right_bnd = xodr_exporter.resample_linear(self.right_bnd, d)

  def debug_print(self, prefix=""):
    print(f"{prefix}Lane[{'%4s' % self.full_id[14:]}]\t", end="")
    left = ['%4s' % lane.full_id[14:] for lane in self.left_neighbors]
    print(f" left: {'%-8s' % str(left)},", end="")
    right = ['%4s' % lane.full_id[14:] for lane in self.right_neighbors]
    print(f" right:{'%-8s' % str(right)},", end="")
    prev = ['%4s' % lane.full_id[14:] for lane in self.predecessors]
    print(f" prev: {'%-16s' % str(prev)},", end="")
    next = ['%4s' % lane.full_id[14:] for lane in self.successors]
    print(f" next: {'%-16s' % str(next)},", end="")
    overlap = ['%4s' % lane.full_id[14:] for lane in self.overlapped]
    print(f" overlapped:{'%-16s' % str(overlap)}")

  def __repr__(self):
    return f"Lane[{self.full_id[6:]}]"

# One-way street with only one lane section
class Road:
  def __init__(self, id):
    self.id = id
    self.original_id = id

    # id => short id
    # 557024174,0,0,10020 => 1740010020
    # 557024174,0,0,20    => 1740000020
    if id.find(",") == -1:
      self.short_id = id
    else:
      sub_ids = [int(s) for s in id.split(",")]
      self.short_id = int((sub_ids[0]-sub_ids[0]//1000*1000)*1e6 + sub_ids[1]*1e5 + sub_ids[2]*1e4 + sub_ids[3])

    self.xodr = None
    self.lanes = OrderedDict()
    self.has_fake_lanes = False
    self.old_ref_line = None
    self.ref_line = None
    self.overlapped_roads = set()
    self.clear_topo()

    # Assume roads are connected in:
    #   ... => (road_a_start, road_a_end) => (road_b_start, road_b_end) => ...
    self.linkage = [set(), set()] # [set([(prev_road_a, "end"), ...]), set([(next_road_b, "start"),...])]
    self.junction = None # For connecting roads of default junctions
    self.in_junction = False

  def add_lane(self, lane):
    lane.road_id = self.id
    lane.road = self
    if lane.is_fake():
      self.has_fake_lanes = True
    self.lanes[lane.id] = lane

  def sort_lanes(self):
    self.lanes = OrderedDict([(k,v) for k, v in reversed(sorted(self.lanes.items()))])

  def clear_topo(self):
    self.predecessor_roads = set()
    self.successor_roads = set()

  def compute_irregular_overlapped_lanes(self):
    # Contacted
    lanes = list(self.lanes.values())
    for idx in range(1, len(lanes)):
      lane_a = lanes[idx-1]
      lane_b = lanes[idx]
      lane_a_left_p1 = lane_a.left_bnd[0]
      lane_a_left_p2 = lane_a.left_bnd[-1]
      lane_a_right_p1 = lane_a.right_bnd[0]
      lane_a_right_p2 = lane_a.right_bnd[-1]
      lane_b_left_p1 = lane_b.left_bnd[0]
      lane_b_left_p2 = lane_b.left_bnd[-1]
      lane_b_right_p1 = lane_b.right_bnd[0]
      lane_b_right_p2 = lane_b.right_bnd[-1]
      head_width_a = math.sqrt(dist2(lane_a_left_p1, lane_a_right_p1))
      tail_width_a = math.sqrt(dist2(lane_a_left_p2, lane_a_right_p2))
      head_width_b = math.sqrt(dist2(lane_b_left_p1, lane_b_right_p1))
      tail_width_b = math.sqrt(dist2(lane_b_left_p2, lane_b_right_p2))
      head_width = math.sqrt(dist2(lane_a_left_p1, lane_b_right_p1))
      tail_width = math.sqrt(dist2(lane_a_left_p2, lane_b_right_p2))
      if lane_a not in lane_b.left_neighbors and lane_b not in lane_a.right_neighbors \
        and (head_width_a + head_width_b > head_width + 0.05 \
        or tail_width_a + tail_width_b > tail_width + 0.05):
        lane_a.overlapped.add(lane_b)
        lane_b.overlapped.add(lane_a)

  def predecessor_overlapped_roads(self):
    roads = list(self.predecessor_roads)
    for road in self.predecessor_roads:
      roads += list(road.overlapped_roads)
    roads.sort(key=lambda r: r.id)
    return roads

  def successor_overlapped_roads(self):
    roads = list(self.successor_roads)
    for road in self.successor_roads:
      roads += list(road.overlapped_roads)
    roads.sort(key=lambda r: r.id)
    return roads

  def backup_ref_line(self):
    self.old_ref_line = self.ref_line

  def build_ref_line(self):
    for lane in self.lanes.values():
      self.ref_line = lane.left_bnd
      # Add more points for a straight line with only 2 points
      if len(self.ref_line) == 2:
        self.ref_line = [(self.ref_line[0][0]*(5-a)/5 + self.ref_line[1][0]*a/5, self.ref_line[0][1]*(5-a)/5 + self.ref_line[1][1]*a/5) for a in range(6)]
      break # left-most lane only

  def resample_ref_line(self, d, method="linear"):
    pts = self.ref_line
    if method == "linear":
      pts = xodr_exporter.resample_linear(pts, d)
    else:
      #pts = xodr_exporter.resample_linear(pts, d)
      pts = xodr_exporter.resample_cubic(pts, self.heading[0], self.heading[1], d)
      if 0: # Show heading normals at tip of reference lines
        line1 = []
        line2 = []
        dd = math.pi / 2
        for d in range(30):
          line1.append((pts[0][0] + d * math.cos(self.heading[0] - dd), pts[0][1] + d * math.sin(self.heading[0] - dd)))
        for d in range(10):
          line2.append((pts[-1][0] + d * math.cos(self.heading[1] - dd), pts[-1][1] + d * math.sin(self.heading[1] - dd)))
        pts = list(reversed(line1)) + pts + line2
    self.ref_line = pts

  def resample_bnd_linear(self, d):
    for lane in self.lanes.values():
      lane.resample_bnd_linear(d)

  def compute_ref_line_bc_derivative(self):
    h1 = math.atan2(self.ref_line[1][1]-self.ref_line[0][1], self.ref_line[1][0]-self.ref_line[0][0])
    h2 = math.atan2(self.ref_line[-1][1]-self.ref_line[-2][1], self.ref_line[-1][0]-self.ref_line[-2][0])
    self.heading = [h1, h2]

  def split_road_with_overlapped_lanes(self):
    new_roads = []
    lanes = list(self.lanes.values())
    lane_idx_base = 0
    old_prev_road_set = None
    old_next_road_set = None
    last_road = None
    overlapped_idx = 1
    while lane_idx_base < len(lanes):
      for lane_idx in range(lane_idx_base, len(lanes)):
        lane = lanes[lane_idx]
        lane_idx_base = lane_idx + 1
        prev_road_set = set([prev.road.original_id for prev in lane.predecessors])
        next_road_set = set([next.road.original_id for next in lane.successors])

        # Reuse old road for the first lane
        if lane_idx == 0:
          # Remove lanes from original road
          self.lanes = OrderedDict()
          self.lanes[lane.id] = lane
          new_road = self
        else:
          # Generate new road id
          # 557024174,0,0,20 => 557024174,0,0,10020
          # 557024174,0,0,20 => 557024174,0,0,20020
          p = self.id.rfind(",")
          new_road_id = self.id[:p+1] + str(int(self.id[p+1:]) + 10000*overlapped_idx)
          overlapped_idx += 1
          new_road = Road(new_road_id)
          new_road.original_id = self.id

        # Use new ref line if road splits otherwise use old ref line and lane offset.
        for lane_k in lanes:
          prev_set = set([prev.road.original_id for prev in lane_k.predecessors]) | prev_road_set
          next_set = set([next.road.original_id for next in lane_k.successors]) | next_road_set
          use_this_ref_line = len(prev_set) == 1 and len(next_set) == 1
          if use_this_ref_line:
            if lane_k != lane:
              fake_lane = Lane(new_road_id + ",-1", "-1", "FAKE")
              fake_lane.left_bnd = lane_k.left_bnd
              fake_lane.right_bnd = lane.left_bnd
              fake_lane.update_poly()
              fake_lane.road = new_road
              new_road.add_lane(fake_lane)
            break

        # Add lane with the same prev/next to the road
        overlap_count = 0
        for j in range(lane_idx, len(lanes)):
          lane_j = lanes[j]
          prev_road_set = set([prev.road.original_id for prev in lane_j.predecessors])
          next_road_set = set([next.road.original_id for next in lane_j.successors])
          has_same_prev_next_road = (old_prev_road_set is None or prev_road_set == old_prev_road_set)
          has_same_prev_next_road = has_same_prev_next_road and (old_next_road_set is None or next_road_set == old_next_road_set)
          old_prev_road_set = prev_road_set
          old_next_road_set = next_road_set
          if len(lane_j.overlapped) > 0:
            overlap_count += 1
          if j == lane_idx or (has_same_prev_next_road and overlap_count <= 1):
            lane_j.road = new_road
            new_road.add_lane(lane_j)
            lane_idx_base = j + 1
          else:
            break
        
        # Update overlapped roads
        if last_road is not None:
          real_lanes = [lane for lane in new_road.lanes.values() if not lane.is_fake()]
          if first(real_lanes) in last(last_road.lanes.values()).overlapped:
            last_road.overlapped_roads.add(new_road)
            new_road.overlapped_roads.add(last_road)

        new_roads.append(new_road)
        last_road = new_road
        break
    
    return new_roads

  def __repr__(self):
    return f"Road[{self.id[6:]}]"

  def debug_print(self):
    from_road_link = ""
    to_road_link = "" 
    for linkage in self.linkage[0]:
      if linkage[1] == "junction":
        from_road_link += f", from Junction[{linkage[0]}]"
      else:
        from_road_link += f", from Road Link[{linkage[0].id[6:]}] {linkage[1]}"
    for linkage in self.linkage[1]:
      if linkage[1] == "junction":
        to_road_link += f", to Junction[{linkage[0]}]"
      else:
        to_road_link += f", to Road Link[{linkage[0].id[6:]}] {linkage[1]}"
    in_junction = f", in Junction[{self.junction.id}]" if self.junction is not None else ""
    prev_roads = f", from Roads[{self.predecessor_roads}]" if len(self.predecessor_roads) > 0 else ""
    next_roads = f", to Roads[{self.successor_roads}]" if len(self.successor_roads) > 0 else ""
    print(f"Road[{'%10s' % self.id[6:]}]{prev_roads}{next_roads}{from_road_link}{to_road_link}{in_junction}")

    for lane in self.lanes.values():
      lane.debug_print("\t\t")


def update_neighbor(lane1, lane2):
  lane1_left_p0 = lane1.left_bnd[0]
  lane1_left_p1 = lane1.left_bnd[-1]
  lane1_right_p0 = lane1.right_bnd[0]
  lane1_right_p1 = lane1.right_bnd[-1]
  lane2_left_p0 = lane2.left_bnd[0]
  lane2_left_p1 = lane2.left_bnd[-1]
  lane2_right_p0 = lane2.right_bnd[0]
  lane2_right_p1 = lane2.right_bnd[-1]

  b1 = is_almost_the_same_pt(lane1_left_p0, lane2_right_p0)
  b2 = is_almost_the_same_pt(lane1_left_p1, lane2_right_p1)
  b3 = is_almost_the_same_pt(lane1_right_p0, lane2_left_p0)
  b4 = is_almost_the_same_pt(lane1_right_p1, lane2_left_p1)
  if b1 and b2:
    lane1.left_neighbors.add(lane2)
    lane2.right_neighbors.add(lane1)
  if b3 and b4:
    lane1.right_neighbors.add(lane2)
    lane2.left_neighbors.add(lane1)


class RoadNetwork:
  def __init__(self, gjson, focused_set=set()):
    self.roads = OrderedDict()
    self.debug_pts = []
    self.default_junctions = []
    self.extra_road_max_id = -1
    self.extra_junction_max_id = -1
    Lane.set_base(0.0, 0.0, 0.0)

    for f in gjson["features"]:
      if f["properties"]["layer"] != "lane":
        continue
      if Lane.base_x == 0.0 and Lane.base_y == 0.0:
        x, y, z = first(f["geometry"]["coordinates"])
        Lane.set_base(x, y, z)

      lane_id = f["properties"]["id"]
      if focused_set and lane_id not in focused_set:
        continue
      lane_type = f["properties"]["lane_type"]
      if lane_type in {"EMERGENCY_LANE"}:
        continue

      pos = lane_id.rfind(",")
      road_id = lane_id[:pos]
      lane_subid = int(lane_id[pos+1:])

      new_lane = Lane(lane_id, lane_subid, lane_type, f["geometry"]["coordinates"])
      if len(new_lane.left_bnd) != 0 and len(new_lane.right_bnd) != 0:
        if road_id not in self.roads:
          self.add_road(Road(road_id))
        self.roads[road_id].add_lane(new_lane)
      else:
        print(f"SKIP problematic lane[{lane_id}].")

  def generate_extra_road_id(self):
    self.extra_road_max_id += 1
    return f"999999999,9,9,{self.extra_road_max_id}"

  def generate_extra_junction_id(self):
    self.extra_junction_max_id += 1
    return f"{8888 + self.extra_junction_max_id}"
    #return f"888888888,8,8,{self.extra_junction_max_id}"

  def add_road(self, road):
    self.roads[road.id] = road

  def update_pt_lane_set(self, pt):
    pt_str = pt_hash(pt)
    assert pt_str in self.pt2lane
    return self.pt2lane[pt_str]

  def all_road_lane_pairs(self):
    for road in self.roads.values():
      for lane in road.lanes.values():
        yield road, lane

  def all_lanes(self):
    for road in self.roads.values():
      for lane in road.lanes.values():
        yield lane

  def update_pt2lane_hash_table(self):
    # Build hash table for boundary points
    self.pt2lane = {}
    for lane in self.all_lanes():
      for pt in lane.poly:
        pt_str = pt_hash(pt)
        if pt_str not in self.pt2lane:
          self.pt2lane[pt_str] = set()
        self.pt2lane[pt_str].add(lane)

  def clear_topo(self):
    for road in self.roads.values():
      road.clear_topo()
      for lane in road.lanes.values():
        lane.clear_topo()

  def compute_lane_topo(self):
    for road in self.roads.values():
      lanes = list(road.lanes.values())
      for i in range(len(lanes)):
        if lanes[i].is_fake():
          continue
        for j in range(i+1, len(lanes)):
          if lanes[j].is_fake():
            continue
          update_neighbor(lanes[i], lanes[j])
        a_hash = pt_hash(lanes[i].left_bnd[0])
        b_hash = pt_hash(lanes[i].right_bnd[0])
        if a_hash != b_hash:
          a = self.update_pt_lane_set(lanes[i].left_bnd[0])
          b = self.update_pt_lane_set(lanes[i].right_bnd[0])
          lanes[i].predecessors = a.intersection(b).difference(set([lanes[i]]))
          for overlapped_road in lanes[i].road.overlapped_roads:
            lanes[i].predecessors -= set(overlapped_road.lanes.values())
        else:
          self.growning = True

        a_hash = pt_hash(lanes[i].left_bnd[-1])
        b_hash = pt_hash(lanes[i].right_bnd[-1])
        if a_hash != b_hash:
          a = self.update_pt_lane_set(lanes[i].left_bnd[-1])
          b = self.update_pt_lane_set(lanes[i].right_bnd[-1])
          lanes[i].successors = a.intersection(b).difference(set([(lanes[i])]))
          for overlapped_road in lanes[i].road.overlapped_roads:
            lanes[i].successors -= set(overlapped_road.lanes.values())
        else:
          self.reducing = True

        lanes[i].remove_fake_lanes_in_topo()
        lanes[i].compute_overlapped_lanes()

  def compute_road_topo(self):
    for road in self.roads.values():
      for lane in road.lanes.values():
        for prev in lane.predecessors:
          road.predecessor_roads.add(prev.road)
        for next in lane.successors:
          road.successor_roads.add(next.road)
      road.compute_irregular_overlapped_lanes()

  def split_road_with_overlapped_lanes(self):
    new_roads = []
    for road in self.roads.values():
      found_lanes_overlapped = False
      for lane in road.lanes.values():
        if len(lane.overlapped) > 0:
          found_lanes_overlapped = True
          break
      if found_lanes_overlapped > 0:
        new_roads += road.split_road_with_overlapped_lanes()
    for new_road in new_roads:
      self.add_road(new_road)
    return len(new_roads) > 0

  def find_common_lane_bnd_pt(self, seps, terminal_type):
    pts_hashmap = {}
    base_pts = []
    for road, start_or_end in seps:
      if start_or_end == terminal_type:
        pt_idx = 0 if terminal_type == "start" else -1
        pt_idx2 = -1 if terminal_type == "start" else 0
        for lane in road.lanes.values():
          pt_left1 = lane.left_bnd[pt_idx]
          pt_left1_hash = pt_hash(pt_left1)
          pt_left2_hash = pt_hash(lane.left_bnd[pt_idx2])
          if pt_left1_hash not in pts_hashmap:
            pts_hashmap[pt_left1_hash] = set()
          elif pt_left2_hash not in pts_hashmap[pt_left1_hash]:
            base_pts.append(pt_left1)
          pts_hashmap[pt_left1_hash].add(pt_left2_hash)
          pt_right1 = lane.right_bnd[pt_idx]
          pt_right1_hash = pt_hash(pt_right1)
          pt_right2_hash = pt_hash(lane.right_bnd[pt_idx2])
          if pt_right1_hash not in pts_hashmap:
            pts_hashmap[pt_right1_hash] = set()
          elif pt_right2_hash not in pts_hashmap[pt_right1_hash]:  # not shared on the other terminal
            base_pts.append(pt_right1)
          pts_hashmap[pt_right1_hash].add(pt_right2_hash)
    return base_pts

  def extend_bnd_by_extrapolation(self, bnd, start_or_end, d):
    if start_or_end == "start":
      heading = math.atan2(bnd[0][1]-bnd[1][1], bnd[0][0]-bnd[1][0])
      bnd.appendleft((bnd[0][0]+math.cos(heading)*d, bnd[0][1]+math.sin(heading)*d))
    else:
      heading = math.atan2(bnd[-1][1]-bnd[-2][1], bnd[-1][0]-bnd[-2][0])
      bnd.append((bnd[-1][0]+math.cos(heading)*d, bnd[-1][1]+math.sin(heading)*d))

  def print_separators(self, seps):
    for sep in seps:
       print(f"{sep.id}: {sep.terminals}")

  def find_lane_terminals_shared_same_directions(self):
    s = DisjointSet()
    separators = []
    for road in self.roads.values():
      found_predecessors = False
      found_successors = False
      for lane in road.lanes.values():
        for lane2 in lane.predecessors:
          end_sep = (lane2.road, "end")
          start_sep = (road, "start")
          s.union(start_sep, end_sep)
          found_predecessors = True
        for lane2 in lane.successors:
          start_sep = (lane2.road, "start")
          end_sep = (road, "end")
          s.union(start_sep, end_sep)
          found_successors = True
      # Add dead-end/start to separation set
      if not found_predecessors:
        separators.append(Separator({(road, "start")}))
      if not found_successors:
        separators.append(Separator({(road, "end")}))

    for terminal_set in s.itersets():
      separators.append(Separator(terminal_set))

    return separators

  def select_road_direction_at_terminals(self, seps):
    for sep in seps:
      start_count = 0
      end_count = 0
      road_with_start_point = None
      road_with_end_point = None
      for road, start_or_end in sep.terminals:
        if start_or_end == "start":
          start_count += 1
          road_with_start_point = road
        else:
          end_count += 1
          road_with_end_point = road
      # Select road direction from road merged to or split from
      if start_count > 1 and end_count == 1:
        sep.road_split_from = road_with_end_point
        sep.road_short_id = road_with_end_point.short_id
        heading = road_with_end_point.heading[1]
      elif start_count == 1 and end_count > 1:
        sep.road_merged_to = road_with_start_point
        sep.road_short_id = road_with_start_point.short_id
        heading = road_with_start_point.heading[0]
      else: # Or select direction of the most straight road
        min_curvature = 999999
        for road, start_or_end in sep.terminals:
          r = road.ref_line
          if len(r) < 3:
            c = 0.0
          else:
            if start_or_end == "start":
              c = curvature(r[0:3])
            else:
              c = curvature(r[-3:])
          if c < min_curvature:
            c = min_curvature
            heading = road.heading[0 if start_or_end == "start" else 1]
      for road, start_or_end in sep.terminals:
        idx = 0 if start_or_end == "start" else 1
        road.heading[idx] = heading
      sep.heading = heading
    return seps

  def determine_separation_line_base_point(self, seps):
    for sep in seps:
      base_pts = []
      # # Find points at lane tips with zero lane width
      # for idx, (road, start_or_end) in enumerate(seps):
      #   pt_idx = 0 if start_or_end == "start" else -1
      #   for lane_id, lane in road.lanes.items():
      #     pt1_hash = pt_hash(lane.left_bnd[pt_idx])
      #     pt2_hash = pt_hash(lane.right_bnd[pt_idx])
      #     if pt1_hash == pt2_hash:
      #       base_pts.append(lane.left_bnd[pt_idx])

      if len(sep.terminals) == 1:
        # Just use terminal point of reference line for roads with no predecessors/successors
        road, start_or_end = first(sep.terminals)
        pt_idx = 0 if start_or_end == "start" else -1
        base_pts += [road.ref_line[pt_idx]]
      else:
        # Find shared start/end points among roads
        base_pts += self.find_common_lane_bnd_pt(sep.terminals, "start")
        base_pts += self.find_common_lane_bnd_pt(sep.terminals, "end")
      assert(len(base_pts) > 0 or len(sep.terminals) == 2)
      if len(base_pts) == 0:
        # Connected without lane merging/splitting
        for idx, (road, start_or_end) in enumerate(sep.terminals):
          pt_idx = 0 if start_or_end == "start" else -1
          if idx == 0:
            base_pt1 = road.ref_line[pt_idx]
          else:
            base_pt2 = road.ref_line[pt_idx]
        base_pts.append(((base_pt1[0]+base_pt2[0])/2, (base_pt1[1]+base_pt2[1])/2))

      if len(base_pts) != 1:
        # For overlapped merging lanes,
        # we usually find 2 or more shared start/end points among roads.
        # Select the point with the largest projected distance that extends the reference line,
        # so that to simplify the recutting with unique cutting position on lane boundaries.
        for road, start_or_end in sep.terminals:
          if len(road.overlapped_roads) > 0:
            pt_idx = 0 if start_or_end == "start" else -1
            heading = road.heading[pt_idx]
            ref_pt = road.ref_line[pt_idx]
            selected_base_pt = None
            for base_pt in base_pts:
              v = (base_pt[0]-ref_pt[0], base_pt[1]-ref_pt[1])
              projected_dist = math.cos(heading)*v[0] + math.sin(heading)*v[1]
              if selected_base_pt is None:
                selected_base_pt = base_pt
                projected_dist_of_selected_base_pt = projected_dist
              else:
                if start_or_end == "start":
                  if projected_dist < projected_dist_of_selected_base_pt:
                    projected_dist_of_selected_base_pt = projected_dist
                    selected_base_pt = base_pt
                else: # "end"
                  if projected_dist > projected_dist_of_selected_base_pt:
                    projected_dist_of_selected_base_pt = projected_dist
                    selected_base_pt = base_pt
            assert(selected_base_pt is not None)
            sep.base_pts = [selected_base_pt]
            break
      else:
        sep.base_pts = base_pts

  def find_nearest_lane_boundary(self, bnd, roads, bnd_start_or_end):
    min_dist = 9999.0
    min_dist_bnd = None
    #lane_str = ""
    for road in roads:
      for lane in road.lanes.values():
        left_bnd2 = lane.left_bnd
        right_bnd2 = lane.right_bnd
        if bnd_start_or_end == "start":
          d1 = dist2(first(bnd), last(left_bnd2))
          d2 = dist2(first(bnd), last(right_bnd2))
        else: # "end"
          d1 = dist2(last(bnd), first(left_bnd2))
          d2 = dist2(last(bnd), first(right_bnd2))
        if d1 < min_dist:
          min_dist_bnd = left_bnd2
          min_dist = d1
          #lane_str = f"{lane} left"
        if d2 < min_dist:
          min_dist_bnd = right_bnd2
          min_dist = d2
          #lane_str = f"{lane} right"
    #print(f"\t\t\t{lane_str} {min_dist}")
    return min_dist_bnd

  def prepare_for_bnd_recut(self, seps):
    roads_to_recut_bnd = set()
    for sep in seps:
      if len(sep.base_pts) != 1:
        continue
      for road, start_or_end in sep.terminals:
        roads_to_recut_bnd.add(road)
    roads_to_recut_bnd = list(roads_to_recut_bnd)
    roads_to_recut_bnd.sort(key=lambda r: r.id)

    # Extend lane boundaries
    d = 10.0  # in meters
    for road in roads_to_recut_bnd:
      for lane_id, lane in road.lanes.items():
        left_bnd = deque(lane.left_bnd)
        right_bnd = deque(lane.right_bnd)

        if lane.is_fake():
          # Extend boundaries of fake lanes.
          # Find nearest boundaries in predecessor_roads/successor_roads
          #print(f"Lane[{lane}] start left extended to:")
          left_prev_bnd = self.find_nearest_lane_boundary(left_bnd, lane.road.predecessor_overlapped_roads(), "start")
          if left_prev_bnd is None:
            # By extrapolation
            self.extend_bnd_by_extrapolation(left_bnd, "start", d)
          else:
            left_bnd.extendleft(clip_xyxy(reversed(left_prev_bnd), d))

          #print(f"Lane[{lane}] start right extended to:")
          right_prev_bnd = self.find_nearest_lane_boundary(right_bnd, lane.road.predecessor_overlapped_roads(), "start")
          if right_prev_bnd is None:
            # By extrapolation
            self.extend_bnd_by_extrapolation(right_bnd, "start", d)
          else:
            right_bnd.extendleft(clip_xyxy(reversed(right_prev_bnd), d))

          #print(f"Lane[{lane}] end left extended to:")
          left_next_bnd = self.find_nearest_lane_boundary(left_bnd, lane.road.successor_overlapped_roads(), "end")
          if left_next_bnd is None:
            # By extrapolation
            self.extend_bnd_by_extrapolation(left_bnd, "end", d)
          else:
            left_bnd.extend(clip_xyxy(left_next_bnd, d))
          #print(f"Lane[{lane}] end right extended to:")
          right_next_bnd = self.find_nearest_lane_boundary(right_bnd, lane.road.successor_overlapped_roads(), "end")
          if right_next_bnd is None:
            # By extrapolation
            self.extend_bnd_by_extrapolation(right_bnd, "end", d)
          else:
            right_bnd.extend(clip_xyxy(right_next_bnd, d))

        else: # not lane.is_fake():
          if len(lane.predecessors) != 0:
              # With boundaries of predecessor lane
            prev_lane = first(lane.predecessors)
            left_bnd.extendleft(clip_xyxy(reversed(prev_lane.left_bnd), d))
            right_bnd.extendleft(clip_xyxy(reversed(prev_lane.right_bnd), d))
          else:
            if len(lane.left_neighbors) and len(first(lane.left_neighbors).predecessors) > 0:
              # With the right boundary of left neighbor's predecessor lane
              neighbor_prev_lane = first(first(lane.left_neighbors).predecessors)
              left_bnd.extendleft(clip_xyxy(reversed(neighbor_prev_lane.right_bnd), d))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(left_bnd, "start", d)
            if len(lane.right_neighbors) and len(first(lane.right_neighbors).predecessors) > 0:
              # with the left boundary of right neighbor's predecessor lane
              neighbor_prev_lane = first(first(lane.right_neighbors).predecessors)
              right_bnd.extendleft(clip_xyxy(reversed(neighbor_prev_lane.left_bnd), d))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(right_bnd, "start", d)
          if len(lane.successors) != 0:
              # With boundaries of predecessor lane
            next_lane = first(lane.successors)
            left_bnd.extend(clip_xyxy(next_lane.left_bnd, d))
            right_bnd.extend(clip_xyxy(next_lane.right_bnd, d))
          else:
            if len(lane.left_neighbors) and len(first(lane.left_neighbors).successors) > 0:
              # With right boundary of left neighbor's successor lane
              neighbor_next_lane = first(first(lane.left_neighbors).successors)
              left_bnd.extend(clip_xyxy(neighbor_next_lane.right_bnd, d))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(left_bnd, "end", d)
            if len(lane.right_neighbors) and len(first(lane.right_neighbors).successors) > 0:
              # With left boundary of right neighbor's successor lane
              neighbor_next_lane = first(first(lane.right_neighbors).successors)
              right_bnd.extend(clip_xyxy(neighbor_next_lane.left_bnd, d))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(right_bnd, "end", d)
        lane.left_bnd_to_recut = left_bnd
        lane.right_bnd_to_recut = right_bnd

  def recut_bnd(self, seps):
    # Cut boundaries with the separation line
    updated_lanes = set()
    for sep in seps:
      if len(sep.base_pts) == 1:
        for road, start_or_end in sep.terminals:
          for lane_id, lane in road.lanes.items():
            if len(self.debug_pts) != 0:
              continue  # skip remaining lanes on errors
            self.debug_pts = lane.recut_bnd(sep.base_pts[0], sep.heading, start_or_end)
            updated_lanes.add(lane)

    # Move results from temp var to the final lane.left_bnd/right_bnd
    for lane in updated_lanes:
      lane.left_bnd = lane.left_bnd_recut_result
      lane.right_bnd = lane.right_bnd_recut_result
      lane.left_bnd_recut_result = None
      lane.right_bnd_recut_result = None

  def refine_lane_terminals(self):
    # Tweak terminals of lane boundaries
    seps = self.find_lane_terminals_shared_same_directions()
    # seps: [
    #   (
    #         heading,
    #         base_pts,
    #         {
    #           (road_a, "end"),
    #           (road_b, "start"), ...
    #         }
    #   )
    #   (...),
    #   ...
    # ]
    self.print_separators(seps)
    self.select_road_direction_at_terminals(seps)
    self.determine_separation_line_base_point(seps)
    self.prepare_for_bnd_recut(seps)
    self.recut_bnd(seps)
    return seps

  def update_road_junction_set(self, connecting_roads, road_junction_set):
    for idx, road in enumerate(connecting_roads):
      road.in_junction = True
      if idx != 0:
        road_junction_set.union(road, connecting_roads[idx-1])

  def build_default_junctions(self, seps):
    road_junction_set = DisjointSet()
    for sep in seps:
      if sep.road_split_from is not None:
        # From spliting roads
        roads = [road for road, start_or_end in sep.terminals if road != sep.road_split_from]
        self.update_road_junction_set(roads, road_junction_set)
      elif sep.road_merged_to is not None:
        # From merging roads
        roads = [road for road, start_or_end in sep.terminals if road != sep.road_merged_to]
        self.update_road_junction_set(roads, road_junction_set)
      elif len(sep.terminals) > 2:
        # From overlapped roads
        for road, start_or_end in sep.terminals:
          if len(road.overlapped_roads) > 0:
            roads = [road_b for road_b, start_or_end_b in sep.terminals if start_or_end == start_or_end_b]
            self.update_road_junction_set(roads, road_junction_set)

    for road_id, road in self.roads.items():
      if not road.in_junction:
        continue
      for prev_road in road.predecessor_roads:
        if prev_road.in_junction:
          road_junction_set.union(road, prev_road)
      for next_road in road.successor_roads:
        if next_road.in_junction:
          road_junction_set.union(road, next_road)

    for connecting_roads in road_junction_set.itersets():
      this_junction = Junction(self.generate_extra_junction_id())
      self.default_junctions.append(this_junction)
      for road in connecting_roads:
        road.junction = this_junction
        this_junction.add_connecting_road(road)  

  def connecting_road_routes(self, lane, base_route=[]):
    # Compute all routes of connecting roads recursively
    if lane.road.junction is None:
      return [base_route]
    new_routes = []
    base_route = base_route + [lane]
    for next_lane in lane.successors:
      new_routes += self.connecting_road_routes(next_lane, base_route)
    return new_routes if len(new_routes) > 0 else [base_route]

  def build_road_from_route(self, route, road_id, road_map, lane_map):
    new_lane = Lane(str(road_id)+",0", 1, "JUNCTION_ROAD")
    new_road = Road(road_id)

    # Add a full fake lane if any lane has a fake lane neighbor.
    # And there's no gaps between their leftmost boundaries.
    has_fake_lane = np.any(max([lane.road.has_fake_lanes for lane in route]))
    if has_fake_lane:
      left_bnd = []
      right_bnd = []
      found_gaps_on_fake_lanes = False
      for idx, lane in enumerate(route):
        leftmost_lane = first(lane.road.lanes.values())
        if len(left_bnd) > 0:
          d = dist2(left_bnd[-1], leftmost_lane.left_bnd[0])
          if d > 0.05 * 0.05:
            found_gaps_on_fake_lanes = True
            break
        left_bnd += leftmost_lane.left_bnd
        right_bnd += lane.left_bnd
      if not found_gaps_on_fake_lanes:
        new_fake_lane = Lane(str(road_id) + ",-1", -1, "FAKE")
        new_fake_lane.left_bnd = left_bnd
        new_fake_lane.right_bnd = right_bnd
        new_fake_lane.update_poly()
        new_road.add_lane(new_fake_lane)

    # Build the new lane
    for idx, lane in enumerate(route):
      new_lane.left_bnd += lane.left_bnd
      new_lane.right_bnd += lane.right_bnd

      # build road terminal mapping table
      k = (lane.road, "start")
      if idx == 0:
        if k not in road_map:
          road_map[k] = []
        road_map[k].append((new_road, "start"))
      else:
        road_map[k] = None  # sep to del
      k = (lane.road, "end")
      if idx == len(route)-1:
        if k not in road_map:
          road_map[k] = []
        road_map[k].append((new_road, "end"))
      else:
        road_map[k] = None  # sep to del

      # build lane terminal mapping table
      if idx == 0:
        k_lane = (lane, "start")
        if k_lane not in lane_map:
          lane_map[k_lane] = []
        lane_map[k_lane].append((new_lane, "start"))
      if idx == len(route)-1:
        k_lane = (lane, "end")
        if k_lane not in lane_map:
          lane_map[k_lane] = []
        lane_map[k_lane].append((new_lane, "end"))

    new_lane.update_poly()
    new_lane.predecessors = route[0].predecessors
    new_lane.successors = route[-1].successors

    new_road.add_lane(new_lane)
    new_road.predecessor_roads = route[0].road.predecessor_roads
    new_road.successor_roads = route[-1].road.successor_roads
    new_road.heading = [route[0].road.heading[0], route[-1].road.heading[1]]
    new_road.build_ref_line()
    new_road.resample_ref_line(5.0)
    new_road.compute_ref_line_bc_derivative()
    return new_road

  def merge_junctions(self, seps):
    print("Merging Junctions...")
    junction_set = DisjointSet()
    new_roads = []
    road_map = {} # mapping between old and new road terminals
    lane_map = {} # mapping between old and new lane terminals
    for junction in self.default_junctions:
      for connecting_road in junction.connecting_roads:
        for next_road in connecting_road.successor_roads:
          if next_road.junction is not None:
            # print(f"{connecting_road.id} => {next_road.id}")
            # print(f"Junction [{connecting_road.junction.id}] + Junction [{next_road.junction.id}]")
            junction_set.union(connecting_road.junction, next_road.junction)
    for junctions_to_merge in junction_set.itersets():
      junctions_to_merge = list(junctions_to_merge)
      junctions_to_merge.sort(key=lambda j: j.id)
      #print(f"To merge: {junctions_to_merge}")
      routes = []
      for j in junctions_to_merge:
        for connecting_road in j.connecting_roads:
          is_incoming_connecting_road = np.all([prev_road.junction is None for prev_road in connecting_road.predecessor_roads])
          if not is_incoming_connecting_road:
            continue
          for lane in connecting_road.lanes.values():
            if lane.is_fake():
              continue
            routes += self.connecting_road_routes(lane)
      new_junction = Junction(self.generate_extra_junction_id())
      for route in routes:
        extra_road_id = self.generate_extra_road_id()
        #print([lane.full_id[14:] for lane in route], f"=> Road[{extra_road_id}]")
        new_road = self.build_road_from_route(route, extra_road_id, road_map, lane_map)
        new_road.junction = new_junction
        #new_road.debug_print()
        self.add_road(new_road)
        new_junction.add_connecting_road(new_road)
        new_roads.append(new_road)

      self.default_junctions.append(new_junction)
      for j in junctions_to_merge:
        if j in self.default_junctions:
          self.default_junctions.remove(j)
        for road in j.connecting_roads:
          del self.roads[road.id]

    # Replace merged roads with new connecting roads in separators
    sep_id_to_del = []
    for sep_idx, sep in enumerate(seps):
      terminal_to_del = []
      new_terminals = []
      for terminal in sep.terminals:
        if terminal in road_map:
          mapped_terminals = road_map[terminal]
          if mapped_terminals is None:
            sep_id_to_del.append(sep_idx)
            break
          else:
            terminal_to_del.append(terminal)
            new_terminals += mapped_terminals
      if len(terminal_to_del) > 0:
        sep.junction = first(new_terminals)[0].junction
      for t in terminal_to_del:
        sep.terminals.remove(t)
      for t in new_terminals:
        sep.terminals.add(t)
    for idx in reversed(sep_id_to_del):
      del seps[idx]

    # Replace merged roads with new connecting roads in predecessor/successor roads
    for (road, start_or_end), new_terminals in road_map.items():
      if new_terminals is None:
        continue
      if start_or_end == "start":
        assert(len(road.predecessor_roads) <= 1)
        if len(road.predecessor_roads) == 1:
          prev_road = first(road.predecessor_roads)
          if road in prev_road.successor_roads:
            prev_road.successor_roads.remove(road)
            for t in new_terminals:
              prev_road.successor_roads.add(t[0])
      else: # "end"
        assert(len(road.successor_roads) <= 1)
        if len(road.successor_roads) == 1:
          next_road = first(road.successor_roads)
          if road in next_road.predecessor_roads:
            next_road.predecessor_roads.remove(road)
            for t in new_terminals:
              next_road.predecessor_roads.add(t[0])

    # Replace merged lanes with new connecting lanes in predecessor/successor
    for (lane, start_or_end), new_terminals in lane_map.items():
      if start_or_end == "start":
        for prev_lane in lane.predecessors:
          if lane in prev_lane.successors:
            prev_lane.successors.remove(lane)
            for t in new_terminals:
              prev_lane.successors.add(t[0])
      else: # "end"
        for next_lane in lane.successors:
          if lane in next_lane.predecessors:
            next_lane.predecessors.remove(lane)
            for t in new_terminals:
              next_lane.predecessors.add(t[0])

  def build_road_connections(self, seps):
    for sep in seps:
      if len(sep.terminals) == 2 and sep.junction is None:
        # Road links between normal roads
        (road_a, start_or_end_a), (road_b, start_or_end_b) = list(sep.terminals)
        if start_or_end_a != start_or_end_b:
          pt_a_idx = 0 if start_or_end_a == "start" else -1
          pt_b_idx = 0 if start_or_end_b == "start" else -1
          road_a.linkage[pt_a_idx].add((road_b, start_or_end_b))
          road_b.linkage[pt_b_idx].add((road_a, start_or_end_a))
          print(f"Link: Road [{road_a.id[6:]}] {start_or_end_a} and Road [{road_b.id[6:]}] {start_or_end_b}")
      elif len(sep.terminals) != 1:
        # Road links between incoming/outcoming and normal roads
        for road_a, start_or_end_a in sep.terminals:
          for road_b, start_or_end_b in sep.terminals:
            if start_or_end_a != "end" or start_or_end_b != "start":
              continue
            pt_a_idx = 0 if start_or_end_a == "start" else -1
            pt_b_idx = 0 if start_or_end_b == "start" else -1

            if sep.junction is None:
              # For direct junctions:
              junction_id = sep.road_short_id
            else:
              # For default junctions:
              junction_id = sep.junction.id

            if road_a.junction is not None:
              # connecting roads
              if road_a in road_b.predecessor_roads:
                road_a.linkage[pt_a_idx].add((road_b, start_or_end_b))
                print(f"Link: Road [{road_a.id[6:]}] {start_or_end_a} and Road [{road_b.id[6:]}] {start_or_end_b}")
            else:
              # incoming/outcoming roads
              road_a.linkage[pt_a_idx].add((junction_id, "junction"))
              print(f"Link: Road [{road_a.id[6:]}] {start_or_end_a} and Junction [{junction_id}]")

            if road_b.junction is not None:
              # connecting roads
              if road_a in road_b.predecessor_roads:
                road_b.linkage[pt_b_idx].add((road_a, start_or_end_a))
                print(f"Link: Road [{road_a.id[6:]}] {start_or_end_a} and Road [{road_b.id[14:]}] {start_or_end_b}")
            else:
              # incoming/outcoming roads
              road_b.linkage[pt_b_idx].add((junction_id, "junction"))
              print(f"Link: Road [{road_b.id[6:]}] {start_or_end_b} and Junction [{junction_id}]")
  
  def build_lane_info(self, preview):
    for road_id, road in self.roads.items():
      road.sort_lanes()
    self.update_pt2lane_hash_table()
    self.compute_lane_topo()
    self.compute_road_topo()

    #return
    if preview:
      return

    if self.split_road_with_overlapped_lanes():
      self.clear_topo()
      self.update_pt2lane_hash_table()
      self.compute_lane_topo()
      self.compute_road_topo()

    for road_id, road in self.roads.items():
      road.build_ref_line()
      road.resample_ref_line(5.0)
      road.compute_ref_line_bc_derivative()

    #return
    # Lane shape    
    seps = self.refine_lane_terminals()
    #return

    # Junctions
    self.build_default_junctions(seps)
    # self.print_separators(seps)
    # for junction in self.default_junctions:
    #   junction.debug_print()
    # print("################################")
    self.merge_junctions(seps)
    # for junction in self.default_junctions:
    #   junction.debug_print()
    # self.print_separators(seps)
    # print("################################")

    # Road Links
    self.build_road_connections(seps)

    # Update lane poly after boundaries updated
    for lane in self.all_lanes():
      lane.update_poly()

    # Rebuild reference line with refined lane boundaries
    for road in self.roads.values():
      road.backup_ref_line()
      road.build_ref_line()
      road.resample_ref_line(3.0)
      road.resample_ref_line(0.1, "cubic")  
      road.resample_bnd_linear(2.0)
      for lane_id, lane in road.lanes.items():
        lane.update_poly()

  def debug_print(self):
    for road in self.roads.values():
      road.debug_print()
    for junction in self.default_junctions:
      junction.debug_print()


def get_polys_in_region():
  selected = set()
  region_min_pt = np.array([region[0], region[1]])
  region_max_pt = np.array([region[2], region[3]])  
  for poly in polys:
    r = np.logical_and(region_min_pt <= poly.xy, poly.xy <= region_max_pt)
    intersected = np.any(np.all(r, axis=1))
    if intersected:
      selected.add(poly)
  return selected

def region_select_callback(eclick, erelease):
  'eclick and erelease are the press and release events'
  global region
  x1, y1 = eclick.xdata, eclick.ydata
  x2, y2 = erelease.xdata, erelease.ydata
  region = (x1, y1, x2, y2)
  #print("(%3.2f, %3.2f) --> (%3.2f, %3.2f)" % (region))
  #print(" The button you used were: %s %s" % (eclick.button, erelease.button))
  polygon_item.PolygonInteractor.current_selection_set = get_polys_in_region()
  #toggle_selector.RS.set_active(False)
  fig.canvas.draw()
  #fig.canvas.flush_events()

def toggle_selector(event):
  if event.key == "enter":
    global focused_set
    for selected_poly in polygon_item.PolygonInteractor.selection_set:
      focused_set.add(selected_poly.lane.full_id)
    plt.close()

  elif event.key == "a":
    selected = get_polys_in_region()
    polygon_item.PolygonInteractor.selection_set.update(selected)
    polygon_item.PolygonInteractor.current_selection_set = set()
    fig.canvas.draw()
    fig.canvas.flush_events()
  elif event.key == "d":
    selected = get_polys_in_region()
    polygon_item.PolygonInteractor.selection_set.difference_update(selected)
    polygon_item.PolygonInteractor.current_selection_set = set()
    fig.canvas.draw()
    fig.canvas.flush_events()
  elif event.key == "e":
    xodr_exporter.export(xodr_filename, my_map, Lane.base_x, Lane.base_y, "navinfo", georef)

def on_pick(event):
  if event.mouseevent.key != 'control' or event.mouseevent.button != 1:
    return
  polygon_item.PolygonInteractor.picked = event.artist
  print(f"Lane[{event.artist.lane.full_id}] {event.artist.lane.type} Road[{event.artist.lane.road.id}] got picked.")
  fig.canvas.draw()
  fig.canvas.flush_events()

def draw_lanes(my_map, ax):
  # Draw lane polygon, and boundary on the selected lane
  polys = []
  polys2 = []
  color_idx = 0
  for lane in my_map.all_lanes():
    if lane.is_fake():
      continue
    xxyy = xyxy2xxyy(lane.poly)

    poly = Polygon(np.column_stack(xxyy), animated=True, color = (0,0,0,0))
    ax.add_patch(poly)
    poly.lane = lane
    polys.append(poly)

    if 0:
      # Draw extended boundaries.
      left_xx = [x for x,y in lane.left_bnd_to_recut]
      left_yy = [y for x,y in lane.left_bnd_to_recut]
      right_xx = [x for x,y in lane.right_bnd_to_recut]
      right_yy = [y for x,y in lane.right_bnd_to_recut]
      p = polygon_item.PolygonInteractor(ax, poly, [left_xx, left_yy], [right_xx, right_yy])
    else:
      p = polygon_item.PolygonInteractor(ax, poly, xyxy2xxyy(lane.left_bnd), xyxy2xxyy(lane.right_bnd))
    
    c = matplotlib.colors.to_rgb(cycle[color_idx])
    color_idx = (color_idx + 1) % len(cycle)
    p.my_color = (c[0], c[1], c[2], 0.95)
    p.my_color2 = (c[0], c[1], c[2], 0.8)

    polys2.append(sgeom.Polygon(lane.poly)) # for filting centerlines
    WorldBox.update(*xxyy)
  return polys, polys2

def draw_ref_lines(my_map):
  for road in my_map.roads.values():
    if road.ref_line:
      pts = road.ref_line
      if 0:
        # Show heading directions at tip of reference lines
        line1 = []
        line2 = []
        for d in range(10):
          line1.append((pts[0][0] - d * math.cos(road.heading[0]), pts[0][1] - d * math.sin(road.heading[0])))
        for d in range(10):
          line2.append((pts[-1][0] + d * math.cos(road.heading[1]), pts[-1][1] + d * math.sin(road.heading[1])))
        pts = list(reversed(line1)) + pts + line2
      pts = xyxy2xxyy(pts)
      plt.plot(pts[0], pts[1], "-*")
      #plt.plot(road.old_ref_line[0], road.old_ref_line[1], "-x")

def draw_debug_pts(my_map):
  # Draw debug points/lines
  for polyline in my_map.debug_pts:
    xyxy = xyxy2xxyy(polyline)
    plt.plot(xyxy[0], xyxy[1], "-o", markersize=10)

def draw_centerlines(g, focused_set):
  for f in g["features"]:
    if f["properties"]["layer"] != "center_line":
      continue
    xx = [x - Lane.base_x for x,y,z in f["geometry"]["coordinates"]]
    yy = [y - Lane.base_y for x,y,z in f["geometry"]["coordinates"]]
    lane_id = f["properties"]["id"]
    if len(focused_set) > 0 and lane_id not in focused_set:
      continue
    plt.plot(xx, yy, "--")

def register_event_handlers(ax, preview):
  # handle lane picking, selection and keyboard events
  fig.canvas.mpl_connect('pick_event', on_pick)
  if preview:
    # drawtype is 'box' or 'line' or 'none'
    toggle_selector.RS = RectangleSelector(ax, region_select_callback,
      useblit=True, button=[1],  # don't use middle button
      minspanx=5, minspany=5, spancoords='pixels', interactive=False)
    def update_selected_region(event):
      if toggle_selector.RS.active:
        toggle_selector.RS.update()
    plt.connect('draw_event', update_selected_region)
  plt.connect('key_press_event', toggle_selector)

def run(geojson_file, focused_set2=set(), preview=True, export=False, georef=""):
  global fig, polys, focused_set, my_map, xodr_filename
  focused_set = focused_set2
  WorldBox.clear()
  fig, ax = plt.subplots()
  pan_zoom = fig_manipulator.PanAndZoom(fig, scale_factor=1.6)
  g = geojson.load(open(geojson_file))

  my_map = RoadNetwork(g, focused_set)
  my_map.build_lane_info(preview)
  my_map.debug_print()

  polys, poly_filters = draw_lanes(my_map, ax)
  draw_ref_lines(my_map)
  draw_debug_pts(my_map)
  draw_centerlines(g, focused_set)

  # Keep aspect ratio to 1:1 in meters
  if os.name == 'nt':
    WorldBox.update_fig_range(ax, 1000, 500)
  else:
    WorldBox.update_fig_range(ax, 1600, 800)

  register_event_handlers(ax, preview)

  focused_set = set()
  xodr_filename = geojson_file.replace(".json", "_low.xodr")
  if export:
    xodr_exporter.export(xodr_filename, my_map, Lane.base_x, Lane.base_y, "navinfo", georef)
  else:
    plt.show()
  return focused_set

if 1:
  if __name__ == '__main__':
    if len(sys.argv) < 2:
      print("Usage: python plot_lane_boundaries.py input.json [--preview]")
      exit(0)
    geojson_path = sys.argv[1]
    georef = "3 | 0 | IGS& 4 | 1 | ENU, 117.285684663802, 36.722913114354,  0"
    if len(sys.argv) > 2 and sys.argv[2] == "--preview":
      focused_set = run(geojson_path, {}, preview=True)
      if focused_set:
        ret = run(geojson_path, focused_set, preview=False, export=True, georef=georef)
    else:
      ret = run(geojson_path, {}, preview=False, export=True, georef=georef)
      print(ret)
    exit(0)

geojson_files = [
  # ("A", "0eca7058-c239-41f3-9f06-8a1243fa2063.json", "3 | 0 | IGS& 4 | 1 | ENU, 121.25589706935,  31.1956300958991, 0"),
  # ("B", "94eeaa34-796c-46d2-89bd-4099f7e70cfc.json", "3 | 0 | IGS& 4 | 1 | ENU, 121.25589706935,  31.1956300958991, 0"), # split ref line not smoothed
  # ("C", "ee2dcc13-a190-48b3-b93f-fc54e2dd9c65.json", "3 | 0 | IGS& 4 | 1 | ENU, 117.285684663802, 36.722913114354,  0"), # Fixed. overlap related. topo: 557392309,0,0,43,1 => 557392309,0,0,14,2
  # ("D", "e2b2f2dc-2436-4870-bb8b-ad5db9db1319.json", "3 | 0 | IGS& 4 | 1 | ENU, 119.01238177903,  34.8047443293035, 0"), # Fixed. topo 557392309,0,0,43,1 => 557392309,0,0,14,2
  
  # ("E", "d6661a91-73af-43fc-bb6b-72bb6b1a2217.json", "3 | 0 | IGS& 4 | 1 | ENU, 121.2231055554,   28.8839460443705, 0"), # Fixed. 4 overlapped lanes in one road. 557004510,0,0,3,4-1, assert(len(lanes_overlapped) == 2)
  # ("F", "3db742cb-855d-4c4f-9f1f-1b6ff3621050.json", "3 | 0 | IGS& 4 | 1 | ENU, 118.727868469432, 34.9886784050614, 0"), # Fixed. connecting road with 2 next roads: 806010035 => {806000036, 806000037}
  # ("G", "75067911-549b-4604-8021-3ebc965cd57b.json", "3 | 0 | IGS& 4 | 1 | ENU, 119.01238177903,  34.8047443293035, 0"), # Fixed. regression: successor: 557371806,0,0,10035,  806000036,   806000037
  # ("H", "dfdafe92-be1a-41c7-a281-ad92d5a94085.json", "3 | 0 | IGS& 4 | 1 | ENU, 121.257908642292, 31.1970074102283, 0"), # Fixed. regression: successor: 557371806,0,0,10035,  806000036,   806000037
  # ("I", "099f151a-d366-4afd-b6ce-b45f6c8b088d.json", "3 | 0 | IGS& 4 | 1 | ENU, 121.254792921245, 31.1981098819524, 0"), # Fixed. lane shape # ref line cut
  # ("J", "8ae44542-62df-4e77-913c-f6ed40c8642a.json", "3 | 0 | IGS& 4 | 1 | ENU, 117.746589006856, 31.7915460281074, 0"), # Fixed. lane shape # ref line cut
  # ("K", "e4b90479-6c46-4674-9870-224beacd90e0.json", "3 | 0 | IGS& 4 | 1 | ENU, 114.40930718556,  30.8577782101929, 0"), # Fixed. irregular overlapped lanes # ref line cut # 556940257,0,0,27, 556940257,0,0,43, 556940257,0,0,10027 assert(len(base_pts) > 0 or len(sep.terminals) == 2)
  
  
#  ("L", "e50db561-c209-4172-bef9-83469a8b215a.json", "3 | 0 | IGS& 4 | 1 | ENU, 120.018651299179, 30.5425742268562, 0"), # overlapped lane
#  ("L", "f853c656-301d-4c45-891f-da87c6e1a52c.json", "3 | 0 | IGS& 4 | 1 | ENU, 120.053409794345, 30.5497380718589, 0"), # cut line
#  ("L", "35cc301a-e17b-4ef2-8e6a-a11bfe86ddde.json", "3 | 0 | IGS& 4 | 1 | ENU, 119.777727825567, 30.7557820994407, 0"), # lane boundary with only 2 pts.
 ("L", "da13bf90-d782-42e3-bf22-42f9546ec0ac.json", "3 | 0 | IGS& 4 | 1 | ENU, 118.718698415905, 32.6414504740387, 0"),




  ]
focused_set = {}

# case A, split overlapped lanes with some non-overlap lane in the same road
#focused_set = {'557024172,0,0,29,2', '557024172,0,0,26,3', '557024172,0,0,26,1', '557024172,0,0,38,2', '557024172,0,0,28,1', '557024172,0,0,29,1', '557024172,0,0,27,1', '557024172,0,0,28,2', '557024172,0,0,27,2', '557024172,0,0,30,1', '557024172,0,0,38,1', '557024172,0,0,30,0', '557024172,0,0,26,2'}
# case A, lane width resampling
#focused_set = {'557024172,0,0,52,1', '557024172,0,0,45,1', '557024172,0,0,52,2', '557024172,0,0,51,1', '557024172,0,0,51,2'}
# case C, failed to build a fake lane with connecting route in the merged junction
#focused_set = {'557392309,0,0,15,3', '557392309,0,0,15,2', '557392309,0,0,43,1', '557392309,0,0,14,1', '557392309,0,0,15,1', '557392309,0,0,47,1', '557392309,0,0,68,2', '557392309,0,0,68,1', '557392309,0,0,45,1', '557392309,0,0,47,0', '557392309,0,0,14,2', '557392309,0,0,44,1'}
# case D, topo issue for new roads from routes in junction:
#focused_set = {'557392309,0,0,15,3', '557392309,0,0,15,1', '557392309,0,0,15,2', '557392309,0,0,68,2', '557392309,0,0,45,1', '557392309,0,0,68,1', '557392309,0,0,47,1', '557392309,0,0,44,1', '557392309,0,0,14,2', '557392309,0,0,47,0', '557392309,0,0,14,1', '557392309,0,0,43,1'}
# case E, 4 overlapped lanes
#focused_set = {'557004510,0,0,9,2', '557004510,0,0,9,1', '557004510,0,0,3,1', '557004510,0,0,46,2', '557004510,0,0,44,1', '557004510,0,0,3,3', '557004510,0,0,8,2', '557004510,0,0,3,4', '557004510,0,0,46,1', '557004510,0,0,7,1', '557004510,0,0,7,0', '557004510,0,0,44,2', '557004510,0,0,3,2', '557004510,0,0,8,1'}
# case E, issue in extending boundaries of fake lanes
#focused_set = {'557004510,0,0,26,2', '557004510,0,0,2,0', '557004510,0,0,5,2', '557004510,0,0,43,1', '557004510,0,0,6,1', '557004510,0,0,6,2', '557004510,0,0,43,2', '557004510,0,0,10,2', '557004510,0,0,5,3', '557004510,0,0,4,2', '557004510,0,0,26,1', '557004510,0,0,5,4', '557004510,0,0,10,1', '557004510,0,0,4,1', '557004510,0,0,5,1', '557004510,0,0,6,3', '557004510,0,0,2,1'}
# case E, range=4000, 557004509,0,0,6,4 only 2 point for left/right_bnd
#focused_set = {}
# case F, split connecting road with 2 next roads
#focused_set = {'557371806,0,0,37,1', '557371806,0,0,44,1', '557371806,0,0,38,1', '557371806,0,0,36,1', '557371806,0,0,34,1', '557371806,0,0,34,2', '557371806,0,0,35,1', '557371806,0,0,44,2', '557371806,0,0,36,0', '557371806,0,0,35,2', '557371806,0,0,35,3'}
# case K, irregular overlapped lanes
#focused_set = {'556940257,0,0,92,2', '556940257,0,0,27,1', '556940257,0,0,91,1', '556940257,0,0,54,1', '556940257,0,0,44,1', '556940257,0,0,91,2', '556940257,0,0,54,2', '556940257,0,0,92,3', '556940257,0,0,52,0', '556940257,0,0,43,1', '556940257,0,0,92,1', '556940257,0,0,43,2', '556940257,0,0,52,1', '556940257,0,0,44,2', '556940257,0,0,27,2', '556940257,0,0,66,1', '556940257,0,0,91,3'}


# case L, overlap lanes
#focused_set = {'557005247,0,0,13,1', '557005247,0,0,49,2', '557005247,0,0,50,1', '557005247,0,0,50,2', '557005247,0,0,50,3', '557005247,0,0,13,0', '557005247,0,0,7,1', '557005247,0,0,49,1'}

#focused_set = {'557371806,0,0,37,1', '557371806,0,0,35,3', '557371806,0,0,44,2', '557371806,0,0,44,1', '557371806,0,0,38,1', '557371806,0,0,35,1', '557371806,0,0,36,1', '557371806,0,0,34,1', '557371806,0,0,35,2', '557371806,0,0,36,0', '557371806,0,0,34,2'}

#focused_set = {'557039865,0,0,10,1', '557039865,0,0,58,0'}
#focused_set = {'557039865,0,0,71,1', '557039865,0,0,31,1'}

#focused_set = {'556980651,0,0,76,1', '556980651,0,0,77,1'} # cut
#focused_set = {'556980651,0,0,81,1', '556980651,0,0,80,1'} # pt_hash

for idx, (case_name, geojson_file, georef) in enumerate(geojson_files):
  print(idx, geojson_file)
  #ret = run(geojson_file, {}, preview=False, export=True, georef=georef)
  ret = run(geojson_file, focused_set, preview=False, georef=georef)
  print(ret)
exit()

if __name__ == '__main__':
  georef = "3 | 0 | IGS& 4 | 1 | ENU, 121.25589706935, 31.1956300958991, 0"
  geojson_file = "0eca7058-c239-41f3-9f06-8a1243fa2063.json"

  # georef = "3 | 0 | IGS& 4 | 1 | ENU, 117.285684663802, 36.722913114354, 0"
  # geojson_file = "ee2dcc13-a190-48b3-b93f-fc54e2dd9c65.json"

  # georef = "3 | 0 | IGS& 4 | 1 | ENU, 121.25589706935, 31.1956300958991, 0"
  # geojson_file = "94eeaa34-796c-46d2-89bd-4099f7e70cfc.json"

  # georef = "3 | 0 | IGS& 4 | 1 | ENU, 119.01238177903, 34.8047443293035, 0"
  # geojson_file = "e2b2f2dc-2436-4870-bb8b-ad5db9db1319.json"

  #focused_set = run(geojson_file, preview=True)
  focused_set = {'557024172,0,0,52,2', '557024172,0,0,42,1', '557024172,0,0,42,3', '557024172,0,0,66,2', '557024172,0,0,52,0', '557024172,0,0,42,4', '557024172,0,0,16,1', '557024172,0,0,63,0', '557024172,0,0,16,4', '557024172,0,0,67,0', '557024172,0,0,53,2', '557024172,0,0,52,3', '557024172,0,0,67,5', '557024172,0,0,66,4', '557024172,0,0,67,4', '557024172,0,0,63,2', '557024172,0,0,42,2', '557024172,0,0,66,5', '557024172,0,0,53,1', '557024172,0,0,42,0', '557024172,0,0,16,0', '557024172,0,0,17,0', '557024172,0,0,67,1', '557024172,0,0,66,1', '557024172,0,0,17,3', '557024172,0,0,17,2', '557024172,0,0,52,1', '557024172,0,0,37,1', '557024172,0,0,63,4', '557024172,0,0,53,0', '557024172,0,0,53,3', '557024172,0,0,16,2', '557024172,0,0,67,3', '557024172,0,0,36,1', '557024172,0,0,66,3', '557024172,0,0,63,1', '557024172,0,0,16,3', '557024172,0,0,63,3', '557024172,0,0,17,1', '557024172,0,0,67,2', '557024172,0,0,37,0', '557024172,0,0,66,0', '557024172,0,0,36,2', '557024172,0,0,36,0', '557024172,0,0,63,5'}
  #focused_set = {'557024173,0,0,13,0', '557024173,0,0,17,3', '557024172,0,0,32,2', '557024173,0,0,14,1', '557024173,0,0,15,0', '557024172,0,0,32,1', '557024173,0,0,12,3', '557024173,0,0,12,1', '557024173,0,0,17,4', '557024173,0,0,17,5', '557024172,0,0,21,3', '557024172,0,0,21,0', '557024172,0,0,21,4', '557024173,0,0,12,0', '557024173,0,0,15,2', '557024173,0,0,12,4', '557024173,0,0,12,2', '557024173,0,0,15,4', '557024173,0,0,17,1', '557024173,0,0,13,1', '557024173,0,0,15,5', '557024173,0,0,11,3', '557024173,0,0,11,0', '557024172,0,0,32,0', '557024173,0,0,11,2', '557024173,0,0,17,2', '557024173,0,0,13,2', '557024173,0,0,15,1', '557024173,0,0,17,0', '557024172,0,0,21,2', '557024173,0,0,14,0', '557024172,0,0,21,1', '557024173,0,0,11,1', '557024173,0,0,15,3'}
  #focused_set = {'557024172,0,0,19,3', '557024172,0,0,19,1', '557024173,0,0,2,3', '557024173,0,0,4,1', '557024173,0,0,7,3', '557024172,0,0,46,2', '557024172,0,0,12,3', '557024172,0,0,43,2', '557024173,0,0,5,0', '557024173,0,0,2,1', '557024173,0,0,3,4', '557024172,0,0,20,3', '557024172,0,0,74,1', '557024173,0,0,6,3', '557024172,0,0,20,2', '557024173,0,0,6,2', '557024173,0,0,2,0', '557024173,0,0,3,2', '557024173,0,0,8,3', '557024172,0,0,43,0', '557024172,0,0,20,4', '557024173,0,0,9,3', '557024173,0,0,2,2', '557024173,0,0,6,0', '557024173,0,0,8,0', '557024172,0,0,12,1', '557024172,0,0,20,1', '557024172,0,0,43,1', '557024172,0,0,49,0', '557024172,0,0,20,0', '557024172,0,0,74,0', '557024172,0,0,12,0', '557024173,0,0,8,2', '557024173,0,0,8,4', '557024173,0,0,7,4', '557024173,0,0,4,3', '557024173,0,0,5,2', '557024173,0,0,2,4', '557024172,0,0,19,2', '557024173,0,0,6,1', '557024173,0,0,9,1', '557024173,0,0,8,1', '557024172,0,0,46,1', '557024173,0,0,4,0', '557024173,0,0,7,0', '557024172,0,0,49,1', '557024173,0,0,3,0', '557024173,0,0,9,0', '557024172,0,0,19,0', '557024172,0,0,12,4', '557024173,0,0,8,5', '557024173,0,0,7,2', '557024172,0,0,44,0', '557024172,0,0,44,1', '557024172,0,0,12,2', '557024173,0,0,9,2', '557024172,0,0,46,0', '557024173,0,0,4,2', '557024173,0,0,3,3', '557024172,0,0,74,3', '557024172,0,0,74,4', '557024172,0,0,49,3', '557024173,0,0,7,1', '557024173,0,0,5,1', '557024173,0,0,8,6', '557024173,0,0,3,1', '557024173,0,0,7,5', '557024172,0,0,49,2', '557024172,0,0,74,2', '557024173,0,0,6,4'}
  #focused_set = {'557024174,0,0,17,3', '557024174,0,0,15,0', '557024174,0,0,14,2', '557024174,0,0,14,1', '557024174,0,0,13,1', '557024174,0,0,17,0', '557024172,0,0,46,1', '557024174,0,0,18,0', '557024172,0,0,29,0', '557024172,0,0,29,3', '557024172,0,0,46,2', '557024174,0,0,17,2', '557024174,0,0,20,1', '557024172,0,0,29,1', '557024174,0,0,19,0', '557024174,0,0,17,1', '557024174,0,0,19,1', '557024174,0,0,13,2', '557024174,0,0,14,0', '557024174,0,0,20,3', '557024174,0,0,16,0', '557024174,0,0,18,1', '557024172,0,0,29,2', '557024174,0,0,18,2', '557024172,0,0,46,0', '557024174,0,0,19,2', '557024174,0,0,20,2', '557024174,0,0,20,0', '557024174,0,0,13,3', '557024174,0,0,16,1', '557024174,0,0,15,1', '557024174,0,0,13,0'}
  #focused_set = {'557024172,0,0,38,1', '557024172,0,0,26,1', '557024172,0,0,30,1', '557024172,0,0,38,2', '557024172,0,0,26,3', '557024172,0,0,29,2', '557024172,0,0,27,2', '557024172,0,0,28,1', '557024172,0,0,28,2', '557024172,0,0,29,1', '557024172,0,0,26,2', '557024172,0,0,30,0', '557024172,0,0,27,1'}
  #focused_set = {'557024172,0,0,71,1', '557024172,0,0,55,1', '557024172,0,0,70,2', '557024172,0,0,71,2', '557024172,0,0,70,1', '557024172,0,0,56,2', '557024172,0,0,56,1', '557024172,0,0,40,0', '557024172,0,0,55,2'}
  #focused_set = {'557024172,0,0,56,2', '557024172,0,0,49,2', '557024172,0,0,56,1', '557024172,0,0,55,1', '557024172,0,0,38,1', '557024172,0,0,71,2', '557024172,0,0,71,1', '557024172,0,0,49,1', '557024172,0,0,55,2', '557024172,0,0,38,2', '557024172,0,0,50,2', '557024172,0,0,50,1', '557024172,0,0,70,1', '557024172,0,0,70,2', '557024172,0,0,40,0'}

  #focused_set = {'557024172,0,0,71,2', '557024172,0,0,56,1', '557024172,0,0,50,1', '557024172,0,0,49,2', '557024172,0,0,38,2', '557024172,0,0,70,2', '557024172,0,0,40,0', '557024172,0,0,56,2', '557024172,0,0,55,2', '557024172,0,0,55,1', '557024172,0,0,50,2', '557024172,0,0,71,1', '557024172,0,0,38,1', '557024172,0,0,70,1', '557024172,0,0,49,1'}
  #focused_set = {'557392309,0,0,16,2', '557392309,0,0,15,1', '557392309,0,0,45,1', '557392309,0,0,16,1', '557392309,0,0,29,1', '557392309,0,0,34,2', '557392309,0,0,27,2', '557392309,0,0,38,1', '557392309,0,0,47,0', '557392309,0,0,27,1', '557392309,0,0,47,1', '557392309,0,0,17,0', '557392309,0,0,37,1', '557392309,0,0,71,1', '557392309,0,0,34,1', '557392309,0,0,27,3', '557392309,0,0,15,3', '557392309,0,0,28,1', '557392309,0,0,79,1', '557392309,0,0,41,1', '557392309,0,0,33,1', '557392308,0,0,7,1', '557392309,0,0,29,3', '557392309,0,0,67,1', '557392309,0,0,39,1', '557392309,0,0,43,1', '557392309,0,0,35,1', '557392309,0,0,55,1', '557392309,0,0,29,2', '557392309,0,0,15,2', '557392309,0,0,33,0', '557392309,0,0,68,2', '557392309,0,0,55,2', '557392309,0,0,36,1', '557392309,0,0,51,1', '557392309,0,0,68,1', '557392309,0,0,14,1', '557392309,0,0,37,0', '557392309,0,0,44,1', '557392309,0,0,14,2', '557392309,0,0,28,2'}
  #focused_set = {'557392309,0,0,47,0', '557392309,0,0,68,2', '557392309,0,0,45,1', '557392309,0,0,15,3', '557392309,0,0,14,2', '557392309,0,0,43,1', '557392309,0,0,47,1', '557392309,0,0,14,1', '557392309,0,0,15,1', '557392309,0,0,44,1', '557392309,0,0,68,1', '557392309,0,0,15,2'}
  if focused_set:
    print(focused_set)
    ret = run(geojson_file, focused_set, preview=False, georef=georef)
    print(ret)

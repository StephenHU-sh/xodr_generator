import matplotlib
matplotlib.use('TkAgg')
#matplotlib.use('WXAgg') 
#matplotlib.use('Qt5Agg') 
#matplotlib.use('QtAgg') 
#matplotlib.use('WebAgg') 

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
from geom_utils import WorldBox, first, dist2, is_almost_the_same_pt, curvature, pt_hash, xxyy2xyxy, xyxy2xxyy

import shapely.geometry as sgeom
from shapely.ops import split
from shapely.geometry import LineString, Point, MultiPoint
from collections import OrderedDict

cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
region = None
fig = None
polys = []
focused_set = set()
polys2 = []
my_map = None

class SeparatorID:
  def __init__(self, id):
    self.v = id

class Separator:
  def __init__(self, id):
    self.id = id
    self.terminals = set()
    self.heading = None
    self.base_pts = []

    # for direct junctions
    self.road_id = None
    self.road_split_from = None
    self.road_merged_to = None

class Lane:
  def __init__(self, full_id, id, poly):
    self.road_id = None
    self.full_id = full_id
    self.id = id
    self.xodr = None

    base_x = -257882.086764
    base_y = 49751.229238
    self.poly = [(x-base_x, y-base_y) for x,y in poly]
    xx, yy = xyxy2xxyy(self.poly)

    # Split the lane polygon into two boundaries
    for idx, pt in enumerate(poly):
      if pt[0] == 1234.0 and pt[1] == 4567.0:
        break
    self.left_bnd = (xx[:idx], yy[:idx])
    self.right_bnd = (list(reversed(xx[idx+1:-1])), list(reversed(yy[idx+1:-1])))
    self.update_poly()
    
    self.left_bnd_to_recut = []  # [(x1,y1),(x2,y2),...])
    self.right_bnd_to_recut = []

    self.left_neighbors = set()
    self.right_neighbors = set()
    self.predecessors = set()
    self.successors = set()
    self.overlapped = set()
    self.reducing = False
    self.growning = False

  def recut_bnd(self, base_pt, heading, start_or_end):
    d = 60.0
    heading += math.pi / 2
    separator_line = [(base_pt[0] + d * math.cos(heading), base_pt[1] + d * math.sin(heading))]
    separator_line.append((base_pt[0] - d * math.cos(heading), base_pt[1] - d * math.sin(heading)))

    left_bnd_polyline = sgeom.LineString(self.left_bnd_to_recut)
    right_bnd_polyline = sgeom.LineString(self.right_bnd_to_recut)
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
      self.left_bnd_to_recut = [(left_sep_pt.x, left_sep_pt.y)] + xxyy2xyxy(left2_pts)
    else: # "end"
      self.left_bnd_to_recut = xxyy2xyxy(left1_pts) + [(left_sep_pt.x, left_sep_pt.y)]

    result = split(right_bnd_polyline, separator_polyline)
    if len(result.geoms) < 2:
      # Failed to cut the boundary. Show debug lines.
      return [self.right_bnd_to_recut, separator_line]
    right1_pts = [result.geoms[0].xy[0], result.geoms[0].xy[1]]
    right2_pts = [result.geoms[1].xy[0], result.geoms[1].xy[1]]
    right_sep_pt = right_bnd_polyline.intersection(separator_polyline)
    assert(isinstance(right_sep_pt, Point))

    if start_or_end == "start":
      self.right_bnd_to_recut = [(right_sep_pt.x, right_sep_pt.y)] + xxyy2xyxy(right2_pts)
    else: # "end"
      self.right_bnd_to_recut = xxyy2xyxy(right1_pts) + [(right_sep_pt.x, right_sep_pt.y)]
    
    return []

  def update_poly(self):
    self.poly = xxyy2xyxy(self.left_bnd) + list(reversed(xxyy2xyxy(self.right_bnd)))


  def debug_print(self, prefix=""):
    # if self.full_id in ("557024172,0,0,36,0"):
    #   print(f"{prefix}Lane[{self.full_id}]")
    #   print(prefix, "right bnd end point:   %.4f, %.4f" % (self.right_bnd[0][-1], self.right_bnd[1][-1]))
    # if self.full_id in ("557024172,0,0,37,0"):
    #   print(f"{prefix}Lane[{self.full_id}]")
    #   print(prefix,"right bnd start point: %.4f, %.4f" % (self.right_bnd[0][0], self.right_bnd[1][0]))
    # return

    print(f"{prefix}Lane[{self.id}]", end="")
    print(f"\tleft:{[lane.id for lane in self.left_neighbors]},", end="")
    print(f"\tright:{[lane.id for lane in self.right_neighbors]},", end="")
    print(f"\tprev:{[lane.full_id for lane in self.predecessors]},", end="")
    print(f"\tnext:{[lane.full_id for lane in self.successors]},", end="")
    print(f"\toverlapped:{[lane.full_id for lane in self.overlapped]},")

# one-way street with only one lane section
class Road:
  def __init__(self, id):
    self.id = id
    self.short_id = int(id.split(",")[-1])
    self.xodr = None
    self.lanes = OrderedDict()
    self.old_ref_line = None
    self.ref_line = None  # (xx, yy)

    # assume roads are connected in:
    #   ... => (road_a_start, road_a_end) => (road_b_start, road_b_end) => ...
    self.linkage = [None, None] # [(prev_road_a, "end"), (next_road_b, "start")]

  def add_lane(self, lane):
    lane.road_id = self.id
    lane.road = self
    self.lanes[lane.id] = lane

  def sort_lanes(self):
    self.lanes = OrderedDict([(k,v) for k, v in reversed(sorted(self.lanes.items()))])

  def backup_ref_line(self):
    self.old_ref_line = self.ref_line

  def build_ref_line(self):
    for id, lane in self.lanes.items():
      self.ref_line = lane.left_bnd
      # Add more points for a straight line with only 2 points
      if len(self.ref_line[0]) == 2:
        xx = [self.ref_line[0][0]*(5-a)/5 + self.ref_line[0][1]*a/5 for a in range(6)]
        yy = [self.ref_line[1][0]*(5-a)/5 + self.ref_line[1][1]*a/5 for a in range(6)]
        self.ref_line = (xx, yy)
      break # left-most lane only

  def resample_ref_line(self, d, method="linear"):
    pts = [(self.ref_line[0][idx], self.ref_line[1][idx]) for idx in range(len(self.ref_line[0]))]
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

    self.ref_line = xyxy2xxyy(pts)

  def compute_ref_line_bc_derivative(self):
    h1 = math.atan2(self.ref_line[1][1]-self.ref_line[1][0], self.ref_line[0][1]-self.ref_line[0][0])
    h2 = math.atan2(self.ref_line[1][-1]-self.ref_line[1][-2], self.ref_line[0][-1]-self.ref_line[0][-2])
    self.heading = [h1, h2]

  def merge_overlapped_lanes(self):
    for (lane_id, lane) in self.lanes.items():
      # Only handle at most 2 lanes got merged
      assert(len(lane.overlapped) <= 1)
      if len(lane.overlapped) > 0:
        overlapped_lane = first(lane.overlapped)
        if overlapped_lane.id + 1 == lane.id:
          # Merge overlapped lane to left lane
          print(f"Merge Lane[{lane.full_id}] and Lane[{overlapped_lane.full_id}]")
          lane.right_bnd = overlapped_lane.right_bnd
          lane.right_neighbors = overlapped_lane.right_neighbors
          lane.predecessors = lane.predecessors.union(overlapped_lane.predecessors)
          assert(len(overlapped_lane.right_neighbors) <= 1)
          for lane2 in overlapped_lane.right_neighbors:
            lane2.left_neighbors.remove(overlapped_lane)
            lane2.left_neighbors.add(lane)
          for lane2 in overlapped_lane.predecessors:
            lane2.successors.remove(overlapped_lane)
            lane2.successors.add(lane)
          for lane2 in overlapped_lane.successors:
            lane2.predecessors.remove(overlapped_lane)
            lane2.predecessors.add(lane)
          assert(len(lane.predecessors) <= 2)
          if len(lane.predecessors) > 1:
            for lane2 in lane.predecessors:
              for lane3 in lane.predecessors:
                if lane2 != lane3:
                  lane2.overlapped.add(lane3)
          lane.successors = lane.successors.union(overlapped_lane.successors)
          assert(len(lane.successors) <= 2)
          if len(lane.successors) > 1:
            for lane2 in lane.successors:
              for lane3 in lane.successors:
                if lane2 != lane3:
                  lane2.overlapped.add(lane3)
          lane.overlapped = set()
          lane.reducing = False
          lane.growning = False
          del self.lanes[overlapped_lane.id]
          return True
    return False

  def __repr__(self):
    return f"Road[{self.id}]"

  def debug_print(self):
    from_road = ""
    to_road = "" 
    if self.linkage[0] is not None:
      if isinstance(self.linkage[0][0], int):
        from_road = f", from Junction[{self.linkage[0][0]}]"
      else:
        from_road = f", from Road[{self.linkage[0][0].id}] {self.linkage[0][1]}"
    if self.linkage[1] is not None:
      if isinstance(self.linkage[1][0], int):
        to_road = f", to Junction[{self.linkage[1][0]}]"
      else:
        to_road = f", to Road[{self.linkage[1][0].id}] {self.linkage[1][1]}"
    print(f"Road[{self.id}]{from_road}{to_road}")

    for lane_id, lane in self.lanes.items():
      lane.debug_print("\t")


def update_neighbor(lane1, lane2):
  lane1_left_p0 = (lane1.left_bnd[0][0], lane1.left_bnd[1][0])
  lane1_left_p1 = (lane1.left_bnd[0][-1], lane1.left_bnd[1][-1])
  lane1_right_p0 = (lane1.right_bnd[0][0], lane1.right_bnd[1][0])
  lane1_right_p1 = (lane1.right_bnd[0][-1], lane1.right_bnd[1][-1])
  lane2_left_p0 = (lane2.left_bnd[0][0], lane2.left_bnd[1][0])
  lane2_left_p1 = (lane2.left_bnd[0][-1], lane2.left_bnd[1][-1])
  lane2_right_p0 = (lane2.right_bnd[0][0], lane2.right_bnd[1][0])
  lane2_right_p1 = (lane2.right_bnd[0][-1], lane2.right_bnd[1][-1])

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
  def __init__(self):
    self.roads = OrderedDict()
    self.debug_pts = []

  def add_road(self, road_id):
    self.roads[road_id] = Road(road_id)

  def pt_lane_set(self, x, y):
    pt_str = pt_hash(x, y)
    assert pt_str in self.pt2lane
    return self.pt2lane[pt_str]

  def update_pt2lane_hash_table(self):
    self.pt2lane = {}

    # Build hash table for boundary points
    for road_id, road in self.roads.items():
      for lane_id, lane in road.lanes.items():
        for pt in lane.poly:
          pt_str = pt_hash(pt[0], pt[1])
          if pt_str not in self.pt2lane:
            self.pt2lane[pt_str] = set()
          self.pt2lane[pt_str].add(lane)


  def compute_overlapped_lanes(self, lane):
    # Spliting
    to_del = []
    for lane2 in lane.predecessors:
      if lane.road_id == lane2.road_id:
        lane.overlapped.add(lane2)
        lane2.overlapped.add(lane)
        to_del.append(lane2)
    for lane3 in to_del:
      lane.predecessors.remove(lane3)

    # Merging
    to_del = []
    for lane2 in lane.successors:
      if lane.road_id == lane2.road_id:
        lane.overlapped.add(lane2)
        lane2.overlapped.add(lane)
        to_del.append(lane2)
    for lane3 in to_del:
      lane.successors.remove(lane3)

  def compute_lane_topo(self):
    for road_id, road in self.roads.items():
      #if road_id != '557024172,0,0,53':
      #  continue
      lanes = list(road.lanes.values())
      for i in range(len(lanes)):
        for j in range(i+1, len(lanes)):
          update_neighbor(lanes[i], lanes[j])
        a_hash = pt_hash(lanes[i].left_bnd[0][0], lanes[i].left_bnd[1][0])
        b_hash = pt_hash(lanes[i].right_bnd[0][0], lanes[i].right_bnd[1][0])
        if a_hash != b_hash:
          a = self.pt_lane_set(lanes[i].left_bnd[0][0], lanes[i].left_bnd[1][0])
          b = self.pt_lane_set(lanes[i].right_bnd[0][0], lanes[i].right_bnd[1][0])
          lanes[i].predecessors = a.intersection(b).difference(set([lanes[i]]))
        else:
          self.growning = True

        a_hash = pt_hash(lanes[i].left_bnd[0][-1], lanes[i].left_bnd[1][-1])
        b_hash = pt_hash(lanes[i].right_bnd[0][-1], lanes[i].right_bnd[1][-1])
        if a_hash != b_hash:
          a = self.pt_lane_set(lanes[i].left_bnd[0][-1], lanes[i].left_bnd[1][-1])
          b = self.pt_lane_set(lanes[i].right_bnd[0][-1], lanes[i].right_bnd[1][-1])
          lanes[i].successors = a.intersection(b).difference(set([(lanes[i])]))
        else:
          self.reducing = True

        self.compute_overlapped_lanes(lanes[i])

  def merge_overlapped_lanes(self):
    while True:
      lanes_got_merged = False
      for road_id, road in self.roads.items():
        lanes_got_merged = lanes_got_merged or road.merge_overlapped_lanes()
      if not lanes_got_merged:
        break

  def update_separator_ids(self, separator_ids, start_sep, end_sep):
    if end_sep in separator_ids and start_sep not in separator_ids:
      separator_ids[start_sep] = separator_ids[end_sep]
    elif end_sep not in separator_ids and start_sep in separator_ids:
      separator_ids[end_sep] = separator_ids[start_sep]
    elif end_sep not in separator_ids and start_sep not in separator_ids:
      new_id = SeparatorID(len(separator_ids))
      separator_ids[start_sep] = new_id
      separator_ids[end_sep] = new_id
    elif end_sep in separator_ids and start_sep in separator_ids:
      if separator_ids[start_sep].v != separator_ids[end_sep]:
        separator_ids[start_sep].v = separator_ids[end_sep].v

  def find_common_lane_bnd_pt(self, seps, terminal_type):
    pts_hashmap = {}
    base_pts = []
    for road, start_or_end in seps:
      if start_or_end == terminal_type:
        pt_idx = 0 if terminal_type == "start" else -1
        pt_idx2 = -1 if terminal_type == "start" else 0
        for lane_id, lane in road.lanes.items():
          pt_left1 = (lane.left_bnd[0][pt_idx], lane.left_bnd[1][pt_idx])
          pt_left1_hash = pt_hash(pt_left1[0], pt_left1[1])
          pt_left2_hash = pt_hash(lane.left_bnd[0][pt_idx2], lane.left_bnd[1][pt_idx2])
          if pt_left1_hash not in pts_hashmap:
            pts_hashmap[pt_left1_hash] = set()
          elif pt_left2_hash not in pts_hashmap[pt_left1_hash]:
            base_pts.append(pt_left1)
          pts_hashmap[pt_left1_hash].add(pt_left2_hash)
          pt_right1 = (lane.right_bnd[0][pt_idx], lane.right_bnd[1][pt_idx])
          pt_right1_hash = pt_hash(pt_right1[0], pt_right1[1])
          pt_right2_hash = pt_hash(lane.right_bnd[0][pt_idx2], lane.right_bnd[1][pt_idx2])
          if pt_right1_hash not in pts_hashmap:
            pts_hashmap[pt_right1_hash] = set()
          elif pt_right2_hash not in pts_hashmap[pt_right1_hash]:  # not shared on the other terminal
            base_pts.append(pt_right1)
          pts_hashmap[pt_right1_hash].add(pt_right2_hash)
    return base_pts

  def extend_bnd_by_extrapolation(self, bnd, start_or_end):
    d = 10.0  # in meters
    if start_or_end == "start":
      heading = math.atan2(bnd[0][1]-bnd[1][1], bnd[0][0]-bnd[1][0])
      bnd.appendleft((bnd[0][0]+math.cos(heading)*d, bnd[0][1]+math.sin(heading)*d))
    else:
      heading = math.atan2(bnd[-1][1]-bnd[-2][1], bnd[-1][0]-bnd[-2][0])
      bnd.append((bnd[-1][0]+math.cos(heading)*d, bnd[-1][1]+math.sin(heading)*d))

  def find_lane_terminals_shared_same_directions(self):
    separator_ids = {}
    separator_set = {}
    for road_id, road in self.roads.items():
      for lane_id, lane in road.lanes.items():
        for lane2 in lane.predecessors:
          end_sep = (lane2.road, "end")
          start_sep = (road, "start")
          self.update_separator_ids(separator_ids, start_sep, end_sep)
        for lane2 in lane.successors:
          start_sep = (lane2.road, "start")
          end_sep = (road, "end")
          self.update_separator_ids(separator_ids, start_sep, end_sep)
    # for k, v in self.separator_ids.items():
    #   print(f"{k}: {v.v}")
    # print("-------------")
    max_sep_id = -1
    for sep, sep_id in separator_ids.items():
      max_sep_id = max(max_sep_id, sep_id.v)
      if sep_id not in separator_set:
        separator_set[sep_id] = Separator(sep_id)
      separator_set[sep_id].terminals.add(sep)

    # Add dead-end/start to separation set
    for road_id, road in self.roads.items():
      found_predecessors = False
      for lane_id, lane in road.lanes.items():
        if len(lane.predecessors) > 0:
          found_predecessors = True
          break
      if not found_predecessors:
        max_sep_id += 1
        new_sep_id = SeparatorID(max_sep_id)
        new_sep = Separator(new_sep_id)
        new_sep.terminals.add((road, "start"))
        separator_set[new_sep_id] = new_sep
    for road_id, road in self.roads.items():
      found_successors = False
      for lane_id, lane in road.lanes.items():
        if len(lane.successors) > 0:
          found_successors = True
          break
      if not found_successors:
        max_sep_id += 1
        new_sep_id = SeparatorID(max_sep_id)
        new_sep = Separator(new_sep_id)
        new_sep.terminals.add((road, "end"))
        separator_set[new_sep_id] = new_sep

    for id, sep in separator_set.items():
       print(f"{id.v}: {sep.terminals}")
    return separator_set

  def select_road_direction_at_terminals(self, sep_set):
    for sep_set_id, sep in sep_set.items():
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
        sep.road_id = road_with_end_point.short_id
        heading = road_with_end_point.heading[1]
      elif start_count == 1 and end_count > 1:
        sep.road_merged_to = road_with_start_point
        sep.road_id = road_with_start_point.short_id
        heading = road_with_start_point.heading[0]
      else: # Or select direction of the most straight road
        min_curvature = 999999
        for road, start_or_end in sep.terminals:
          r = road.ref_line
          if start_or_end == "start":
            c = curvature(r[0][0:3], r[1][0:3])
          else:
            c = curvature(r[0][-3:], r[1][-3:])
          if c < min_curvature:
            c = min_curvature
            heading = road.heading[0 if start_or_end == "start" else 1]
      for road, start_or_end in sep.terminals:
        idx = 0 if start_or_end == "start" else 1
        # if road.id == "557024172,0,0,36" and start_or_end == "end":
        #   print("old road end heading: ", math.degrees(road.heading[idx]))
        #   print("new road end heading: ", math.degrees(heading))
        road.heading[idx] = heading
      sep.heading = heading
    return sep_set

  def determine_separation_line_base_point(self, sep_set):
    for sep_set_id, sep in sep_set.items():
      base_pts = []
      # # Find points at lane tips with zero lane width
      # for idx, (road, start_or_end) in enumerate(seps):
      #   pt_idx = 0 if start_or_end == "start" else -1
      #   for lane_id, lane in road.lanes.items():
      #     pt1_hash = pt_hash(lane.left_bnd[0][pt_idx], lane.left_bnd[1][pt_idx])
      #     pt2_hash = pt_hash(lane.right_bnd[0][pt_idx], lane.right_bnd[1][pt_idx])
      #     if pt1_hash == pt2_hash:
      #       base_pts.append((lane.left_bnd[0][pt_idx], lane.left_bnd[1][pt_idx]))

      if len(sep.terminals) == 1:
        # Just use terminal point of reference line for roads with no predecessors/successors
        road, start_or_end = first(sep.terminals)
        pt_idx = 0 if start_or_end == "start" else -1
        base_pts += [(road.ref_line[0][pt_idx], road.ref_line[1][pt_idx])]
      else:
        # Find shared start/end points among roads
        base_pts += self.find_common_lane_bnd_pt(sep.terminals, "start")
        base_pts += self.find_common_lane_bnd_pt(sep.terminals, "end")

      assert(len(base_pts) > 0 or len(sep.terminals) == 2)
      if len(base_pts) == 0:
        for idx, (road, start_or_end) in enumerate(sep.terminals):
          pt_idx = 0 if start_or_end == "start" else -1
          if idx == 0:
            base_pt1 = (road.ref_line[0][pt_idx], road.ref_line[1][pt_idx])
          else:
            base_pt2 = (road.ref_line[0][pt_idx], road.ref_line[1][pt_idx])
        base_pts.append(((base_pt1[0]+base_pt2[0])/2, (base_pt1[1]+base_pt2[1])/2))

      if len(base_pts) != 1:
        # For overlapped merging lanes,
        # we usually find 2 or more shared start/end points among roads.
        # Draw the separation line along the first 2 points.
        heading = math.atan2(base_pts[0][1]-base_pts[1][1], base_pts[0][0]-base_pts[1][0])
        heading += math.pi/2
        for road, start_or_end in sep.terminals:
          if start_or_end == "start":
            heading2 = math.atan2(road.ref_line[1][1]-road.ref_line[1][0], road.ref_line[0][1]-road.ref_line[0][0])
            if math.cos(heading)*math.cos(heading2)+math.sin(heading)*math.sin(heading) < 0.5:
             heading += math.pi
            for road2, start_or_end in sep.terminals:
              if start_or_end == "start":
                road2.heading[0] = heading
              else:
                road2.heading[1] = heading
            sep.heading = heading
            sep.base_pts = [base_pts[0]]
            break
      else:
        sep.base_pts = base_pts
      print(f"{sep_set_id.v}: {sep.terminals}")
      print("\t", base_pts)
      print("==========================================")

  def prepare_for_bnd_recut(self, sep_set):
    roads_to_recut_bnd = {}
    for sep_set_id, sep in sep_set.items():    
      if len(sep.base_pts) != 1:
        continue
      for road, start_or_end in sep.terminals:
        roads_to_recut_bnd[road] = (True, True)
    # for road, (cut_start, cut_end) in roads_to_recut_bnd.items():
    #   for lane_id, lane in road.lanes.items():
    #     if len(lane.predecessors) > 1:
    #       cut_start = False
    #       break
    #   for lane_id, lane in road.lanes.items():
    #     if len(lane.successors) > 1:
    #       cut_end = False
    #       break
    #   roads_to_recut_bnd[road] = (cut_start, cut_end)

    # Extend lane boundaries
    for road, (cut_start, cut_end) in roads_to_recut_bnd.items():
      for lane_id, lane in road.lanes.items():
        left_bnd = deque(xxyy2xyxy(lane.left_bnd))
        right_bnd = deque(xxyy2xyxy(lane.right_bnd))
        if cut_start:
          if len(lane.predecessors) != 0:
             # With boundaries of predecessor lane
            prev_lane = first(lane.predecessors)
            left_bnd.extendleft(reversed(xxyy2xyxy(prev_lane.left_bnd)))
            right_bnd.extendleft(reversed(xxyy2xyxy(prev_lane.right_bnd)))
          else:
            if len(lane.left_neighbors) and len(first(lane.left_neighbors).predecessors) > 0:
              # With the right boundary of left neighbor's predecessor lane
              neighbor_prev_lane = first(first(lane.left_neighbors).predecessors)
              left_bnd.extendleft(reversed(xxyy2xyxy(neighbor_prev_lane.right_bnd)))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(left_bnd, "start")
            if len(lane.right_neighbors) and len(first(lane.right_neighbors).predecessors) > 0:
              # with the left boundary of right neighbor's predecessor lane
              neighbor_prev_lane = first(first(lane.right_neighbors).predecessors)
              right_bnd.extendleft(reversed(xxyy2xyxy(neighbor_prev_lane.left_bnd)))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(right_bnd, "start")
        if cut_end:
          if len(lane.successors) != 0:
             # With boundaries of predecessor lane
            next_lane = first(lane.successors)
            left_bnd.extend(xxyy2xyxy(next_lane.left_bnd))
            right_bnd.extend(xxyy2xyxy(next_lane.right_bnd))
          else:
            if len(lane.left_neighbors) and len(first(lane.left_neighbors).successors) > 0:
              # With right boundary of left neighbor's successor lane
              neighbor_next_lane = first(first(lane.left_neighbors).successors)
              left_bnd.extend(xxyy2xyxy(neighbor_next_lane.right_bnd))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(left_bnd, "end")
            if len(lane.right_neighbors) and len(first(lane.right_neighbors).successors) > 0:
              # With left boundary of right neighbor's successor lane
              neighbor_next_lane = first(first(lane.right_neighbors).successors)
              right_bnd.extend(xxyy2xyxy(neighbor_next_lane.left_bnd))
            else:
              # By extrapolation
              self.extend_bnd_by_extrapolation(right_bnd, "end")
        lane.left_bnd_to_recut = left_bnd
        lane.right_bnd_to_recut = right_bnd

  def recut_bnd(self, sep_set):
    # Cut boundaries with the separation line
    updated_lanes = set()
    for sep_set_id, sep in sep_set.items():
      if len(sep.base_pts) == 1:
        for road, start_or_end in sep.terminals:
          for lane_id, lane in road.lanes.items():
            if len(self.debug_pts) != 0:
              continue  # skip remaining lanes on any errors
            self.debug_pts = lane.recut_bnd(sep.base_pts[0], sep.heading, start_or_end)
            updated_lanes.add(lane)

    # Move results from temp var to the final lane.left_bnd/right_bnd
    for lane in updated_lanes:
      lane.left_bnd = xyxy2xxyy(lane.left_bnd_to_recut)
      lane.right_bnd = xyxy2xxyy(lane.right_bnd_to_recut)
      lane.left_bnd_to_recut = []
      lane.right_bnd_to_recut = []

  def refine_lane_terminals(self):
    # Tweak terminals of lane boundaries
    sep_set = self.find_lane_terminals_shared_same_directions()
    # sep_set: {
    #   sep_id_a:(
    #         heading,
    #         base_pts,
    #         {
    #           (road_a, "end"),
    #           (road_b, "start"), ...
    #         }
    #   )
    #   sep_id_b:(...),
    #   ...
    # }
    self.select_road_direction_at_terminals(sep_set)
    self.determine_separation_line_base_point(sep_set)
    self.prepare_for_bnd_recut(sep_set)
    self.recut_bnd(sep_set)
    return sep_set

  def build_road_connections(self, sep_set):
    for sep_set_id, sep in sep_set.items():
      if len(sep.terminals) == 2:
        (road_a, start_or_end_a), (road_b, start_or_end_b) = list(sep.terminals)
        if start_or_end_a != start_or_end_b:
          pt_a_idx = 0 if start_or_end_a == "start" else -1
          pt_b_idx = 0 if start_or_end_b == "start" else -1
          a_hash = pt_hash(road_a.ref_line[0][pt_a_idx], road_a.ref_line[1][pt_a_idx])
          b_hash = pt_hash(road_b.ref_line[0][pt_b_idx], road_b.ref_line[1][pt_b_idx])
          #if a_hash == b_hash:
          road_a.linkage[pt_a_idx] = (road_b, start_or_end_b)
          road_b.linkage[pt_b_idx] = (road_a, start_or_end_a)
          print(f"Link Road [{road_a.id}] and Road [{road_b.id}]")
      elif len(sep.terminals) > 2:
        # For direct junctions
        terminals = list(sep.terminals)
        for idx_i in range(len(terminals)):
          road_a, start_or_end_a = terminals[idx_i]
          for idx_j in range(idx_i+1, len(terminals)):
            road_b, start_or_end_b = terminals[idx_j]
            if start_or_end_a == start_or_end_b:
              continue
            pt_a_idx = 0 if start_or_end_a == "start" else -1
            pt_b_idx = 0 if start_or_end_b == "start" else -1
            junction_id = sep.road_id # use road id as direct junction id
            road_a.linkage[pt_a_idx] = (junction_id, "junction")
            road_b.linkage[pt_b_idx] = (junction_id, "junction")

  def save_direct_junction_info(self, sep_set):
    self.direct_junction_info = []
    for sep_set_id, sep in sep_set.items():
      if sep.road_split_from is None and sep.road_merged_to is None:
        continue
      self.direct_junction_info.append(sep)

  def build_lane_info(self, preview):
    for road_id, road in self.roads.items():
      road.sort_lanes()
    self.update_pt2lane_hash_table()
    self.compute_lane_topo()

    #return
    if preview:
      return

    self.merge_overlapped_lanes()

    for road_id, road in self.roads.items():
      road.build_ref_line()
      road.resample_ref_line(5.0)
      road.compute_ref_line_bc_derivative()
    
    sep_set = self.refine_lane_terminals()
    self.build_road_connections(sep_set)
    self.save_direct_junction_info(sep_set)

    # Update lane poly after boundaries updated
    for road_id, road in self.roads.items():
      for lane_id, lane in road.lanes.items():
        lane.update_poly()

    # Rebuild reference line with refined lane boundaries
    for road_id, road in self.roads.items():
      road.backup_ref_line()
      road.build_ref_line()
      road.resample_ref_line(3.0)
      road.resample_ref_line(0.01, "cubic")

  def debug_print(self):
    for road_id, road in self.roads.items():
      # if road_id not in ("557024172,0,0,36"):
      #   continue
      road.debug_print()


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
      focused_set.add(selected_poly.lane_id)
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
    xodr_exporter.export(my_map)


def on_pick(event):
  if event.mouseevent.key != 'control' or event.mouseevent.button != 1:
    return
  polygon_item.PolygonInteractor.picked = event.artist
  print(f"Lane[{event.artist.lane_id}] got picked.")
  fig.canvas.draw()
  fig.canvas.flush_events()


def run(focused_set2=set()):
  global fig, polys, focused_set, my_map
  focused_set = focused_set2
  fig, ax = plt.subplots()
  pan_zoom = fig_manipulator.PanAndZoom(fig, scale_factor=1.6)
  g = geojson.load(open("out.json"))

  my_map = RoadNetwork()

  for f in g["features"]:
    if f["properties"]["layer"] != "lane":
      continue
    lane_id = f["properties"]["id"]

    if focused_set and lane_id not in focused_set:
      continue

    pos = lane_id.rfind(",")
    road_id = lane_id[:pos]
    lane_subid = int(lane_id[pos+1:])

    if road_id not in my_map.roads:
      my_map.add_road(road_id)
    my_map.roads[road_id].add_lane(Lane(lane_id, lane_subid, [(x,y) for x,y,z in f["geometry"]["coordinates"]]))

  my_map.build_lane_info(preview=(len(focused_set) == 0))
  my_map.debug_print()

  # Draw lane polygon, and boundary on the selected lane
  color_idx = 0
  for road_id, road in my_map.roads.items():
    for lane_subid, lane in road.lanes.items():
      xxyy = xyxy2xxyy(lane.poly)

      poly = Polygon(np.column_stack(xxyy), animated=True, color = (0,0,0,0))
      ax.add_patch(poly)
      poly.lane_id = lane.full_id # (road_id, lane_subid)
      polys.append(poly)

      if 0:
        # Draw extend boundaries.
        left_xx = [x for x,y in lane.left_bnd_to_recut]
        left_yy = [y for x,y in lane.left_bnd_to_recut]
        right_xx = [x for x,y in lane.right_bnd_to_recut]
        right_yy = [y for x,y in lane.right_bnd_to_recut]
        p = polygon_item.PolygonInteractor(ax, poly, [left_xx, left_yy], [right_xx, right_yy])
      else:
        p = polygon_item.PolygonInteractor(ax, poly, lane.left_bnd, lane.right_bnd)
      
      c = matplotlib.colors.to_rgb(cycle[color_idx])
      color_idx = (color_idx + 1) % len(cycle)
      p.my_color = (c[0], c[1], c[2], 0.3)
      p.my_color2 = (c[0], c[1], c[2], 0.6)

      polys2.append(sgeom.Polygon(lane.poly))
      WorldBox.update(*xxyy)

  # Draw reference lines
  for road_id, road in my_map.roads.items():
    # if road_id not in ("557024172,0,0,36", "557024172,0,0,53"):
    #   continue
    if road.ref_line:
      plt.plot(road.ref_line[0], road.ref_line[1], "-*")
    #plt.plot(road.old_ref_line[0], road.old_ref_line[1], "-x")

  # Draw debug points/lines
  for polyline in my_map.debug_pts:
    xyxy = xyxy2xxyy(polyline)
    plt.plot(xyxy[0], xyxy[1], "-o", markersize=10)

  # Draw center lines
  for f in g["features"]:
    if f["properties"]["layer"] == "center_line":
      #print(f["properties"]["id"])
      xx = [x for x,y,z in f["geometry"]["coordinates"]]
      yy = [y for x,y,z in f["geometry"]["coordinates"]]
      pts = [[x,y] for x,y,z in f["geometry"]["coordinates"]]
      pts2 = [sgeom.Point(pts[len(pts)//2])]
      inside = False
      for poly in polys2:
        k = [poly.contains(pt) for pt in pts2]
        if np.all(k):
          inside = True
          continue
      if not inside:
        continue
      plt.plot(xx, yy, "--")

  # Keep aspect ratio to 1:1 in meters
  if os.name == 'nt':
    WorldBox.update_fig_range(ax, 1000, 500)
  else:
    WorldBox.update_fig_range(ax, 1600, 800)

  # handle lane picking, selection and keyborad events
  fig.canvas.mpl_connect('pick_event', on_pick)
  if not focused_set:
    # drawtype is 'box' or 'line' or 'none'
    toggle_selector.RS = RectangleSelector(ax, region_select_callback,
      drawtype='box', useblit=True, button=[1],  # don't use middle button
      minspanx=5, minspany=5, spancoords='pixels', interactive=False)
    def update_selected_region(event):
      if toggle_selector.RS.active:
        toggle_selector.RS.update()
    plt.connect('draw_event', update_selected_region)
  plt.connect('key_press_event', toggle_selector)

  plt.show()
  return focused_set


if __name__ == '__main__':
  #focused_set = run()
  #focused_set = {'557024172,0,0,52,2', '557024172,0,0,42,1', '557024172,0,0,42,3', '557024172,0,0,66,2', '557024172,0,0,52,0', '557024172,0,0,42,4', '557024172,0,0,16,1', '557024172,0,0,63,0', '557024172,0,0,16,4', '557024172,0,0,67,0', '557024172,0,0,53,2', '557024172,0,0,52,3', '557024172,0,0,67,5', '557024172,0,0,66,4', '557024172,0,0,67,4', '557024172,0,0,63,2', '557024172,0,0,42,2', '557024172,0,0,66,5', '557024172,0,0,53,1', '557024172,0,0,42,0', '557024172,0,0,16,0', '557024172,0,0,17,0', '557024172,0,0,67,1', '557024172,0,0,66,1', '557024172,0,0,17,3', '557024172,0,0,17,2', '557024172,0,0,52,1', '557024172,0,0,37,1', '557024172,0,0,63,4', '557024172,0,0,53,0', '557024172,0,0,53,3', '557024172,0,0,16,2', '557024172,0,0,67,3', '557024172,0,0,36,1', '557024172,0,0,66,3', '557024172,0,0,63,1', '557024172,0,0,16,3', '557024172,0,0,63,3', '557024172,0,0,17,1', '557024172,0,0,67,2', '557024172,0,0,37,0', '557024172,0,0,66,0', '557024172,0,0,36,2', '557024172,0,0,36,0', '557024172,0,0,63,5'}
  #focused_set = {'557024173,0,0,13,0', '557024173,0,0,17,3', '557024172,0,0,32,2', '557024173,0,0,14,1', '557024173,0,0,15,0', '557024172,0,0,32,1', '557024173,0,0,12,3', '557024173,0,0,12,1', '557024173,0,0,17,4', '557024173,0,0,17,5', '557024172,0,0,21,3', '557024172,0,0,21,0', '557024172,0,0,21,4', '557024173,0,0,12,0', '557024173,0,0,15,2', '557024173,0,0,12,4', '557024173,0,0,12,2', '557024173,0,0,15,4', '557024173,0,0,17,1', '557024173,0,0,13,1', '557024173,0,0,15,5', '557024173,0,0,11,3', '557024173,0,0,11,0', '557024172,0,0,32,0', '557024173,0,0,11,2', '557024173,0,0,17,2', '557024173,0,0,13,2', '557024173,0,0,15,1', '557024173,0,0,17,0', '557024172,0,0,21,2', '557024173,0,0,14,0', '557024172,0,0,21,1', '557024173,0,0,11,1', '557024173,0,0,15,3'}
  focused_set = {'557024172,0,0,19,3', '557024172,0,0,19,1', '557024173,0,0,2,3', '557024173,0,0,4,1', '557024173,0,0,7,3', '557024172,0,0,46,2', '557024172,0,0,12,3', '557024172,0,0,43,2', '557024173,0,0,5,0', '557024173,0,0,2,1', '557024173,0,0,3,4', '557024172,0,0,20,3', '557024172,0,0,74,1', '557024173,0,0,6,3', '557024172,0,0,20,2', '557024173,0,0,6,2', '557024173,0,0,2,0', '557024173,0,0,3,2', '557024173,0,0,8,3', '557024172,0,0,43,0', '557024172,0,0,20,4', '557024173,0,0,9,3', '557024173,0,0,2,2', '557024173,0,0,6,0', '557024173,0,0,8,0', '557024172,0,0,12,1', '557024172,0,0,20,1', '557024172,0,0,43,1', '557024172,0,0,49,0', '557024172,0,0,20,0', '557024172,0,0,74,0', '557024172,0,0,12,0', '557024173,0,0,8,2', '557024173,0,0,8,4', '557024173,0,0,7,4', '557024173,0,0,4,3', '557024173,0,0,5,2', '557024173,0,0,2,4', '557024172,0,0,19,2', '557024173,0,0,6,1', '557024173,0,0,9,1', '557024173,0,0,8,1', '557024172,0,0,46,1', '557024173,0,0,4,0', '557024173,0,0,7,0', '557024172,0,0,49,1', '557024173,0,0,3,0', '557024173,0,0,9,0', '557024172,0,0,19,0', '557024172,0,0,12,4', '557024173,0,0,8,5', '557024173,0,0,7,2', '557024172,0,0,44,0', '557024172,0,0,44,1', '557024172,0,0,12,2', '557024173,0,0,9,2', '557024172,0,0,46,0', '557024173,0,0,4,2', '557024173,0,0,3,3', '557024172,0,0,74,3', '557024172,0,0,74,4', '557024172,0,0,49,3', '557024173,0,0,7,1', '557024173,0,0,5,1', '557024173,0,0,8,6', '557024173,0,0,3,1', '557024173,0,0,7,5', '557024172,0,0,49,2', '557024172,0,0,74,2', '557024173,0,0,6,4'}
  if focused_set:
    print(focused_set)
    run(focused_set)

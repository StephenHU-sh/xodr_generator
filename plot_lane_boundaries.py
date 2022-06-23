import matplotlib
matplotlib.use('TkAgg')
#matplotlib.use('WXAgg') 
#matplotlib.use('Qt5Agg') 
#matplotlib.use('QtAgg') 
#matplotlib.use('WebAgg') 

import math
import numpy as np
import geojson
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import RectangleSelector
import polygon_item
import fig_manipulator
import xodr_exporter

import shapely.geometry as sgeom
from collections import OrderedDict

cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
region = None
fig = None
polys = []
focused_set = set()
polys2 = []
my_map = None


def is_almost_the_same_pt(p1, p2):
  return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) < 0.05 * 0.05

def pt_hash(x, y):
  return "%.2f,%.2f" % (x, y)

class Lane:
  def __init__(self, full_id, id, poly):
    self.road_id = None
    self.full_id = full_id
    self.id = id

    base_x = -257882.086764
    base_y = 49751.229238
    for pt_idx, (x,y,z) in enumerate(poly):
      poly[pt_idx] = (x-base_x, y-base_y, z)
    self.poly = poly

    # split the lane polygon into two boundaries
    self.left_bnd = []
    self.right_bnd = []

    self.left_neighbors = set()
    self.right_neighbors = set()
    self.predecessors = set()
    self.successors = set()
    self.overlapped = set()
    self.reducing = False
    self.growning = False

  def poly_pts(self):
    for x,y,z in self.poly:
      yield x,y

  def debug_print(self, prefix=""):
    print(f"{prefix}Lane[{self.id}]", end="")
    print(f"\tleft:{[lane.id for lane in self.left_neighbors]},", end="")
    print(f"\tright:{[lane.id for lane in self.right_neighbors]},", end="")
    print(f"\tprev:{[lane.full_id for lane in self.predecessors]},", end="")
    print(f"\tnext:{[lane.full_id for lane in self.successors]},", end="")
    print(f"\toverlapped:{[lane.full_id for lane in self.overlapped]},")


class Road:
  def __init__(self, id):
    self.id = id
    self.lanes = OrderedDict()

  def add_lane(self, lane):
    lane.road_id = self.id
    self.lanes[lane.id] = lane

  def sort_lanes(self):
    self.lanes = OrderedDict([(k,v) for k, v in reversed(sorted(self.lanes.items()))])

  def build_ref_line(self):
    for id, lane in self.lanes.items():
      self.ref_line = lane.left_bnd
      # add more points for a straight line with only 2 points
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
      # line1 = []
      # line2 = []
      # dd = math.pi / 2
      # for d in range(10):
      #   line1.append((pts[0][0] + d * math.cos(self.heading[0] - dd), pts[0][1] + d * math.sin(self.heading[0] - dd)))
      # for d in range(10):
      #   line2.append((pts[-1][0] + d * math.cos(self.heading[1] - dd), pts[-1][1] + d * math.sin(self.heading[1] - dd)))
      # pts = list(reversed(line1)) + pts + line2

    xx = [x for x, y in pts]
    yy = [y for x, y in pts]
    self.ref_line = (xx, yy)

  def compute_ref_line_bc_derivative(self):
    h1 = math.atan2(self.ref_line[1][1]-self.ref_line[1][0], self.ref_line[0][1]-self.ref_line[0][0])
    h2 = math.atan2(self.ref_line[1][-1]-self.ref_line[1][-2], self.ref_line[0][-1]-self.ref_line[0][-2])
    self.heading = (h1, h2)

  def debug_print(self):
    print(f"Road[{self.id}]")
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

  def add_road(self, road_id):
    self.roads[road_id] = Road(road_id)

  def pt_lane_set(self, x, y):
    pt_str = pt_hash(x, y)
    assert pt_str in self.pt2lane
    return self.pt2lane[pt_str]

  def split_lane_poly(self):
    self.pt2lane = {}

    # build pt hash
    for road_id, road in self.roads.items():
      for lane_id, lane in road.lanes.items():
        for pt in lane.poly_pts():
          pt_str = pt_hash(pt[0], pt[1])
          if pt_str not in self.pt2lane:
            self.pt2lane[pt_str] = set()
          self.pt2lane[pt_str].add(lane)

    for road_id, road in self.roads.items():
      for lane_id, lane in road.lanes.items():
        xx = [x for x,y,z in lane.poly]
        yy = [y for x,y,z in lane.poly]

        # split poly ring on the point shared by other road
        poly_pts = list(lane.poly_pts())
        split_pt_indices = []
        for idx, pt in enumerate(poly_pts):
          if idx == 0 or idx + 2 >= len(poly_pts):
            continue
          for lane2 in self.pt_lane_set(pt[0], pt[1]):
            if lane2.road_id != road_id:
              split_pt_indices.append(idx)
              break

        # split poly ring by reversed direction
        if len(split_pt_indices) != 2:
          for idx in range(1, len(xx)-1):
            dot = (xx[idx-1]-xx[idx])*(xx[idx+1]-xx[idx]) + (yy[idx-1]-yy[idx])*(yy[idx+1]-yy[idx])
            d2a = (xx[idx-1]-xx[idx])*(xx[idx-1]-xx[idx]) + (yy[idx-1]-yy[idx])*(yy[idx-1]-yy[idx])
            d2b = (xx[idx+1]-xx[idx])*(xx[idx+1]-xx[idx]) + (yy[idx+1]-yy[idx])*(yy[idx+1]-yy[idx])
            dot /= math.sqrt(d2a*d2b+0.00000000001)
            if abs(dot) < 0.2:
              split_pt_indices = [idx, idx+1]
              break

        assert(len(split_pt_indices) == 2)
        idx = split_pt_indices[0]
        lane.left_bnd = (xx[:idx+1], yy[:idx+1])
        lane.right_bnd = (list(reversed(xx[idx+1:-1])), list(reversed(yy[idx+1:-1])))

  def compute_overlapped_lanes(self, lane):
    # spliting
    to_del = []
    for lane2 in lane.predecessors:
      if lane.road_id == lane2.road_id:
        lane.overlapped.add(lane2)
        lane2.overlapped.add(lane)
        to_del.append(lane2)
    for lane3 in to_del:
      lane.predecessors.remove(lane3)

    # merging
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

  def smooth_ref_line_bc_derivative(self):
    pass


  def build_lane_info(self):
    for road_id, road in self.roads.items():
      road.sort_lanes()
    self.split_lane_poly()
    self.compute_lane_topo()
    for road_id, road in self.roads.items():
      road.build_ref_line()
      road.resample_ref_line(5.0)
      road.compute_ref_line_bc_derivative()
    self.smooth_ref_line_bc_derivative()
    for road_id, road in self.roads.items():
      road.resample_ref_line(0.1, "cubic")

  def debug_print(self):
    for road_id, road in self.roads.items():
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
    for selected_poly in polygon_item.PolygonInteractor.seletion_set:
      focused_set.add(selected_poly.lane_id)
    plt.close()

  elif event.key == "a":
    selected = get_polys_in_region()
    polygon_item.PolygonInteractor.seletion_set.update(selected)
    polygon_item.PolygonInteractor.current_selection_set = set()
    fig.canvas.draw()
    fig.canvas.flush_events()
  elif event.key == "d":
    selected = get_polys_in_region()
    polygon_item.PolygonInteractor.seletion_set.difference_update(selected)
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


x_min = 1e30
x_max = -1e30
y_min = 1e30
y_max = -1e30
def update_world_box(xx, yy):
  global x_min, x_max, y_min, y_max
  x_min = min(x_min, min(xx))
  x_max = max(x_max, max(xx))
  y_min = min(y_min, min(yy))
  y_max = max(y_max, max(yy))


def run(focused_set2=set()):
  global fig, polys, focused_set, my_map
  global x_min, x_max, y_min, y_max
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
    my_map.roads[road_id].add_lane(Lane(lane_id, lane_subid, f["geometry"]["coordinates"]))

  my_map.build_lane_info()
  my_map.debug_print()

  # draw lane polygon
  color_idx = 0
  for road_id, road in my_map.roads.items():
    for lane_subid, lane in road.lanes.items():
      lane_poly = lane.poly
      xx = [x for x,y,z in lane_poly]
      yy = [y for x,y,z in lane_poly]

      poly = Polygon(np.column_stack([xx, yy]), animated=True, color = (0,0,0,0))
      ax.add_patch(poly)
      poly.lane_id = (road_id, lane_subid)
      polys.append(poly)

      p = polygon_item.PolygonInteractor(ax, poly)
      c = matplotlib.colors.to_rgb(cycle[color_idx])
      color_idx = (color_idx + 1) % len(cycle)
      p.my_color = (c[0], c[1], c[2], 0.3)
      p.my_color2 = (c[0], c[1], c[2], 0.6)

      polys2.append(sgeom.Polygon([(x,y) for x,y,z in lane_poly]))
      update_world_box(xx, yy)

  # for road_id, road in my_map.roads.items():
  #   if road_id != '557024172,0,0,67':
  #    continue
  #   for lane_subid, lane in road.lanes.items():
  #     #if lane_subid not in (1, 3, 5):
  #     if lane_subid not in (1,):
  #       continue
  #     #plt.plot(lane.left_bnd[0], lane.left_bnd[1], "*")
  #     plt.plot(lane.right_bnd[0], lane.right_bnd[1], "*")

  # draw reference lines
  for road_id, road in my_map.roads.items():
    plt.plot(road.ref_line[0], road.ref_line[1], "*")
    #bc_pts = xodr_exporter.compute_bc_derivative(road)
    #plt.plot([x for x,y in bc_pts], [y for x,y in bc_pts], "x")

  # draw center lines
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

  # keep aspect ratio to 1:1 in meters
  w = 1800
  h = 900
  dx = x_max - x_min
  dy = y_max - y_min
  if dx/dy > w/h:
    dy = dx/w*h
    y = (y_max + y_min) / 2
    y_min = y - dy / 2
    y_max = y + dy / 2
  else:
    dx = dy*w/h
    x = (x_min + x_max) / 2
    x_min = x - dx / 2
    x_max = x + dx / 2
  ax.set_xlim((x_min, x_max))
  ax.set_ylim((y_min, y_max))

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

  mng = plt.get_current_fig_manager()
  #mng.window.showMaximized()
  #mng.window.setGeometry(0,0,w,h)

  plt.show()
  return focused_set


if __name__ == '__main__':
  #focused_set = run()
  focused_set = {'557024172,0,0,52,2', '557024172,0,0,42,1', '557024172,0,0,42,3', '557024172,0,0,66,2', '557024172,0,0,52,0', '557024172,0,0,42,4', '557024172,0,0,16,1', '557024172,0,0,63,0', '557024172,0,0,16,4', '557024172,0,0,67,0', '557024172,0,0,53,2', '557024172,0,0,52,3', '557024172,0,0,67,5', '557024172,0,0,66,4', '557024172,0,0,67,4', '557024172,0,0,63,2', '557024172,0,0,42,2', '557024172,0,0,66,5', '557024172,0,0,53,1', '557024172,0,0,42,0', '557024172,0,0,16,0', '557024172,0,0,17,0', '557024172,0,0,67,1', '557024172,0,0,66,1', '557024172,0,0,17,3', '557024172,0,0,17,2', '557024172,0,0,52,1', '557024172,0,0,37,1', '557024172,0,0,63,4', '557024172,0,0,53,0', '557024172,0,0,53,3', '557024172,0,0,16,2', '557024172,0,0,67,3', '557024172,0,0,36,1', '557024172,0,0,66,3', '557024172,0,0,63,1', '557024172,0,0,16,3', '557024172,0,0,63,3', '557024172,0,0,17,1', '557024172,0,0,67,2', '557024172,0,0,37,0', '557024172,0,0,66,0', '557024172,0,0,36,2', '557024172,0,0,36,0', '557024172,0,0,63,5'}
  if focused_set:
    run(focused_set)

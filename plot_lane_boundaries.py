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

roads = {}

class Lane:
  def __init__(self, poly):
    self.poly = poly
    pass

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
    xodr_exporter.export(roads)

def run(focused_set2=set()):
  global fig, polys, focused_set, roads
  focused_set = focused_set2
  fig, ax = plt.subplots()
  pan_zoom = fig_manipulator.PanAndZoom(fig, scale_factor=1.6)
  g = geojson.load(open("out.json"))
  #print(g["type"])
  #print(g["name"])
  #print(g["features"][0])
  roads = OrderedDict()

  def on_pick(event):
    if event.mouseevent.key != 'control' or event.mouseevent.button != 1:
      return
    polygon_item.PolygonInteractor.picked = event.artist
    print(f"Lane[{event.artist.lane_id}] got picked.")
    fig.canvas.draw()
    fig.canvas.flush_events()

  # centerlines = {}
  # for f in g["features"]:
  #   if f["properties"]["layer"] == "center_line":
  #     lane_id = f["properties"]["id"]
  #     xx = [x for x,y,z in f["geometry"]["coordinates"]]
  #     yy = [y for x,y,z in f["geometry"]["coordinates"]]
  #     centerlines[lane_id] = (xx, yy)

  x_min = 1e30
  x_max = -1e30
  y_min = 1e30
  y_max = -1e30
  idx = 0
  for f in g["features"]:
    if f["properties"]["layer"] != "lane":
      continue
    lane_id = f["properties"]["id"]

    if focused_set and lane_id not in focused_set:
      continue

    pos = lane_id.rfind(",")
    road_id = lane_id[:pos]
    lane_subid = lane_id[pos+1:]
    lane_subid = int(lane_subid)
    if road_id not in roads:
      roads[road_id] = {}
    roads[road_id][lane_subid] = Lane(f["geometry"]["coordinates"])

  for road_id, road in roads.items():
    print(road_id)
    odict = OrderedDict([(k,v) for k, v in reversed(sorted(road.items()))])
    roads[road_id] = odict

  base_x = 257983.394766
  base_y = -49697.366495
  for road_id, road in roads.items():
    print(road_id)
    for lane_idx, (lane_subid, lane) in enumerate(road.items()):
      lane_geom = lane.poly
      for pt_idx, (x,y,z) in enumerate(lane_geom):
        lane_geom[pt_idx] = (x-base_x, y-base_y, z)

      print("\t", lane_subid)
      xx = [x for x,y,z in lane_geom]
      yy = [y for x,y,z in lane_geom]

      poly = Polygon(np.column_stack([xx, yy]), animated=True, color = (0,0,0,0))
      ax.add_patch(poly)
      poly.lane_id = (road_id, lane_subid)
      polys.append(poly)
      p = polygon_item.PolygonInteractor(ax, poly)
      c = matplotlib.colors.to_rgb(cycle[idx])
      p.my_color = (c[0], c[1], c[2], 0.3)
      p.my_color2 = (c[0], c[1], c[2], 0.6)

      polys2.append(sgeom.Polygon([(x,y) for x,y,z in lane_geom]))

      x_min = min(x_min, min(xx))
      x_max = max(x_max, max(xx))
      y_min = min(y_min, min(yy))
      y_max = max(y_max, max(yy))

      # split the lane polygon into two boundaries
      for idx in range(1, len(xx)-1):
        dot = (xx[idx-1]-xx[idx])*(xx[idx+1]-xx[idx]) + (yy[idx-1]-yy[idx])*(yy[idx+1]-yy[idx])
        d2a = (xx[idx-1]-xx[idx])*(xx[idx-1]-xx[idx]) + (yy[idx-1]-yy[idx])*(yy[idx-1]-yy[idx])
        d2b = (xx[idx+1]-xx[idx])*(xx[idx+1]-xx[idx]) + (yy[idx+1]-yy[idx])*(yy[idx+1]-yy[idx])
        dot /= math.sqrt(d2a*d2b+0.00000000001)
        if abs(dot) < 0.2:
          if lane_idx == 0:
            print(len(xx), idx)
            road.ref_line = (xx[:idx+1], yy[:idx+1])
            if len(road.ref_line[0]) == 2: # add more points for a straight line with only 2 points
              xx = [road.ref_line[0][0]*(5-a)/5 + road.ref_line[0][1]*a/5 for a in range(6)]
              yy = [road.ref_line[1][0]*(5-a)/5 + road.ref_line[1][1]*a/5 for a in range(6)]
              road.ref_line = (xx, yy)
          road[lane_subid].left_bnd = (xx[:idx+1], yy[:idx+1])
          road[lane_subid].right_bnd = (list(reversed(xx[idx+1:-1])), list(reversed(yy[idx+1:-1])))
          break
      idx = (idx + 1) % len(cycle)

  # draw reference lines
  for road_id, road in roads.items():
    plt.plot(road.ref_line[0], road.ref_line[1], "*")
    #bc_pts = xodr_exporter.compute_bc_derivative(road)
    #plt.plot([x for x,y in bc_pts], [y for x,y in bc_pts], "x")

  # draw center lines
  region_min_pt = np.array([x_min, y_min])
  region_max_pt = np.array([x_max, y_max])
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

  #plt.plot(xx, yy, marker="o")

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
    focused_set2 = set()
    for lane_id in sorted(list(focused_set)):
      print(lane_id)
      if lane_id.endswith(",0"):
        focused_set2.add(lane_id)
    run(focused_set)

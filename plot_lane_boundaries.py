import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import RectangleSelector
import polygon_item
import fig_manipulator
import geojson
import math

cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
region = None

def region_select_callback(eclick, erelease):
  'eclick and erelease are the press and release events'
  global region
  x1, y1 = eclick.xdata, eclick.ydata
  x2, y2 = erelease.xdata, erelease.ydata
  region = (x1, y1, x2, y2)
  #print("(%3.2f, %3.2f) --> (%3.2f, %3.2f)" % (region))
  #print(" The button you used were: %s %s" % (eclick.button, erelease.button))


def toggle_selector(event):
  if event.key == "enter":
    plt.close()


def run(my_region=None):
  fig, ax = plt.subplots()
  pan_zoom = fig_manipulator.PanAndZoom(fig, scale_factor=1.6)
  g = geojson.load(open("out.json"))
  #print(g["type"])
  #print(g["name"])
  #print(g["features"][0])

  def on_pick(event):
    if event.mouseevent.key != 'control' or event.mouseevent.button != 1:
      return
    polygon_item.PolygonInteractor.picked = event.artist
    #print(f"{event.artist}")
    fig.canvas.draw()
    fig.canvas.flush_events()

  # centerlines = {}
  # for f in g["features"]:
  #   if f["properties"]["layer"] == "center_line":
  #     lane_id = f["properties"]["id"]
  #     xx = [x for x,y,z in f["geometry"]["coordinates"]]
  #     yy = [y for x,y,z in f["geometry"]["coordinates"]]
  #     centerlines[lane_id] = (xx, yy)

  if my_region is not None:
    region_min_pt = np.array([my_region[0], my_region[1]])
    region_max_pt = np.array([my_region[2], my_region[3]])

  x_min = 1e30
  x_max = -1e30
  y_min = 1e30
  y_max = -1e30
  idx = 0
  for f in g["features"]:
    if f["properties"]["layer"] == "lane":
      lane_id = f["properties"]["id"]
      # if lane_id.find("557024173,0,0,7,") == -1:
      #   continue
      xx = [x for x,y,z in f["geometry"]["coordinates"]]
      yy = [y for x,y,z in f["geometry"]["coordinates"]]
      pts = [[x,y] for x,y,z in f["geometry"]["coordinates"]]
      if my_region is not None:
        r = np.logical_and(region_min_pt <= pts, pts <= region_max_pt)
        intersected = np.any(np.all(r, axis=1))
        if not intersected:
          continue

      if 0:
        plt.fill(xx, yy)
      else:
        poly = Polygon(np.column_stack([xx, yy]), animated=True, color = (0,0,0,0))
        ax.add_patch(poly)
        p = polygon_item.PolygonInteractor(ax, poly)
        c = matplotlib.colors.to_rgb(cycle[idx])
        p.my_color = (c[0], c[1], c[2], 0.3)
      x_min = min(x_min, min(xx))
      x_max = max(x_max, max(xx))
      y_min = min(y_min, min(yy))
      y_max = max(y_max, max(yy))
      #break

      # if 1:
      #   # split the lane polygon into two boundaries
      #   two_side = False
      #   for idx in range(1, len(xx)-1):
      #     dot = (xx[idx-1]-xx[idx])*(xx[idx+1]-xx[idx]) + (yy[idx-1]-yy[idx])*(yy[idx+1]-yy[idx])
      #     d2a = (xx[idx-1]-xx[idx])*(xx[idx-1]-xx[idx]) + (yy[idx-1]-yy[idx])*(yy[idx-1]-yy[idx])
      #     d2b = (xx[idx+1]-xx[idx])*(xx[idx+1]-xx[idx]) + (yy[idx+1]-yy[idx])*(yy[idx+1]-yy[idx])
      #     dot /= math.sqrt(d2a*d2b+0.000000001)
      #     if abs(dot) < 0.2:
      #       if lane_id.endswith(",0"):
      #         plt.plot(xx[:idx], yy[:idx], "-")
      #       plt.plot(xx[idx+1:-1], yy[idx+1:-1], "-")
      #       two_side = True
      #       break
      # else:
      #   plt.plot(xx, yy, "-")
    if f["properties"]["layer"] == "center_line":
      #print(f["properties"]["id"])
      xx = [x for x,y,z in f["geometry"]["coordinates"]]
      yy = [y for x,y,z in f["geometry"]["coordinates"]]
      pts = [[x,y] for x,y,z in f["geometry"]["coordinates"]]
      if my_region is not None:
        r = np.logical_and(region_min_pt <= pts, pts <= region_max_pt)
        intersected = np.any(np.all(r, axis=1))
        if not intersected:
          continue
      plt.plot(xx, yy, "--")
    idx = (idx + 1) % len(cycle)

  #plt.plot(xx, yy, marker="o")

  ax.set_xlim((x_min, x_max))
  ax.set_ylim((y_min, y_max))
  fig.canvas.mpl_connect('pick_event', on_pick)

  if my_region is None:
    # drawtype is 'box' or 'line' or 'none'
    toggle_selector.RS = RectangleSelector(ax, region_select_callback,
      drawtype='box', useblit=True, button=[1],  # don't use middle button
      minspanx=5, minspany=5, spancoords='pixels', interactive=True)
    plt.connect('key_press_event', toggle_selector)
    def update_selected_region(event):
      if toggle_selector.RS.active:
        toggle_selector.RS.update()
    plt.connect('draw_event', update_selected_region)

  plt.show()
  return region

if __name__ == '__main__':
  region = run()
  if region != None:
    run(region)

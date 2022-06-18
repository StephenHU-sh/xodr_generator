from scenariogeneration import xodr
import os
import math

def export_lane(odr, lane, lane_id):
    planview = xodr.PlanView()
    s = 0.0
    for idx in range(len(lane[0])-1):
        dx = lane[0][idx+1] - lane[0][idx]
        dy = lane[1][idx+1] - lane[1][idx]
        heading = math.atan2(dy, dx)
        dist = math.sqrt(dx*dx + dy*dy)
        #planview.add_geometry(xodr.Line(dist), heading)
        planview.add_fixed_geometry(xodr.Line(dist), lane[0][idx], lane[1][idx], heading, s)
        s += dist

    centerlane = xodr.Lane(lane_type=xodr.LaneType.median)
    lanesection = xodr.LaneSection(0, centerlane)
    lanesection.add_left_lane(xodr.Lane(lane_type=xodr.LaneType.median, a=0.3))
    lanesection.add_right_lane(xodr.Lane(lane_type=xodr.LaneType.median, a=0.3))

    left_lane_with_roadmark = xodr.Lane(a=4)
    left_lane_with_roadmark.add_roadmark(xodr.STD_ROADMARK_BROKEN)
    lanesection.add_left_lane(left_lane_with_roadmark)
    right_lane_with_roadmark = xodr.Lane(a=4)
    right_lane_with_roadmark.add_roadmark(xodr.STD_ROADMARK_SOLID)
    lanesection.add_right_lane(right_lane_with_roadmark)

    lanes = xodr.Lanes()
    lanes.add_lanesection(lanesection)

    road = xodr.Road(lane_id, planview, lanes)
    odr.add_road(road)

def export(lanes):
    # create the opendrive
    odr = xodr.OpenDrive("myroad")
    for idx, (lane_id, lane) in enumerate(lanes.items()):
        if lane_id.endswith(",0"):
            export_lane(odr, lane[0], idx)
    #odr.adjust_roads_and_lanes()
    odr.write_xml("test.xodr")
    print("Done")
#!/usr/bin/env python
from __future__ import print_function
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from pymavlink import mavutil
import pprint
import json

def get_column_labels(command_name):
    '''return dictionary of column labels if available'''
    cmd = cmd_reverse_lookup(command_name)
    if cmd == 0:
        return {}
    labels = {}
    enum = mavutil.mavlink.enums['MAV_CMD'][cmd]
    for col in enum.param.keys():
        labels[col] = make_column_label(command_name, enum.param[col], "P%u" % col)
    return labels

def getMavlinkEnums():
    mavlink_enums = {}
    for ent in mavutil.mavlink.enums:
        enum_dict = {}
        for cmd in mavutil.mavlink.enums[ent]:
            enum = mavutil.mavlink.enums[ent][cmd]
            name = enum.name
            name = name.replace(ent+'_', '')
            if 'ENUM_END' in name:
                continue
            enum_dict[name] = {'value':cmd, 'description':enum.description, 'param':enum.param}
            if enum.param:
                pass
        mavlink_enums[ent] = enum_dict
    return mavlink_enums

if __name__ == '__main__':
    mavlink_enums = getMavlinkEnums()
    # print(mavlink_enums.keys())
#     pprint.pprint(mavlink_enums['MAV_TYPE'])
    with open('mavlink_meta.txt', 'w+') as fid:
         fid.write(json.dumps(mavlink_enums, indent=4, separators=(',', ': ')))
         
    pprint.pprint(mavlink_enums)
        
    # arduplane
    mode_mapping_apm = mavutil.mode_mapping_apm
    
    # arducopter
    mode_mapping_acm = mavutil.mode_mapping_acm
    
    # ardurover
    mode_mapping_rover = mavutil.mode_mapping_rover
    
    # ardutracker
    mode_mapping_tracker = mavutil.mode_mapping_tracker
    
    # ardusub
    mode_mapping_sub = mavutil.mode_mapping_sub
    
    # PX4
    mainstate_mapping_px4 = mavutil.mainstate_mapping_px4
    
    # print(mavutil.mavlink.MAV_AUTOPILOT_PX4)
    # print(mavutil.mavlink.enums['MAV_AUTOPILOT'][3].description)
#!/usr/bin/env python

import xml.etree.ElementTree as ET
import json
import math
import os
from os.path import dirname, abspath

# get xml tree from urdf
tree = ET.parse(dirname(dirname(abspath(__file__)))+'/urdf/complete.urdf')
root = tree.getroot()

# open json files and store content
with open(dirname(abspath(__file__))+'/nico_humanoid_legged.json','r') as f1, open(dirname(abspath(__file__))+'/nico_humanoid_upper.json','r') as f2:
    json1 = json.load(f1)
    json2 = json.load(f2)

# iterate over all joints in the urdf and get their limits
for joint in root.findall('joint'):
    name  = joint.get('name')
    limit = joint.find('limit')
    # convert joint limits from rad to degrees
    if limit != None:
        lower = int(round(math.degrees(float(limit.get('lower')))))
        upper = int(round(math.degrees(float(limit.get('upper')))))
        # if a motor exists in the json with the same name as the joint, update limits
        if name in json1['motors']:
            json1['motors'][name]['angle_limit'] = [lower,upper]
        if name in json2['motors']:
            json2['motors'][name]['angle_limit'] = [lower,upper]

# remove old json files to avoid leftovers
try:
    os.mkdir(dirname(abspath(__file__))+'/generated')
except OSError:
    pass
try:
    os.remove(dirname(abspath(__file__))+'/generated/nico_humanoid_legged.json')
except OSError:
    pass
try:
    os.remove(dirname(abspath(__file__))+'/generated/nico_humanoid_upper.json')
except OSError:
    pass

# save updated json files
with open(dirname(abspath(__file__))+'/generated/nico_humanoid_legged.json', 'w') as f:
    json.dump(json1, f, indent = 2)
with open(dirname(abspath(__file__))+'/generated/nico_humanoid_upper.json', 'w') as f:
    json.dump(json2, f, indent = 2)

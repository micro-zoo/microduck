#!/usr/bin/env python3
"""Convert the fixture twin's closed=0 reference to the existing robotd coordinates.

Writes a new file; does not modify /etc/robot, serial registers or service state.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

JOINTS = dict(zip((20,21,22,23,24,30,31,32,33,34,10,11,12,13,14),
    ('left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle',
     'neck_pitch','head_pitch','head_yaw','head_roll','mouth',
     'right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle')))
RADIANS_PER_TICK=2*math.pi/4096
# Existing robotd mouth_target(0); preserve its public coordinates and 35-degree travel.
MOUTH_CLOSED=-5*math.pi/180

DEFAULT_MODEL=Path(__file__).resolve().parents[1]/'kinematics/assets/alpha/robot_walk.xml'

def model_limits(path):
    root=ET.parse(path).getroot();result={}
    for joint in root.iter('joint'):
        name=joint.get('name');bounds=joint.get('range')
        if bounds is not None:result[name]=list(map(float,bounds.split()))
        elif (limit:=joint.find('limit')) is not None and limit.get('lower') is not None:
            result[name]=[float(limit.get('lower')),float(limit.get('upper'))]
    result['mouth']=[MOUTH_CLOSED,30*math.pi/180]
    return result

def convert(document,extended=False,model=DEFAULT_MODEL):
    if document.get('mouth_reference')!='closed_is_zero':raise ValueError('Expected explicit closed_is_zero fixture reference')
    joints=document['joints']
    if len(joints)!=15 or {e['id'] for e in joints}!=set(JOINTS):raise ValueError('Expected exactly 15 distinct motor IDs')
    result=[]
    limits=model_limits(model) if extended else {}
    for e in joints:
        id=e['id'];zero=e['zero_tick']
        if type(id) is not int or e['name']!=JOINTS[id]:raise ValueError('Joint name/ID mismatch')
        if type(zero) not in (int,float) or not math.isfinite(zero) or not 0<=zero<=4095:raise ValueError('Invalid encoder zero')
        if e.get('reference_rad')!=0:raise ValueError('Expected fixture q=0 for every joint')
        if id==34:zero-=MOUTH_CLOSED/RADIANS_PER_TICK
        if extended:zero%=4096
        elif not 0<=zero<=4095:raise ValueError('Effective runtime zero outside the single-turn encoder')
        entry={'name':e['name'],'id':id,'zero_tick':zero}
        if extended:
            bounds=limits.get(e['name'])
            if not bounds or len(bounds)!=2 or not all(math.isfinite(q) for q in bounds) or not -math.pi<=bounds[0]<bounds[1]<=math.pi or bounds[1]-bounds[0]>=2*math.pi-2*RADIANS_PER_TICK:
                raise ValueError(f"{e['name']}: model limits do not select a unique turn")
            entry['limits_rad']=bounds
        result.append(entry)
    return {'position_mode':'extended_position','joints':result} if extended else {'joints':result}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('fixture',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--extended',action='store_true',help='Export Mode 4 coordinates with model joint limits')
    p.add_argument('--model',type=Path,default=DEFAULT_MODEL,help='MJCF or URDF joint limits')
    a=p.parse_args();result=convert(json.loads(a.fixture.read_text()),a.extended,a.model)
    # Refuse silent replacement of a robot-specific calibration.
    with a.output.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(f'Exported 15 runtime zeroes to {a.output}; mouth closed is -5 degrees in robotd, 0 in the twin.')
if __name__=='__main__':main()

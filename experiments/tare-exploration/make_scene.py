#!/usr/bin/env python3
"""Generate a bounded, occluded simulation; only the outer task boundary reaches TARE."""
from pathlib import Path
import json, hashlib
from xml.etree import ElementTree as E
root=E.Element('sdf',version='1.8'); world=E.SubElement(root,'world',name='default')
physics=E.SubElement(world,'physics',name='1ms',type='bullet-featherstone')
E.SubElement(physics,'max_step_size').text='0.001';E.SubElement(physics,'real_time_factor').text='1.0'
for lib,name in [('physics','Physics'),('sensors','Sensors'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('contact','Contact')]:
 p=E.SubElement(world,'plugin',filename='gz-sim-'+lib+'-system',name='gz::sim::systems::'+name)
 if name=='Sensors':E.SubElement(p,'render_engine').text='ogre2'
light=E.SubElement(world,'light',name='sun',type='directional')
for k,v in [('pose','0 0 10 0 0 0'),('diffuse','0.8 0.8 0.8 1'),('specular','0.2 0.2 0.2 1'),('direction','-0.5 0.1 -0.9'),('cast_shadows','true')]:E.SubElement(light,k).text=v
objects=[]
def box(name,x,y,z,sx,sy,sz,color='0.65 0.72 0.78 1'):
 m=E.SubElement(world,'model',name=name);E.SubElement(m,'static').text='true'
 E.SubElement(m,'pose').text=f'{x} {y} {z} 0 0 0';l=E.SubElement(m,'link',name='link')
 for tag in ['collision','visual']:
  q=E.SubElement(l,tag,name=tag);g=E.SubElement(q,'geometry');b=E.SubElement(g,'box');E.SubElement(b,'size').text=f'{sx} {sy} {sz}'
  if tag=='visual':
   material=E.SubElement(q,'material');E.SubElement(material,'ambient').text=color;E.SubElement(material,'diffuse').text=color
 objects.append(dict(name=name,center=[x,y,z],size=[sx,sy,sz]))
box('floor',0,0,-.1,40,40,.2,'0.65 0.65 0.65 1')
for name,x,y,sx,sy in [('north',0,6.5,13.2,.2),('south',0,-6.5,13.2,.2),('east',6.5,0,.2,13.2),('west',-6.5,0,.2,13.2)]:box(name,x,y,1.1,sx,sy,2.2)
box('east_occluding_partition',2,-2.25,1.0,.2,8.5,2.)
box('southwest_partition',-3.75,-2,1.,5.5,.2,2.)
box('northwest_box',-3.5,3,0.6,1.2,1.2,1.2,'0.7 0.35 0.2 1')
box('east_box',4.5,-3.5,0.6,1.,1.,1.2,'0.7 0.35 0.2 1')
p=Path(__file__).with_name('exploration.sdf');E.indent(root);E.ElementTree(root).write(p,encoding='utf-8',xml_declaration=True)
p.with_suffix('.json').write_text(json.dumps(dict(world_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),objects=objects,spawn=[0,0,.375,0],task_boundary_in_initial_body_xy=[-6.0,6.0,-6.0,6.0],planner_access='Outer task boundary only. Obstacle geometry is simulator/evaluation data, not a planner input.',ground_truth_role='evaluation only'),indent=2)+'\n')
print(p)

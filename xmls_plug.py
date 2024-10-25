import xml.etree.ElementTree as ET
import numpy as np
import math

def AA(theta):
    theta = theta*math.pi/180
    arr = np.array([[math.cos(theta),-1*math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    return arr
def AAZ(theta):
    theta = theta*math.pi/180
    arr = np.array([[math.cos(theta),-1*math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])
    return arr

def indent(elem, level=0):
    i = "\n" + level*"\t"
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "\t"
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

# join11 = ET.SubElement(model, 'link')
def link(name,fileloc,density,meshloc,color):
    link11 = ET.Element('link')
    link11.set('name', name)
    # '0 0 0 0 0 0'
    pose11 = ET.SubElement(link11,'pose')
    pose11.text=' '.join(str(i) for i in meshloc)
    pose11.set("degrees", "true")
    # inertia,密度
    inertia11 = ET.SubElement(link11,'inertial')
    inertia11.set("auto","true")
    ET.SubElement(inertia11,'density').text='{0}'.format(density)
    # collision
    collision11 = ET.SubElement(link11,'collision')
    collision11.set('name', name+'colli')
    geometry11 = ET.SubElement(collision11,'geometry')
    mesh11 = ET.SubElement(geometry11,'mesh')
    # 'file://home/cbb/gz_models/robot/cl1.STL'
    ET.SubElement(mesh11,'uri').text = fileloc
    ET.SubElement(mesh11,'scale').text = '0.1 0.1 0.1'
    # visual
    visual11 = ET.SubElement(link11,'visual')
    visual11.set('name', name+'visu')
    geometry11 = ET.SubElement(visual11,'geometry')
    mesh11 = ET.SubElement(geometry11,'mesh')
    # 'file://home/cbb/gz_models/robot/cl1.STL'
    ET.SubElement(mesh11,'uri').text = fileloc
    ET.SubElement(mesh11,'scale').text = '0.1 0.1 0.1'
    material11 = ET.SubElement(visual11,'material')
    ET.SubElement(material11,'ambient').text=' '.join(str(i) for i in color[0])
    ET.SubElement(material11,'diffuse').text=' '.join(str(i) for i in color[1])
    ET.SubElement(material11,'specular').text=' '.join(str(i) for i in color[2])
    return link11

model = ET.Element('model')
model.set("name", "robot")

xloc=6.55
yloc=5.079
locs = np.zeros((6,2))
for i in range(3):
    locs[i,0:2]=np.dot(AA(120*i),np.reshape([xloc,yloc],(2,1))).T
for i in range(3):
    locs[i+3,0:2]=np.dot(AA(120*i),np.reshape([-1*xloc,yloc],(2,1))).T

z=0
zz = 105*math.sin(15*math.pi/180)*0.1
for xloc,yloc,zang in zip(locs[:,0],locs[:,1],[0,120,240,0,120,240]):
    print(xloc,yloc)
    z=z+1
    fileloc = 'meshes/cl1.STL'
    for i in range(1,4):
        if i %2:
            j=1
            k=180
        else:
            j=-1
            k=0
        link11 = link('link{0}{1}'.format(z,i),fileloc,1000,[xloc,yloc,zz*i,k,13.5*j,zang],[[1,0,0,1],[1,0,0,1],[1,0,0,1]])
        model.append(link11)

    fileloc = 'meshes/chengg_down.STL'
    link11 = link('link{0}_down'.format(z),fileloc,1000,[xloc,yloc,0,0,180-13.5,zang],[[1,0,0,1],[1,0,0,1],[1,0,0,1]])
    model.append(link11)
    fileloc = 'meshes/chengg_up.STL'
    link11 = link('link{0}_up'.format(z),fileloc,1000,[xloc,yloc,zz*4,0,180-13.5,zang],[[1,0,0,1],[1,0,0,1],[1,0,0,1]])
    model.append(link11)

for i in range(5):
    j1loc = 'meshes/jiaol_gan.STL'
    join11 = link('join{0}'.format(i),j1loc,1000,[0,0,i*zz,0,0,0],[[1,0,0,1],[1,0,0,1],[1,0,0,1]])
    model.append(join11)

def joint(name,typ,locPart,loc,parent,child,dirPart,dir):
    joi11 = ET.Element('joint')
    joi11.set('name', name)
    joi11.set('type',typ)
    pose = ET.SubElement(joi11,'pose')
    pose.set('relative_to',locPart)
    pose.text=' '.join(str(i) for i in loc)
    ET.SubElement(joi11,'parent').text=parent
    ET.SubElement(joi11,'child').text=child

    axis= ET.SubElement(joi11,'axis')
    xyz = ET.SubElement(axis,'xyz')
    xyz.text=dir
    xyz.set('expressed_in',dirPart)
    limit = ET.SubElement(axis,'limit')
    ET.SubElement(limit,'lower').text='-3.14'
    ET.SubElement(limit,'upper').text='3.14'
    return joi11

joinloc=[0,0,0,0,0,0]
for i in range(1,7):
    chil = 'link{0}_down'.format(i)
    joint1 = joint('revjoin{0}1'.format(i),'revolute',chil,joinloc,'join0',chil,chil,'0 1 0')
    model.append(joint1)
for i in range(1,7):
    chil = 'link{0}_up'.format(i)
    joint1 = joint('revjoin{0}5'.format(i),'revolute',chil,joinloc,'join4',chil,chil,'0 1 0')
    model.append(joint1)
for i in range(1,7):
    chil = 'link{0}1'.format(i)
    joint1 = joint('revjoin{0}2'.format(i),'revolute',chil,joinloc,'join1',chil,chil,'0 1 0')
    model.append(joint1)
for i in range(1,7):
    chil = 'link{0}2'.format(i)
    joint1 = joint('revjoin{0}3'.format(i),'revolute',chil,joinloc,'join2',chil,chil,'0 1 0')
    model.append(joint1)
for i in range(1,7):
    chil = 'link{0}3'.format(i)
    joint1 = joint('revjoin{0}4'.format(i),'revolute',chil,joinloc,'join3',chil,chil,'0 1 0')
    model.append(joint1)

def linkClose(name,meshloc):
    link21 = ET.Element('link')
    link21.set('name',name)
    ET.SubElement(link21,'pose').text=' '.join(str(i) for i in meshloc)
    return link21

tpx1=-1*zz/(2*math.tan(13.5*math.pi/180))/2-xloc
tpx2=-1*zz/(2*math.tan(13.5*math.pi/180))/2+xloc
for i,j,z in zip(range(1,7),[0,0,120,120,240,240],[tpx1,tpx2,tpx1,tpx2,tpx1,tpx2]):
    tmp=np.reshape([z,yloc,zz/4],(3,1))
    loc=np.dot(AAZ(j),tmp).T
    link21 = linkClose('link{0}1_clo'.format(i),np.append(loc[0,:],np.array([0,0,0])))
    model.append(link21)
tmpv=zz/(2*math.sin(13.5*math.pi/180))/2
for i in range(1,6,2):
    chil='link{0}1_clo'.format(i)
    pare='link{0}_down'.format(i)
    joint1=joint('prevjoin{0}1'.format(i),'revolute',chil,[-1*tmpv,0,0,0,0,0],pare,chil,chil,'0 1 0')
    model.append(joint1)

#竖杆之间添加铰链
def addPlugin(parentl,childmodel,childlink):
    plu = ET.Element('plugin')
    plu.set('filename','libgz-sim-detachable-joint-system.so')
    plu.set('name','gz::sim::systems::DetachableJoint')
    ET.SubElement(plu,'parent_link').text=parentl
    ET.SubElement(plu,'child_model').text=childmodel
    ET.SubElement(plu,'child_link').text=childlink
    ET.SubElement(plu,'attach_topic').text='close_loop'
    return plu

for i in range(1,6,2):
    pare = 'link{0}1'.format(i)
    chil = 'link{0}1_clo'.format(i)
    plugin1 = addPlugin(pare,'robot',chil)
    model.append(plugin1)

fixJ=ET.Element('joint')
fixJ.set('name','fixJ')
fixJ.set('type','fixed')
ET.SubElement(fixJ,'parent').text='world'
ET.SubElement(fixJ,'child').text='join0'

root = ET.Element('sdf')
root.set('version',"1.7")
root.append(model)
indent(root)
# print()
tree = ET.ElementTree(root)

tree.write('/home/cbb/gz_models/robot/robot2.sdf', encoding='utf-8', xml_declaration=True)
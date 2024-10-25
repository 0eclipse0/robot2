# from pysdf import SDF, Link, Joint, Model
import sdformat14 as sdf
from sdformat14 import ConfigureResolveAutoInertials, ParserConfig
import sys
from gz.math7 import Vector3d, Inertiald, MassMatrix3d, Pose3d
import numpy as np

link = np.loadtxt("/home/cbb/sdf/locdata/link.csv",delimiter=',')
joint = np.loadtxt("/home/cbb/sdf/locdata/joint.csv",delimiter=',')
heng = np.loadtxt("/home/cbb/sdf/locdata/heng.csv",delimiter=',')
bjoint = np.loadtxt("/home/cbb/sdf/locdata/bjoint.csv",delimiter=',')
grip = np.loadtxt("/home/cbb/sdf/locdata/grip.csv",delimiter=',')
print(len(link))
# print(dir(sdf.Link()))
def sLink(sname,mass,inertia,loc,fname='__model__'):
    link = sdf.Link()
    link.set_name(sname)
    # link.set_pose_relative_to(fname)
    # link.set_raw_pose(Pose3d(1,2,3,4,5,6))
    # collision
    # colli = sdf.Collision()
    # colli.set_name = 
    #inertial
    exmass = mass
    ixx = inertia[0]
    iyy = inertia[1]
    izz = inertia[2]
    massMat = MassMatrix3d(exmass,Vector3d(ixx,iyy,izz),Vector3d.ZERO)
        # ,Vector3d(ixx,iyy,izz),Vector3d.ZERO
    exInertial = Inertiald()
    exInertial.auto = True
    link.set_inertial(exInertial)
    link.set_auto_inertia(True)
    #visual
    cl1 = sdf.Mesh()
    # loc='file://home/cbb/gz_models/robot/cl1.STL'
    cl1.set_uri(loc)
    cl1.set_optimization(sdf.MeshOptimization.CONVEX_DECOMPOSITION)
    geometry = sdf.Geometry()
    geometry.set_mesh_shape(cl1)
    geometry.set_type(sdf.GeometryType.MESH)
    visual = sdf.Visual()
    visual.set_name(link.name()+'v')
    visual.set_geometry(geometry)
    link.add_visual(visual)
    return link

def oneSec(mname):
    model = sdf.Model()
    model.set_name(mname)
    j1loc = 'file://home/cbb/gz_models/robot/join.STL'
    join1 = sLink('joint1',1,[1,2,3],j1loc)
    l1loc = 'file://home/cbb/gz_models/robot/cl1.STL'
    link1 = sLink('link11',1,[1,2,3],l1loc,join1.name())
    model.add_link(join1)
    model.add_link(link1)
    return model

def bJoint(num,mass):
    link = sdf.Link()
    return

def main():
    input_file = '/home/cbb/sdf/building_robot.sdf'
    root = sdf.Root()
    try:
        root.load(input_file)
    except sdf.SDFErrorsException as e:
        print(e, file=sys.stderr)
    
    sdfParserConfig = ParserConfig()
    # errosr = root.load_sdf_string(sdf,sdfParserConfig)
    sdfParserConfig.set_calculate_inertial_configuration(ConfigureResolveAutoInertials.SAVE_CALCULATION)
    root.resolve_auto_inertials([], sdfParserConfig)
    world = root.world_by_index(0)
    world.set_name('robot_env')
    # model = sdf.Model()
    # model.set_name('robot')
    
    # model.add_link(link)
    model = oneSec('robot')

    world.add_model(model)

    with open('/home/cbb/gz_models/robot/robot.sdf', "w") as f:
      f.write(root.to_string())

if __name__ == "__main__":
    main()
# def oneSec(num,mass,inertia):
#     l1 = Link(name = "link{0}".format(num))
#     l1.inertial.mass=mass
#     l1.inertial.inertia.ixx=inertia[0]
#     l1.visuals
#     return l1
# l1 = oneSec(11,0.009,[1,1,1])
# element = SDF(
#     Model(
#         l1,
#     ),
#     version = "1.8"
# )
# element.to_file("first.sdf")

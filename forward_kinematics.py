from numpy import *
from math import *


def rotv(axis, angle):
    if axis == 0:
        R = mat(array([1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle)]).reshape(3, 3))

    if axis == 1:
        R = mat(array([cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle)]).reshape(3, 3))
    if axis == 2:
        R = mat(array([cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1]).reshape(3, 3))

    return R
def planner_fk(angle):

    R1 = rotv(0, 1.5708) * rotv(1, 0) * rotv(2, 0)
    R2 = rotv(0, 1.5708) * rotv(1, -1.5708) * rotv(2, 0)
    R3 = rotv(0, -1.5708) * rotv(1, -1.5708) * rotv(2, 0)
    R4 = rotv(0, 1.5708) * rotv(1, -1.5708) * rotv(2, 0)
    R5 = rotv(0, -1.5708) * rotv(1, -1.5708) * rotv(2, 0)
    R6 = rotv(0, 1.5708) * rotv(1, -1.5708) * rotv(2, 0)
    R7 = rotv(0, 3.1416) * rotv(1, 3.1416) * rotv(2, 0)

    joint1 = mat(array([0.039551, -0.048766, 0.045]).reshape(3, 1))
    link1 = mat(array([-2.96387934653267E-05, 0.0268789023773296, 0.000903120063559949]).reshape(3, 1))
    joint2 = mat(array([0, 0.0415, 0]).reshape(3, 1))
    link2 = mat(array([-2.530405669883E-05, 0.0105892867487408, -0.0434690030331724]).reshape(3, 1))
    joint3 = mat(array([0, 0, -0.07]).reshape(3, 1))
    link3 = mat(array([-2.96387934653267E-05, 0.0268789023773296, 0.000903120063559949]).reshape(3, 1))
    joint4 = mat(array([0, 0.0415, 0]).reshape(3, 1))
    link4 = mat(array([-2.53040566988647E-05, 0.0105892867487408, -0.0434690030331725]).reshape(3, 1))
    joint5 = mat(array([0, 0, -0.07]).reshape(3, 1))
    link5 = mat(array([-2.96387934653128E-05, 0.0268789023773294, 0.000903120063559963]).reshape(3, 1))
    joint6 = mat(array([0, 0.0415, 0]).reshape(3, 1))
    link6 = mat(array([-2.52721157880595E-05, 0.0105759084400103, -0.0434978122730002]).reshape(3, 1))
    joint7 = mat(array([0, 0, -0.072]).reshape(3, 1))
    link7 = mat(array([0, 0.0071844, 0.00091123]).reshape(3, 1))

    point1 =joint1
    point2=joint1+R1*rotv(1,-angle[0])*(link1+joint2)
    point3=joint1+R1*rotv(1,-angle[0])*(link1+joint2+R2*rotv(0,-angle[1])*(link2+joint3))
    point4=joint1+R1*rotv(1,-angle[0])*(link1+joint2+R2*rotv(0,-angle[1])*(link2+joint3+R3*rotv(1,-angle[2])*(link3+joint4)))
    point5=joint1+R1*rotv(1,-angle[0])*(link1+joint2+R2*rotv(0,-angle[1])*(link2+joint3+R3*rotv(1,-angle[2])*(link3+joint4+R4*rotv(0,-angle[3])*(link4+joint5))))
    point6=joint1+R1*rotv(1,-angle[0])*(link1+joint2+R2*rotv(0,-angle[1])*(link2+joint3+R3*rotv(1,-angle[2])*(link3+joint4+R4*rotv(0,-angle[3])*(link4+joint5+R5*rotv(1,-angle[4])*(link5+joint6)))))
    point7=joint1+R1*rotv(1,-angle[0])*(link1+joint2+R2*rotv(0,-angle[1])*(link2+joint3+R3*rotv(1,-angle[2])*(link3+joint4+R4*rotv(0,-angle[3])*(link4+joint5+R5*rotv(1,-angle[4])*(link5+joint6+R6*rotv(0,-angle[5])*(link6+joint7))))))
    point8=joint1+R1*rotv(1,-angle[0])*(link1+joint2+R2*rotv(0,-angle[1])*(link2+joint3+R3*rotv(1,-angle[2])*(link3+joint4+R4*rotv(0,-angle[3])*(link4+joint5+R5*rotv(1,-angle[4])*(link5+joint6+R6*rotv(0,-angle[5])*(link6+joint7+R7*rotv(2,-angle[6])*(link7)))))))

    axis_x =mat(array([-1, 0, 0]).reshape(3, 1))
    axis_y =mat(array([0, -1, 0]).reshape(3, 1))
    axis_z =mat(array([0, 0, -1]).reshape(3, 1))

    axis1 =axis_y
    axis2 =rotv(1,-angle[0])*axis_x
    axis3 =rotv(1,-angle[0])*rotv(0,-angle[1])*axis_y
    axis4 =rotv(1,-angle[0])*rotv(0,-angle[1])*rotv(1,-angle[2])*axis_x
    axis5 =rotv(1,-angle[0])*rotv(0,-angle[1])*rotv(1,-angle[2])*rotv(0,-angle[3])*axis_y
    axis6 =rotv(1,-angle[0])*rotv(0,-angle[1])*rotv(1,-angle[2])*rotv(0,-angle[3])*rotv(1,-angle[4])*axis_x
    axis7 =rotv(1,-angle[0])*rotv(0,-angle[1])*rotv(1,-angle[2])*rotv(0,-angle[3])*rotv(1,-angle[4])*rotv(0,-angle[5])*axis_z

    jaco_point1 =cross(transpose(axis1),transpose(point2-point1))
    jaco_point2 =cross(transpose(axis2),transpose(point3-point1))
    jaco_point3 =cross(transpose(axis3),transpose(point4-point1))
    jaco_point4 =cross(transpose(axis4),transpose(point5-point1))
    jaco_point5 =cross(transpose(axis5),transpose(point6-point1))
    jaco_point6 =cross(transpose(axis6),transpose(point7-point1))
    jaco_point7 =cross(transpose(axis7),transpose(point8-point1))

    jaco_axis1=transpose(axis1)
    jaco_axis2=transpose(axis2)
    jaco_axis3=transpose(axis3)
    jaco_axis4=transpose(axis4)
    jaco_axis5=transpose(axis5)
    jaco_axis6=transpose(axis6)
    jaco_axis7=transpose(axis7)

    jaco_trans=mat(array([jaco_axis1,jaco_point1,jaco_axis2,jaco_point2,jaco_axis3,jaco_point3,jaco_axis4,jaco_point4,
                          jaco_axis5,jaco_point5,jaco_axis6,jaco_point6,jaco_axis7,jaco_point7]).reshape(7,6))
    jaco=transpose(jaco_trans)

    point=transpose(mat(array([point1,point2,point3,point4,point5,point6,point7,point8])))

    rot=rotv(1,-angle[0])*rotv(0,-angle[1])*rotv(1,-angle[2])*rotv(0,-angle[3])*rotv(1,-angle[4])*rotv(0,-angle[5])*rotv(2,-angle[6])

    return point8,rot,point,jaco

def rot2angax(*args):
    if len(args)==1:
        sg=1
        ep=0
    else:
        if args[1]>=0:
            sg=1
        else:
            sg=-1
        if len(args)==3:
            ep=abs(args[2])
            if ep>0.001:
                ep=0.001
        else:
            ep=0
    lll=mat(array([args[0][2,1]-args[0][1,2],args[0][0,2]-args[0][2,0],args[0][1,0]-args[0][0,1]]).reshape(3,1))
    abslll=linalg.norm(lll)

    if abslll > ep:
        angax = (atan2(abslll, trace(args[0])) / abslll) * lll
    elif abslll <= ep and trace(args[0]) <= 3 - ep:
        angax = mat(array([0, 0, 0]).reshape(3, 1))
    elif abslll <= ep and trace(args[0]) >= -1 + ep:
        if ((args[0][1, 0] >= 0)) and (args[0][2, 1] >= 0) and (args[0][0, 2] >= 0):
            nn = sg * mat(array(
                [sqrt((args[0][0, 0] + 1) / 2), sqrt((args[0][1, 1] + 1) / 2), sqrt((args[0][2, 2] + 1) / 2)]).reshape(
                3, 1))
        elif ((args[0][1, 0] >= 0)) and (args[0][2, 1] < 0) and (args[0][0, 2] <= 0):
            nn = sg * mat(array(
                [sqrt((args[0][0, 0] + 1) / 2), sqrt((args[0][1, 1] + 1) / 2), -sqrt((args[0][2, 2] + 1) / 2)]).reshape(
                3, 1))
        elif ((args[0][1, 0] < 0)) and (args[0][2, 1] <= 0) and (args[0][0, 2] >= 0):
            nn = sg * mat(array(
                [sqrt((args[0][0, 0] + 1) / 2), -sqrt((args[0][1, 1] + 1) / 2), sqrt((args[0][2, 2] + 1) / 2)]).reshape(
                3, 1))
        elif ((args[0][1, 0] <= 0)) and (args[0][2, 1] >= 0) and (args[0][0, 2] < 0):
            nn = sg * mat(array(
                [-sqrt((args[0][0, 0] + 1) / 2), sqrt((args[0][1, 1] + 1) / 2), sqrt((args[0][2, 2] + 1) / 2)]).reshape(
                3, 1))
        angax=pi*nn

    return angax



if __name__ =="__main__":
    angle=(0,0,0,0,0,0,0)
    point8,rot,point,jaco=planner_fk(angle)
    print (point8)
    # print rot
    # print point
    # print jaco
    # print rot2angax(mat(array([1,2,3,2,2,1,2,3,4]).reshape(3, 3)))
    # print 2.0/mat(array([1,2,3]).reshape(3, 1))
    


# print point1
# print point
# print jaco

# print axis7
# print rotv(0,pi)*link1
# print multiply(rotv(0,pi),link1)
# link2 =mat(array([0, 0, 1]).reshape(1,3))
# link3 =mat(array([0, 1, 0]).reshape(1,3))
# print cross(link2,link3)
# print link2
# print point2
# print transpose(point2)

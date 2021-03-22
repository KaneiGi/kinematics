from numpy import *
from math import *
from forward_kinematics import *

angle = (0, 0, 0, 0, 0, 0, 0)

point_target, rot_target, point_list_target, jacobian_target = planner_fk(angle)

kf = 1
km = 10000
K = mat(diag(array([kf, kf, kf, km, km, km])))
T = 1
dlt = 1
qk=mat(array([0, 0, 0, 0, 0, 0, 0]).reshape(7,1))
point_init, rot_init, point_list_init, jacobian_init = planner_fk([0, 0, 0, 0, 0, 0, 0])

cal_max = 100
error_point = point_target-point_init
error_rot=rot2angax(rot_target*transpose(rot_init))
error_vector=mat(array([error_point,error_rot]).reshape(6,1))
jacobian_renew=jacobian_init
V=0.5*transpose(error_vector)*K*error_vector
tau = transpose(jacobian_renew)
A = transpose(jacobian_renew) * K * jacobian_renew


for i in range(cal_max):
    Vm=V
    tau=transpose(jacobian_renew)
    A=transpose(jacobian_renew)*K*jacobian_renew
    D=T*(A+(0.5*V[0,0]+dlt)*mat(eye(7)))
    dqk=linalg.solve(D,tau)
    qk=qk+dqk*T
    qk2list=list()
    for j in range(7):
        qk2list.append(qk[j,0])
    point_renew, rot_renew, point_list_renew, jacobian_renew = planner_fk(qk2list)
    error_point = point_target - point_renew
    error_rot = rot2angax(rot_target * transpose(rot_renew))
    error_vector = mat(array([error_point, error_rot]).reshape(6, 1))
    V=0.5*transpose(error_vector)*K*error_vector
    error=linalg.norm(error_vector)
    if error<0.01:
        break
    # if V[0,0]>0.999*Vm[0,0]:
    #     break
print (qk2list)




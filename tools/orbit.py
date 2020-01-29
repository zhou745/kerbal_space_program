import numpy as np
epsilon=1e-8

def dir_by_ap_pe(target_ap,target_pe,apoapsis,periapsis,position,velocity_orb,vessel,conn):
    Mu = vessel.orbit.body.mass*conn.space_center.g
    r_vec = np.array(position())
    r_len = np.sqrt(np.dot(r_vec,r_vec)+epsilon)

    v_vec = np.array(velocity_orb()) 
    v_len = np.sqrt(np.dot(v_vec,v_vec)+epsilon)
    
    E = -Mu/r_len+0.5*v_len*v_len
    J = np.cross(r_vec,v_vec)   
    J2 = np.dot(J,J)

    MuEJ2 = np.sqrt(Mu*Mu+4*E*J2)

    rAp = (-Mu-MuEJ2)/(2*E)
    rPe = (-Mu+MuEJ2)/(2*E)

    dL_dApPe = np.array([np.sign(rAp-target_ap),np.sign(rPe-target_pe)])
    dApPe_dJ2 = np.array([-1,1])*J2/MuEJ2
    dJ2_dv = np.array([ (r_vec[1]*r_vec[1]+r_vec[2]*r_vec[2])*v_vec[0]-(r_vec[1]*v_vec[1]+r_vec[2]*v_vec[2])*r_vec[0],
                        (r_vec[0]*r_vec[0]+r_vec[2]*r_vec[2])*v_vec[1]-(r_vec[0]*v_vec[0]+r_vec[2]*v_vec[2])*r_vec[1],
                        (r_vec[1]*r_vec[1]+r_vec[0]*r_vec[0])*v_vec[2]-(r_vec[1]*v_vec[1]+r_vec[0]*v_vec[0])*r_vec[2] ])*2
    
    dv_dir = np.dot(dL_dApPe,dApPe_dJ2)*dJ2_dv
    dv_dir = dv_dir/np.sqrt(np.dot(dv_dir,dv_dir)+epsilon)
    loss = (np.abs(rAp-target_ap)+np.abs(rPe-target_pe))/(target_ap+target_pe)

    return(dv_dir,loss)

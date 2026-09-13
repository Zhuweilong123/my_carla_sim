"""Robust polyline/Frenet geometry helpers."""
import math
import numpy as np
from typing import List, Tuple

def _segments(path):
    pts=np.asarray([(p[0],p[1]) for p in path],dtype=float)
    if len(pts)<2: return pts,np.zeros(0),np.zeros((0,2))
    d=np.diff(pts,axis=0); lengths=np.linalg.norm(d,axis=1)
    tang=d/np.maximum(lengths[:,None],1e-9); return pts,lengths,tang

def find_match_points(xy_list: List[Tuple[float,float]], path: List[Tuple[float,float,float,float]], is_first_run=True, pre_match_index=0):
    if not path: return [0]*len(xy_list), [(x,y,0.0,0.0) for x,y in xy_list]
    pts,lengths,tang=_segments(path); cum=np.r_[0.0,np.cumsum(lengths)]
    indices=[]; projections=[]
    for x,y in xy_list:
        q=np.array([x,y]); best=(float('inf'),0,0.0)
        for i,t in enumerate(tang):
            u=float(np.dot(q-pts[i],t)); u=max(0.0,min(lengths[i],u))
            d=float(np.sum((q-(pts[i]+u*t))**2))
            if d<best[0]: best=(d,i,u)
        _,i,u=best; p=pts[i]+u*tang[i]; theta=math.atan2(tang[i,1],tang[i,0]); kappa=float(path[min(i,len(path)-1)][3])
        projections.append((float(p[0]),float(p[1]),theta,kappa)); indices.append(i)
    return indices, projections

def cal_s_map_fun(path, origin_xy):
    if not path: return []
    pts,lengths,_=_segments(path); cum=np.r_[0.0,np.cumsum(lengths)]
    idx,proj=find_match_points([origin_xy],path,True,0); i=idx[0]
    t=np.array([math.cos(proj[0][2]),math.sin(proj[0][2])]); s0=cum[i]+float(np.dot(np.array(origin_xy)-pts[i],t))
    return list(cum-s0)

def cal_projection_s_fun(path, match_index_list, xy_list, s_map):
    out=[]
    for idx,(x,y) in zip(match_index_list,xy_list):
        theta=path[idx][2]; t=np.array([math.cos(theta),math.sin(theta)])
        out.append(float(s_map[idx]+np.dot(np.array([x,y])-np.array(path[idx][:2]),t)))
    return out

def cal_s_l_fun(obs_xy_list, path, s_map):
    idx,projections=find_match_points(obs_xy_list,path,True,0); s=cal_projection_s_fun(path,idx,obs_xy_list,s_map); l=[]
    for (x,y),(_,_,theta,_) in zip(obs_xy_list,projections):
        l.append(float(-math.sin(theta)*(x-projections[len(l)][0])+math.cos(theta)*(y-projections[len(l)][1])))
    return s,l

def cal_s_l_deri_fun(xy_list,V_xy_list,a_xy_list,local_path_xy_opt,origin_xy):
    idx,projections=find_match_points(xy_list,local_path_xy_opt,True,0)
    out=[[],[],[],[],[],[],[]]
    for i,(_,_,theta,kappa) in enumerate(projections):
        n=np.array([-math.sin(theta),math.cos(theta)]); t=np.array([math.cos(theta),math.sin(theta)])
        vh=np.array(V_xy_list[i]); ah=np.array(a_xy_list[i]); l=float(np.dot(np.array(origin_xy)-np.array([projections[i][0],projections[i][1]]),n)); ds=float(np.dot(vh,t)/max(1e-3,1-kappa*l)); dl=float(np.dot(vh,n)); ddl=float(np.dot(ah,n)-kappa*(1-kappa*l)*ds*ds); lds=dl/ds if abs(ds)>1e-6 else 0.0; dds=float(np.dot(ah,t)+2*ds*ds*kappa*lds)/max(1e-3,1-kappa*l); ldds=(ddl-lds*dds)/(ds*ds) if abs(ds)>1e-6 else 0.0
        for arr,val in zip(out,[l,dl,ds,ddl,lds,dds,ldds]): arr.append(val)
    return tuple(out)
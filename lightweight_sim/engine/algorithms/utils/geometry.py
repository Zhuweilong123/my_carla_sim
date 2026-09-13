"""Polyline heading and curvature utilities."""
import math
import numpy as np

def cal_heading_kappa(points):
    if len(points)<2: return [0.0]*len(points),[0.0]*len(points)
    xy=np.asarray(points,dtype=float); d=np.diff(xy,axis=0); ds=np.linalg.norm(d,axis=1); theta=np.unwrap(np.arctan2(d[:,1],d[:,0])); theta=np.r_[theta[0],theta,theta[-1]]; theta=(theta[:-1]+theta[1:])/2
    k=np.zeros(len(xy)); dt=np.diff(theta); seg=np.maximum(ds,1e-6); k[1:-1]=np.sin(dt[:-1])/np.maximum((seg[:-1]+seg[1:])/2,1e-6); k[0]=k[1] if len(k)>1 else 0.0; k[-1]=k[-2] if len(k)>1 else 0.0
    return list(theta),list(k)
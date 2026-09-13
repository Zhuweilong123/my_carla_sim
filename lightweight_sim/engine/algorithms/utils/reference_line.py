"""Reference-line smoothing with an optional QP backend."""
from .geometry import cal_heading_kappa

def smooth_reference_line(points, w_cost_smooth=0.4, w_cost_length=0.3, w_cost_ref=0.3, x_thre=0.2, y_thre=0.2):
    if len(points)<2: return [(float(p[0]),float(p[1]),0.0,0.0) for p in points]
    xy=[(float(p[0]),float(p[1])) for p in points]
    try:
        import cvxopt
    except ImportError:
        cvxopt=None
    if cvxopt is not None and len(xy)>=3:
        try:
            import numpy as np
            n=len(xy); ref=np.asarray(xy).reshape(-1); A1=np.zeros((2*n-4,2*n)); A2=np.zeros((2*n-2,2*n))
            for i in range(n-2):
                A1[2*i,2*i:2*i+5:2]=[1,-2,1]; A1[2*i+1,2*i+1:2*i+6:2]=[1,-2,1]
            for i in range(n-1):
                A2[2*i,2*i:2*i+3:2]=[1,-1]; A2[2*i+1,2*i+1:2*i+4:2]=[1,-1]
            H=2*(w_cost_smooth*A1.T@A1+w_cost_length*A2.T@A2+w_cost_ref*np.eye(2*n)); f=-2*w_cost_ref*ref; lb=ref-np.tile([x_thre,y_thre],n); ub=ref+np.tile([x_thre,y_thre],n); G=np.r_[np.eye(2*n),-np.eye(2*n)]; h=np.r_[ub,-lb]
            res=cvxopt.solvers.qp(cvxopt.matrix(H),cvxopt.matrix(f),cvxopt.matrix(G),cvxopt.matrix(h)); vals=np.asarray(res['x']).reshape(-1); xy=list(zip(vals[0::2],vals[1::2]))
        except Exception:
            xy=[(float(p[0]),float(p[1])) for p in points]
    theta,kappa=cal_heading_kappa(xy); return [(float(x),float(y),float(theta[i]),float(kappa[i])) for i,(x,y) in enumerate(xy)]
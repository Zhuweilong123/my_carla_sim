"""Obstacle state and collision management."""
import math
import numpy as np
from typing import List, Tuple
from .data_types import Obstacle
class ObstacleManager:
    def __init__(self): self._obstacles=[]; self._next_id=1; self.last_collision_ids=[]
    def add_obstacle(self,obs): self._obstacles.append(obs); self._next_id=max(self._next_id,obs.id+1)
    def add_from_config(self,configs):
        for c in configs:
            self.add_obstacle(Obstacle(id=c.get("id",self._next_id),x=c["x"],y=c["y"],length=c.get("length",4.5),width=c.get("width",2.0),speed=c.get("speed",0.0),heading=c.get("heading",0.0),type=c.get("type","vehicle")))
    def step(self,dt):
        for o in self._obstacles: o.step(dt)
    def get_all(self): return self._obstacles
    def get_obstacle_xy_list(self): return [(o.x,o.y) for o in self._obstacles]
    @staticmethod
    def _rect(x,y,length,width,heading):
        c,s=math.cos(heading),math.sin(heading); local=np.array([[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]])
        return local@np.array([[c,-s],[s,c]]).T+np.array([[x,y]])
    @staticmethod
    def _sat(a,b):
        for corners in (a,b):
            for i in range(4):
                edge=corners[(i+1)%4]-corners[i]; axis=np.array([-edge[1],edge[0]])/max(np.linalg.norm(edge),1e-9)
                if np.max(a@axis)<np.min(b@axis) or np.max(b@axis)<np.min(a@axis): return False
        return True
    def check_collision(self,ego_x,ego_y,ego_length,ego_width,ego_phi):
        ego=self._rect(ego_x,ego_y,ego_length,ego_width,ego_phi); amin,amax=ego.min(0),ego.max(0)
        self.last_collision_ids = []
        for o in self._obstacles:
            obs=self._rect(o.x,o.y,o.length,o.width,o.heading); bmin,bmax=obs.min(0),obs.max(0)
            if amax[0]<bmin[0] or amin[0]>bmax[0] or amax[1]<bmin[1] or amin[1]>bmax[1]: continue
            if self._sat(ego,obs):
                self.last_collision_ids.append(o.id)
        return bool(self.last_collision_ids)
    def clear(self): self._obstacles.clear()
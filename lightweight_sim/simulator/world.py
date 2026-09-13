"""Road geometry and reference-line construction."""
import math
import numpy as np
from typing import List,Tuple,Optional
from ..algorithms.utils.geometry import cal_heading_kappa
from ..algorithms.utils.reference_line import smooth_reference_line
from .data_types import PathPoint,RoadDef,RoadSegment
class World:
    def __init__(self,road_def:Optional[RoadDef]=None):
        self.road_def=road_def or RoadDef(); self.lane_width=self.road_def.lane_width; self.num_lanes=self.road_def.num_lanes; self._raw_waypoints=[]; self._ref_path=[]; self._s_map=[]; self._generate_road()
    def _generate_road(self):
        points=[]
        if self.road_def.segments:
            for seg in self.road_def.segments:
                p=self._generate_segment(seg)
                if points and p and math.hypot(p[0][0]-points[-1][0],p[0][1]-points[-1][1])<0.01: p=p[1:]
                if points and p and math.hypot(p[0][0]-points[-1][0],p[0][1]-points[-1][1])>0.5: raise ValueError("Road segments are disconnected")
                points.extend(p)
        else: points=self._generate_straight(200,0)
        self._raw_waypoints=points
        if len(points)>=2:
            self._ref_path=self._build_ref_path(points); self._s_map=[0.0]
            for a,b in zip(self._ref_path[:-1],self._ref_path[1:]): self._s_map.append(self._s_map[-1]+math.hypot(b.x-a.x,b.y-a.y))
    def _generate_segment(self,seg):
        t=seg.type.lower(); p=seg.params
        if t=='straight': return self._generate_straight(p.get('length',100),p.get('heading',0),p.get('start',(0,0)),p.get('resolution',2))
        if t=='arc': return self._generate_arc(p.get('radius',50),p.get('angle',math.pi/2),p.get('center',(50,-50)),p.get('start_angle',0),p.get('resolution',2))
        if t=='waypoints': return [tuple(x) for x in p.get('points',[])]
        raise ValueError(f'Unknown road segment type: {t}')
    @staticmethod
    def _generate_straight(length,heading,start=(0,0),resolution=2):
        n=max(2,int(abs(length)/max(resolution,1e-3))+1); return list(zip(np.linspace(start[0],start[0]+length*math.cos(heading),n),np.linspace(start[1],start[1]+length*math.sin(heading),n)))
    @staticmethod
    def _generate_arc(radius,angle,center,start_angle=0,resolution=2):
        n=max(2,int(abs(radius*angle)/max(resolution,1e-3))+1); a=np.linspace(start_angle,start_angle+angle,n); return list(zip(center[0]+radius*np.cos(a),center[1]+radius*np.sin(a)))
    def _build_ref_path(self,points):
        if len(points)>150: points=points[::max(1,len(points)//100)]
        smoothed=smooth_reference_line(points)
        return [PathPoint(*p) for p in smoothed]
    @property
    def ref_path(self): return self._ref_path
    @property
    def ref_path_as_tuples(self): return [(p.x,p.y,p.theta,p.kappa) for p in self._ref_path]
    @property
    def s_map(self): return self._s_map
    @property
    def total_length(self): return self._s_map[-1] if self._s_map else 0.0
    def get_lane_center(self,lane_idx): return -self.num_lanes*self.lane_width/2+(lane_idx+0.5)*self.lane_width
    def get_lane_boundaries(self): return [-self.num_lanes*self.lane_width/2+i*self.lane_width for i in range(self.num_lanes+1)]
    def distance_to_reference(self, x, y):
        """Return the shortest distance from a point to the reference polyline."""
        if len(self._ref_path) < 2:
            return 0.0
        best = float("inf")
        for a, b in zip(self._ref_path[:-1], self._ref_path[1:]):
            dx = b.x - a.x
            dy = b.y - a.y
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                projection = 0.0
            else:
                projection = ((x - a.x) * dx + (y - a.y) * dy) / length_sq
                projection = max(0.0, min(1.0, projection))
            px = a.x + projection * dx
            py = a.y + projection * dy
            best = min(best, math.hypot(x - px, y - py))
        return best

    def is_on_road(self, x, y, margin=0.0):
        if not self._ref_path:
            return True
        distance = self.distance_to_reference(x, y)
        half_width = self.num_lanes * self.lane_width / 2.0
        return distance <= half_width + margin
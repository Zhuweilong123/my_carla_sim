"""Longitudinal PID producing physical acceleration in m/s^2."""
from collections import deque
class LongitudinalPIDController:
    def __init__(self,K_P=1.15,K_I=0.0,K_D=0.0,dt=0.05,error_threshold=1.0):
        self.K_P=K_P; self.K_I=K_I; self.K_D=K_D; self.dt=dt; self.error_threshold=error_threshold; self.target_speed=50.0; self.error_buffer=deque(maxlen=60)
    def control(self,current_speed_ms):
        error= self.target_speed-3.6*current_speed_ms; self.error_buffer.append(error)
        if len(self.error_buffer)>=2:
            integral=sum(self.error_buffer)*self.dt; derivative=(self.error_buffer[-1]-self.error_buffer[-2])/self.dt
        else: integral=0.0; derivative=0.0
        if abs(error)>self.error_threshold: integral=0.0; self.error_buffer.clear()
        accel=(self.K_P*error+self.K_I*integral+self.K_D*derivative)/3.6
        return max(-6.0,min(3.0,accel))
    def set_target(self,speed_kmh): self.target_speed=float(speed_kmh)
    def reset(self): self.error_buffer.clear()
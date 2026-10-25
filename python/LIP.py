import numpy as np
from SimDraw import*

class LIPPlanner:
    def __init__(self, zc: np.float32, dt: np.float32):
        self.zc_ = zc
        self.g_ = 9.81 
        self.w_ = np.sqrt(self.g_ / self.zc_)
        self.pc_ = np.array([0.0, 0.0])  # initial CoM position (x, y)
        self.dt_ = dt
        self.cxi_ = np.array([0.0, 0.0])  # initial capture point position
    
    def CoM_dynamics(self, cxi: np.ndarray, pc:np.ndarray) -> np.ndarray:
        dpc = self.w_ * (cxi - pc)
        return dpc

    def CP_dynamics (self, p: np.ndarray, cxi: np.ndarray) -> np.ndarray:
        # p: zmp
        # cxi: current capture point
        dcxi = self.w_ * (cxi - p)
        return dcxi
    
    def computeCoM(self, cxi: np.ndarray) -> np.ndarray:     
        dpc = self.CoM_dynamics(cxi, self.pc_)
        self.pc_ = self.pc_ + dpc * self.dt_
        return self.pc_.copy()  # Return a copy to avoid reference aliasing

    def computeCxi (self, p: np.ndarray):
        dcxi = self.CP_dynamics(p, self.cxi_)
        self.cxi_ = self.cxi_ + dcxi*self.dt_
        return self.cxi_.copy()


def main():
    zc = 0.4
    Tswing = 1.0
    dt = 0.001
    hip_width = 0.27

    lip_planner = LIPPlanner(zc, dt)
    print("created a lip_planner")
    
    # 4 seconds duration sampled at dt = 0.001s
    time = np.arange(0, 4.0, dt)
    cxi = 0.2 * np.ones([len(time), 2])  # Desired Capture Point at 0.2m
    zmp = np.zeros([len(time), 2])        # Stance foot / ZMP at (0, 0)
    
    # Foot positions (Left & Right contact frames)
    lf_pos = np.array([0.0, hip_width / 2.0, 0.0])
    rf_pos = np.array([0.0, -hip_width / 2.0, 0.0])

    # Initialize 3D real-time visualizer
    # decimate=25 renders every 25ms (40 FPS) for smooth real-time visualization
    sim_draw = SimDraw(zc=zc, hip_width=hip_width, decimate=25)
    print("Simulating real-time LIP...")
    
    com_data = []
    for i in range(len(time)):
        com = lip_planner.computeCoM(cxi[i, :])
        com_data.append(com)
        
        # Real-time 3D update
        sim_draw.update(com=com, zmp=zmp[i, :], cp=cxi[i, :], lf=lf_pos, rf=rf_pos)
    
    com_data = np.array(com_data)
    print("Simulation finished. Close window to exit.")
    sim_draw.keep()


if __name__ == "__main__":
    main()

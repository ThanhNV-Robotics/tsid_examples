import numpy as np
import matplotlib.pyplot as plt

class LIPPlanner:
    def __init__(self, zc: float, dt: float, m: float):
        self.zc_ = zc
        self.g_ = 9.81 
        self.m_ = m
        self.w_ = np.sqrt(self.g_ / self.zc_)
        self.pc_ = np.array([0.0, 0.0])  # initial CoM position (x, y)
        self.dt_ = dt
    
    def CoM_dynamics(self, cxi, pc):
        # dot(pc) = w * (cxi - pc)
        dpc = self.w_ * (cxi - pc)
        return dpc
    
    def computeCoM(self, cxi):
        dpc = self.CoM_dynamics(cxi, self.pc_)
        self.pc_ = self.pc_ + dpc * self.dt_
        return self.pc_.copy()  # Return a copy to avoid reference aliasing
class SimDraw:
    """
    3D Real-time visualizer for the Linear Inverted Pendulum (LIP) model.
    Visualizes:
      - Pendulum rod connecting ZMP (on ground z=0) to CoM (at height zc).
      - CoM point (with vertical projection line and ground shadow).
      - ZMP (Center of Pressure) marker on the ground.
      - Capture Point (CP / divergent component of motion) marker on the ground.
      - Foot sole outline around the ZMP.
      - Historical 3D trajectories of CoM and ground ZMP.
    """
    def __init__(self, zc: float = 0.4, 
                 xlim=(-0.2, 0.5), 
                 ylim=(-0.3, 0.3), 
                 zlim=(0.0, 0.6), 
                 decimate: int = 25, 
                 show_foot: bool = True):
        self.zc = zc
        self.decimate = decimate
        self.show_foot = show_foot
        self.step_count = 0
        
        # Interactive mode on for real-time rendering
        plt.ion()
        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.xlim = xlim
        self.ylim = ylim
        self.zlim = zlim
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_zlim(zlim)
        
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.set_title('Linear Inverted Pendulum (LIP) 3D Simulation')
        self.ax.view_init(elev=25, azim=-60)
        
        # Ground wireframe at z=0
        gx = np.linspace(xlim[0], xlim[1], 10)
        gy = np.linspace(ylim[0], ylim[1], 10)
        GX, GY = np.meshgrid(gx, gy)
        self.ax.plot_wireframe(GX, GY, np.zeros_like(GX), color='lightgray', alpha=0.4, lw=0.5)
        
        # Trajectory history buffers
        self.com_hist = []
        self.zmp_hist = []
        
        # 3D Graphic primitives
        self.rod, = self.ax.plot([], [], [], 'k-', lw=3, label='Pendulum Rod')
        self.com_pt, = self.ax.plot([], [], [], 'o', color='royalblue', markersize=12, label=r'CoM ($x_c$)')
        self.com_proj, = self.ax.plot([], [], [], 'b--', lw=1, alpha=0.4)
        self.com_shadow, = self.ax.plot([], [], [], 'o', color='cornflowerblue', markersize=6, alpha=0.5)
        
        self.zmp_pt, = self.ax.plot([], [], [], 's', color='crimson', markersize=8, label=r'ZMP ($p$)')
        self.cp_pt, = self.ax.plot([], [], [], 'X', color='darkorange', markersize=9, markeredgewidth=2, label=r'Capture Point ($\xi$)')
        
        self.foot_line, = self.ax.plot([], [], [], 'r-', lw=1.2, alpha=0.6, label='Foot Sole')
        
        self.com_traj, = self.ax.plot([], [], [], 'b-', lw=1.5, alpha=0.7, label='CoM Trajectory')
        self.zmp_traj, = self.ax.plot([], [], [], 'r:', lw=1.2, alpha=0.7, label='ZMP Trajectory')
        
        self.ax.legend(loc='upper right', fontsize=8)

    def update(self, com, zmp=None, cp=None, pause: float = 0.001):
        """
        Updates the 3D visualization.
        :param com: [x, y] or [x, y, z] CoM position. If 2D, height defaults to self.zc.
        :param zmp: [x, y] or [x, y, z] ZMP position on ground (defaults to origin).
        :param cp:  [x, y] or [x, y, z] Capture Point on ground (optional).
        :param pause: time in seconds to allow GUI events to process (default 0.001s).
        """
        self.step_count += 1
        if self.step_count % self.decimate != 0:
            return
        
        if not plt.fignum_exists(self.fig.number):
            return

        com_3d = np.array([com[0], com[1], com[2] if len(com) > 2 else self.zc])
        if zmp is not None:
            zmp_3d = np.array([zmp[0], zmp[1], zmp[2] if len(zmp) > 2 else 0.0])
        else:
            zmp_3d = np.array([0.0, 0.0, 0.0])

        self.com_hist.append(com_3d)
        self.zmp_hist.append(zmp_3d)

        # 1. Pendulum rod: connects ZMP to CoM
        self.rod.set_data([zmp_3d[0], com_3d[0]], [zmp_3d[1], com_3d[1]])
        self.rod.set_3d_properties([zmp_3d[2], com_3d[2]])

        # 2. CoM mass point
        self.com_pt.set_data([com_3d[0]], [com_3d[1]])
        self.com_pt.set_3d_properties([com_3d[2]])

        # 3. Ground projection & shadow
        self.com_proj.set_data([com_3d[0], com_3d[0]], [com_3d[1], com_3d[1]])
        self.com_proj.set_3d_properties([0.0, com_3d[2]])
        self.com_shadow.set_data([com_3d[0]], [com_3d[1]])
        self.com_shadow.set_3d_properties([0.0])

        # 4. ZMP marker
        self.zmp_pt.set_data([zmp_3d[0]], [zmp_3d[1]])
        self.zmp_pt.set_3d_properties([zmp_3d[2]])

        # 5. Foot sole boundary around ZMP
        if self.show_foot:
            fx = [-0.04, 0.08, 0.08, -0.04, -0.04]
            fy = [-0.03, -0.03, 0.03, 0.03, -0.03]
            foot_x = [zmp_3d[0] + dx for dx in fx]
            foot_y = [zmp_3d[1] + dy for dy in fy]
            self.foot_line.set_data(foot_x, foot_y)
            self.foot_line.set_3d_properties([zmp_3d[2]] * len(fx))

        # 6. Capture point (if provided)
        if cp is not None:
            cp_z = cp[2] if len(cp) > 2 else 0.0
            self.cp_pt.set_data([cp[0]], [cp[1]])
            self.cp_pt.set_3d_properties([cp_z])

        # 7. Update trajectory lines
        com_arr = np.array(self.com_hist)
        zmp_arr = np.array(self.zmp_hist)
        self.com_traj.set_data(com_arr[:, 0], com_arr[:, 1])
        self.com_traj.set_3d_properties(com_arr[:, 2])
        self.zmp_traj.set_data(zmp_arr[:, 0], zmp_arr[:, 1])
        self.zmp_traj.set_3d_properties(zmp_arr[:, 2])

        plt.pause(pause)

    def clear_history(self):
        """Clears trajectory history."""
        self.com_hist = []
        self.zmp_hist = []

    def keep(self):
        """Keeps the visualization window open after the simulation finishes."""
        plt.ioff()
        plt.show()


def main():
    zc = 0.4
    m = 15
    dt = 0.001

    lip_planner = LIPPlanner(zc, dt, m)
    print("created a lip_planner")
    
    # 4 seconds duration sampled at dt = 0.001s
    time = np.arange(0, 4.0, dt)
    cxi = 0.2 * np.ones([len(time), 2])  # Desired Capture Point at 0.2m
    zmp = np.zeros([len(time), 2])        # Stance foot / ZMP at (0, 0)
    
    # Initialize 3D real-time visualizer
    # decimate=25 renders every 25ms (40 FPS) for smooth real-time visualization
    sim_draw = SimDraw(zc=zc, decimate=25)
    print("Simulating real-time LIP...")
    
    com_data = []
    for i in range(len(time)):
        com = lip_planner.computeCoM(cxi[i, :])
        com_data.append(com)
        
        # Real-time 3D update
        sim_draw.update(com=com, zmp=zmp[i, :], cp=cxi[i, :])
    
    com_data = np.array(com_data)
    print("Simulation finished. Close window to exit.")
    sim_draw.keep()


if __name__ == "__main__":
    main()

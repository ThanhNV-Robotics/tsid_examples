import numpy as np
from LIP import LIPPlanner
from SimDraw import SimDraw

class WalkPlanner:
    
    def __init__(self, Tswing: np.float32, 
                 hip_width: np.float32,
                 zc: np.float32,
                 dt: np.float32):
        self.Tswing_ = Tswing # swing time
        self.hip_width_ = hip_width
        self.dt_ = dt
        self.lip_model_ = LIPPlanner(zc,dt)

        self.step_length_ = 0.0
        self.max_step_length_ = 0.4 # m
        self.desired_stand_leg = True # True = Left Leg, False  = Right Leg
        self.phi_com_ = 0.0 # phase variable of the CoM
        self.phi_left_ = 0.0 # phase variable for the left leg
        self.phi_right_ = 0.0 # phase variable for the right leg

        self.zmp_d_ = np.array([0.0, 0.0]) # zmp_ reference
        self.cxi_d_ = np.array([0.0, 0.0]) # capture point_ reference

        self.left_foot_pos = np.array([0.0,0.5*self.hip_width_,0.0])
        self.right_foot_pos = np.array([0.0,-0.5*self.hip_width_,0.0])

        self.cxi0_ = np.array([0.0, 0.0])  # initial capture point

        self.cxi_out_ = np.array([0.0, 0.0])
        self.com_out_ = np.array([0.0, 0.0])

        self.doneInitWalking = False
       

    def computeStepLength(self, vx: np.float32):
        # Clips between -max_step and +max_step
        self.step_length_ = np.clip(vx * self.Tswing_, 0.0 , self.max_step_length_)

    def init_walking (self):
        if self.doneInitWalking:
            return

        self.phi_com_ += self.dt_/self.Tswing_ # ramp up com phase variable

        self.cxi_d_[0] = 0.0
        self.cxi_d_[1] = 0.5*self.hip_width_ # move the CoM to the left side

        if (self.phi_com_ >= 1):
            # switch the zmp to the Left leg
            self.zmp_d_[0] = 0.0
            self.zmp_d_[1] = 0.5*self.hip_width_ # move the ZMP to the left foot
            self.phi_com_ = 0.0 # reset CoM phase variable
            print ("Init Walking Done!")

            # capture the initial capture point at current position
            self.cxi0_ = self.cxi_out_.copy()
            self.doneInitWalking = True

        else:
            b = np.e ** (self.lip_model_.w_*self.Tswing_)
            self.zmp_d_ = (self.cxi_d_ - b*self.cxi0_)/(1-b)
        

    def plan_walking (self):
        
        if not self.doneInitWalking:
            self.init_walking() # move CoM to left leg first
        
        else: # start swing the leg
            self.phi_com_ += self.dt_/self.Tswing_

            if self.desired_stand_leg: # if left leg is planned to stand

                self.cxi_d_[0] = 0.0
                self.cxi_d_[1] = -0.5*self.hip_width_ # move the CoM to the left side

                if (self.phi_com_ >= 1):
                    self.desired_stand_leg = not self.desired_stand_leg
                    self.phi_com_ = 0
                    self.zmp_d_[0] = 0.0
                    self.zmp_d_[1] = -0.5*self.hip_width_ # move the ZMP to the right foot
                    # capture the initial capture point at current position
                    self.cxi0_ = self.cxi_out_.copy()
                    print("Right Leg Stand, Left Leg Swing")

                else:
                    
                    b = np.e ** (self.lip_model_.w_*self.Tswing_)
                    self.zmp_d_ = (self.cxi_d_ - b*self.cxi0_)/(1-b)

            else: # right leg stand

                self.cxi_d_[0] = 0.0
                self.cxi_d_[1] = 0.5*self.hip_width_ # move the CoM to the left side

                if (self.phi_com_ >= 1):
                    self.desired_stand_leg = not self.desired_stand_leg # switch flag to change the leg
                    self.phi_com_ = 0
                    self.zmp_d_[0] = 0.0
                    self.zmp_d_[1] = 0.5*self.hip_width_ # move the ZMP to the right foot
                    # capture the initial capture point at current position
                    self.cxi0_ = self.cxi_out_.copy()
                    print("Left Leg Stand, Right Leg Swing")

                else:                    
                    b = np.e ** (self.lip_model_.w_*self.Tswing_)
                    self.zmp_d_ = (self.cxi_d_ - b*self.cxi0_)/(1-b)

        self.cxi_out_ = self.lip_model_.computeCxi(self.zmp_d_)
        self.com_out_ = self.lip_model_.computeCoM(self.cxi_out_)

            

def main ():
    # Model Parameters:
    zc = 0.5
    Tswing = 1.0
    dt = 0.001

    # Class Init
    walk_planner = WalkPlanner(Tswing=Tswing, hip_width=0.27, zc=zc, dt = dt)

    simu_duration = 6.0
    len_simu = int(simu_duration/dt)

    # Initialize 3D real-time visualizer
    # decimate=25 renders every 25ms (40 FPS) for smooth real-time visualization
    sim_draw = SimDraw(zc=zc, decimate=25)
    print("Simulating real-time LIP...")
    
    # main simulation loop
    while (True):

        walk_planner.plan_walking()

        sim_draw.update(com=walk_planner.com_out_, zmp=walk_planner.zmp_d_, cp=walk_planner.cxi_out_, 
                        lf = walk_planner.left_foot_pos,
                        rf = walk_planner.right_foot_pos)

    sim_draw.keep()

if __name__ == "__main__":
    main()
    print("done")


        
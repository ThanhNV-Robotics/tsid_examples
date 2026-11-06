import numpy as np
from LIP import LIPPlanner
from SimDraw import SimDraw
from utils.bezier_1D import*
class WalkPlanner:
    
    def __init__(self, Tswing: np.float32, 
                 hip_width: np.float32,
                 zc: np.float32,
                 dt: np.float32):
        self.Tswing_ = Tswing # swing time
        self.hip_width_ = hip_width
        self.dt_ = dt
        self.lip_model_ = LIPPlanner(zc,dt)

        
        self.max_step_length_ = 0.4 # m
        self.desired_stand_leg = True # True = Left Leg, False  = Right Leg
        self.phi_com_ = 0.0 # phase variable of the CoM
        self.phi_left_ = 0.0 # phase variable for the left leg
        self.phi_right_ = 0.0 # phase variable for the right leg

        self.zmp_d_ = np.array([0.0, 0.0]) # desired zmp_ 
        self.cxi_d_ = np.array([0.0, 0.0]) # desired capture point_

        self.left_foot_pos = np.array([0.0,0.5*self.hip_width_,0.0])
        self.right_foot_pos = np.array([0.0,-0.5*self.hip_width_,0.0])

        self.step_length_ = 0.1
        self.step_height_ = 0.1
        # quadratic Bezier peaks at half its middle point: 2*h gives a peak height of h
        self.Pz_d_ = np.array([0.0,2.0*self.step_height_,0.0]) # anchor point for z height trajectory
        self.Px_d_ = np.array([0.0,0.5*self.step_length_,self.step_length_]) # offsets from swing_start_
        self.swing_start_ = np.array([0.0, 0.0, 0.0]) # swing foot position at liftoff
        self.foothold_x_ = 0.0 # landing x of the current swing foot

        self.cxi0_ = np.array([0.0, 0.0])  # initial capture point

        self.cxi_out_ = np.array([0.0, 0.0])
        self.com_out_ = np.array([0.0, 0.0])

        self.doneInitWalking = False
       

    def computeStepLength(self, vx: np.float32):
        # Clips between -max_step and +max_step
        self.step_length_ = np.clip(vx * self.Tswing_, 0.0 , self.max_step_length_)

    def start_swing(self, swing_foot, stance_foot):
        # swing_foot: position of the foot will swing in this phase
        # stand_foot: position of the foot will stand in this phase
        
        # latch the liftoff position; the foot lands step_length ahead of the stance foot
        self.swing_start_ = swing_foot.copy()
        self.foothold_x_ = stance_foot[0] + self.step_length_ # landing x, also the target capture point
        dx = self.foothold_x_ - self.swing_start_[0]
        self.Px_d_ = np.array([0.0, 0.5*dx, dx])

    def update_swing_foot(self, swing_foot, phi):
        # Bezier gives the offset from liftoff, not an increment: assign, never accumulate
        swing_foot[0] = self.swing_start_[0] + bezier_1D(self.Px_d_, phi)
        swing_foot[2] = self.swing_start_[2] + bezier_1D(self.Pz_d_, phi)

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
            self.start_swing(self.right_foot_pos, self.left_foot_pos) # right leg swings first

        else:
            b = np.e ** (self.lip_model_.w_*self.Tswing_)
            self.zmp_d_ = (self.cxi_d_ - b*self.cxi0_)/(1-b)
        

    def plan_walking (self):
        #------------------------------------------
        # Init walking/ Moving CoM to the Left Side
        #------------------------------------------

        if not self.doneInitWalking:
            self.init_walking() # move CoM to left leg first

        #------------------------------------------
        # Start Walking
        #------------------------------------------
        else: # start swing the leg
            self.phi_com_ += self.dt_/self.Tswing_

            #------------------------------------------
            # Left Stand, Right Swing phase
            #------------------------------------------
            if self.desired_stand_leg: # if left leg is planned to stand

                # drive the capture point to the right foot's foothold by the end of the step
                self.cxi_d_[0] = self.foothold_x_
                self.cxi_d_[1] = -0.5*self.hip_width_

                if (self.phi_com_ >= 1):
                    self.update_swing_foot(self.right_foot_pos, 1.0) # land exactly on the foothold
                    self.desired_stand_leg = not self.desired_stand_leg
                    self.phi_com_ = 0
                    self.zmp_d_[0] = self.right_foot_pos[0]
                    self.zmp_d_[1] = -0.5*self.hip_width_ # move the ZMP to the right foot
                    # capture the initial capture point at current position
                    self.cxi0_ = self.cxi_out_.copy()
                    self.start_swing(self.left_foot_pos, self.right_foot_pos)
                    print("Right Leg Stand, Left Leg Swing")

                else:
                    b = np.e ** (self.lip_model_.w_*self.Tswing_)
                    self.zmp_d_ = (self.cxi_d_ - b*self.cxi0_)/(1-b)

                    # Compute swing foot trajectory
                    self.update_swing_foot(self.right_foot_pos, self.phi_com_)

            
            #------------------------------------------
            # Right Stand, Left Swing phase
            #------------------------------------------
            else: # right leg stand

                # drive the capture point to the left foot's foothold by the end of the step
                self.cxi_d_[0] = self.foothold_x_
                self.cxi_d_[1] = 0.5*self.hip_width_

                if (self.phi_com_ >= 1):
                    self.update_swing_foot(self.left_foot_pos, 1.0) # land exactly on the foothold
                    self.desired_stand_leg = not self.desired_stand_leg # switch flag to change the leg
                    self.phi_com_ = 0
                    self.zmp_d_[0] = self.left_foot_pos[0]
                    self.zmp_d_[1] = 0.5*self.hip_width_ # move the ZMP to the left foot
                    # capture the initial capture point at current position
                    self.cxi0_ = self.cxi_out_.copy()
                    self.start_swing(self.right_foot_pos, self.left_foot_pos)
                    print("Left Leg Stand, Right Leg Swing")

                else:
                    b = np.e ** (self.lip_model_.w_*self.Tswing_)
                    self.zmp_d_ = (self.cxi_d_ - b*self.cxi0_)/(1-b)

                    # compute desired foot position
                    self.update_swing_foot(self.left_foot_pos, self.phi_com_)

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


        
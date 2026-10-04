#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Robot_Simulator.h"
#include <Eigen/Dense>
#include <mujoco/mujoco.h>

using namespace std;
using namespace Eigen;
const string XML_PATH = "models/mjcf/right_leg_scene.xml";
const string URDF_PATH = "models/biped/biped.urdf";

int main(int argc, char **argv) {
  std::cout << "MuJoCo version: " << mj_versionString() << std::endl;

  // load mujoco model from xml
  char error[1024] = "";
  mjModel *mj_model =
      mj_loadXML(XML_PATH.c_str(), nullptr, error, sizeof(error));

  if (mj_model == nullptr) {
    std::cerr << "Error: " << error << std::endl;
    return 1;
  }
  mjData *mj_data = mj_makeData(mj_model);
  std::cout << "MuJoCo model loaded successfully" << std::endl;

  // Init an initial configuration
  VectorXd q_init(mj_model->nv);
  q_init << 0.05, 0, 0.0, 0.04, 0, -0.05;
  // set to mj_data
  for (int i = 0; i < mj_model->nv; i++) {
    mj_data->qpos[i] = q_init[i];
    mj_data->qvel[i] = 0.0;
  }
  mj_forward(mj_model, mj_data);

  Robot_Simulator rb_sim(mj_model, mj_data);
  rb_sim.iniGLFW();
  rb_sim.createWindow("Biped Robot Leg", false);

  std::cout << "Init Robot Simulator" << endl;

  while (!rb_sim.shouldClose()) {
    double sim_start = mj_data->time;

    while (mj_data->time - sim_start < 1.0 / 60.0 && rb_sim.runSim) {

      rb_sim.applyPerturbation();
      mj_step(mj_model, mj_data);
    }
    rb_sim.updateScene();
  }

  // clean up
  rb_sim.Close();
  return 0;
}
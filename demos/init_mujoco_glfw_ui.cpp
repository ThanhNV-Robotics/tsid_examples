#include <cstdio>
#include <string>

#include <mujoco/mujoco.h>

#include "Robot_Simulator.h"

const std::string BIPED_MODEL_DIR = "models/mjcf";

int main(int argc, char** argv) {
  std::printf("MuJoCo version %s\n", mj_versionString());

  const std::string default_model = BIPED_MODEL_DIR + "/right_leg_scene.xml";
  const std::string model_path = (argc > 1) ? argv[1] : default_model;

  char loadError[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, loadError, sizeof(loadError));
  if (!m) {
    std::fprintf(stderr, "failed to load %s: %s\n", model_path.c_str(), loadError);
    return 1;
  }
  mjData* d = mj_makeData(m);

  UIctr ui(m, d);
  ui.iniGLFW();
  ui.enableTracking();
  ui.createWindow("BipedMjcCpp", /*saveVideo=*/false);

  double simstart = d->time;
  while (!glfwWindowShouldClose(ui.window)) {
    simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0 && ui.runSim) {
      ui.applyPerturbation();
      mj_step(m, d);
    }
    ui.updateScene();
  }

  // UIctr::Close() (invoked by the window-close callback above) already
  // frees mj_model/mj_data and terminates GLFW.
  return 0;
}

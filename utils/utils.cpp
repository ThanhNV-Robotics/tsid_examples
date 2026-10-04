#include "utils.h"
#include <cstdio>
#include <iostream>

namespace utils {

void loadMjXml(mjModel *&m, mjData *&d, const string xml_path) {
  char loadError[1024] = "";
  m = mj_loadXML(xml_path.c_str(), nullptr, loadError, sizeof(loadError));

  if (!m) {
    std::fprintf(stderr, "Failed to load MuJoCo scene '%s': %s\n",
                 xml_path.c_str(), loadError);
    d = nullptr;
    return;
  }
  d = mj_makeData(m);
}

vector<string> getJointNames(const mjModel *m) {
  vector<string> joint_names;
  if (!m)
    return joint_names;

  joint_names.reserve(m->njnt);
  for (int i = 0; i < m->njnt; ++i) {
    const char *name = mj_id2name(m, mjOBJ_JOINT, i);
    joint_names.emplace_back(name ? name
                                  : "unnamed_joint_" + std::to_string(i));
  }
  return joint_names;
}

void printMjModelInfo(mjModel *m) {
  vector<string> joint_names = getJointNames(m);
  // print joint names
  for (int i = 0; i < joint_names.size(); i++) {
    std::cout << "Joint " << i << " :" << joint_names[i] << endl;
  }
  std::cout << "Model nv: " << m->nv << endl;
  std::cout << "Model nq: " << m->nq << endl;
}

void setRobotInitConfiguration(mjModel *&m, mjData *&d, Eigen::VectorXd q0) {
  if (!m || !d)
    return;
  // check dimension consistency
  if (q0.size() != m->nq) {
    std::cerr << "Error: Dimension mismatch! q_0 size is " << q0.size()
              << ", but model m->nq is " << m->nq << std::endl;
    return;
  }
  // assign the mujoco qpos and zero to velocity
  for (int i = 0; i < m->nq; i++) {
    d->qpos[i] = q0(i);
  }
  for (int i = 0; i < m->nv; i++) {
    d->qvel[i] = 0.0;
  }
  mj_forward(m, d);
  return;
}

} // namespace utils

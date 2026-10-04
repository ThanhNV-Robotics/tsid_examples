#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

using namespace std;

namespace utils {
vector<string> getJointNames(const mjModel *m);

void loadMjXml(mjModel *&m, mjData *&d, const string xml_path);
void printMjModelInfo(mjModel *m);
void setRobotInitConfiguration(mjModel *&m, mjData *&d, Eigen::VectorXd q0);

} // namespace utils

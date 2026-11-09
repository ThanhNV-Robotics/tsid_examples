#pragma once

#include <string>

#include <Eigen/Dense>

namespace utils
{

// Sole corners of a foot link, from its collision box in the URDF: the box face pointing most
// against `normal` (with several boxes, the lowest one along -normal), as 4 points in the link
// frame (= Pinocchio body frame), ordered (-i,-j), (-i,+j), (+i,-j), (+i,+j) over the face's two
// in-plane axes. Used for the TSID contact points and the foot height of the contact fusion.
// false (warning on stderr) if the URDF cannot be parsed, the link is missing or has no box.
bool soleCornersFromUrdf(const std::string &urdf_path, const std::string &link_name,
                         const Eigen::Vector3d &normal, Eigen::Matrix3Xd &corners);

}  // namespace utils

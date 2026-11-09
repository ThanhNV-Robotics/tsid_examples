#include "common/urdf_geometry.h"

#include <cmath>
#include <iostream>
#include <limits>

#include <urdf_parser/urdf_parser.h>

namespace utils
{

bool soleCornersFromUrdf(const std::string &urdf_path, const std::string &link_name,
                         const Eigen::Vector3d &normal, Eigen::Matrix3Xd &corners)
{
    if (urdf_path.empty())
        return false;
    const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_path);
    if (!urdf_model)
    {
        std::cerr << "[soleCornersFromUrdf] Warning: failed to parse URDF '" << urdf_path << "'" << std::endl;
        return false;
    }
    const urdf::LinkConstSharedPtr link = urdf_model->getLink(link_name);
    if (!link)
    {
        std::cerr << "[soleCornersFromUrdf] Warning: '" << link_name
                  << "' is not a URDF link, cannot read its collision geometry" << std::endl;
        return false;
    }

    // Among the link's collision boxes, take the face pointing most against the
    // contact normal (the sole); with several boxes, the one lowest along -normal.
    // URDF link frames coincide with Pinocchio's body frames, so the result is
    // directly in the contact frame.
    double best_height = std::numeric_limits<double>::infinity();
    for (const auto &collision : link->collision_array)
    {
        if (!collision || !collision->geometry || collision->geometry->type != urdf::Geometry::BOX)
            continue;
        const auto box = std::static_pointer_cast<const urdf::Box>(collision->geometry);
        const urdf::Pose &pose = collision->origin;
        const Eigen::Vector3d center(pose.position.x, pose.position.y, pose.position.z);
        const Eigen::Matrix3d R =
            Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z).toRotationMatrix();
        const Eigen::Vector3d half(box->dim.x / 2.0, box->dim.y / 2.0, box->dim.z / 2.0);

        int k = 0;
        for (int a = 1; a < 3; ++a)
        {
            if (std::abs(R.col(a).dot(normal)) > std::abs(R.col(k).dot(normal)))
                k = a;
        }
        const double s = (R.col(k).dot(normal) > 0.0) ? -1.0 : 1.0;
        const Eigen::Vector3d face_center = center + s * half(k) * R.col(k);
        const double height = face_center.dot(normal);
        if (height >= best_height)
            continue;
        best_height = height;

        // Corners ordered (-i,-j), (-i,+j), (+i,-j), (+i,+j) over the face's two in-plane axes
        const int i = (k == 0) ? 1 : 0;
        const int j = 3 - k - i;
        corners.resize(3, 4);
        int c = 0;
        for (double si : {-1.0, 1.0})
        {
            for (double sj : {-1.0, 1.0})
            {
                corners.col(c++) = face_center + si * half(i) * R.col(i) + sj * half(j) * R.col(j);
            }
        }
    }
    if (!std::isfinite(best_height))
    {
        std::cerr << "[soleCornersFromUrdf] Warning: link '" << link_name << "' has no collision box" << std::endl;
        return false;
    }
    return true;
}

}  // namespace utils

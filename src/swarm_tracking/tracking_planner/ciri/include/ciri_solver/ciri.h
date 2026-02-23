#ifndef _CIRI_H_
#define _CIRI_H_

#include <ciri_utils/lbfgs.h>
#include <ciri_utils/sdlp.h>
#include <ciri_utils/geometry_utils.h>
#include <ciri_utils/polytope.h>
#include <ciri_utils/ellipsoid.h>
#include <ciri_utils/mvie.h>
#include <memory>

namespace ciri {
    using namespace optimization_utils;
    using namespace geometry_utils;
    using namespace type_utils;
    using namespace std;

    class CIRI {
    private:
        double robot_r_{0};
        int iter_num_{1};

        Ellipsoid sphere_template_;
        Polytope optimized_polytope_;

    private:

/**
 * @brief findEllipsoid: find maximum ellipsoid with RILS
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param out_ell the output ellipsoid
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */
        void findEllipsoid(
        const Eigen::Matrix3Xd &pc,
        const Eigen::Vector3d &a,
        const Eigen::Vector3d &b,
        Ellipsoid &out_ell);

        static void findTangentPlaneOfSphere(const Eigen::Vector3d &center, const double &r,
                                             const Eigen::Vector3d &pass_point,
                                             const Eigen::Vector3d &seed_p,
                                             Eigen::Vector4d &outter_plane);

    public:
        CIRI() = default;

        ~CIRI() = default;

        typedef shared_ptr<CIRI> Ptr;

        void setupParams(double robot_r, int iter_num);

        RET_CODE comvexDecomposition(const Eigen::MatrixX4d &bd,
                                     const Eigen::Matrix3Xd &pc,
                                     const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b);

        RET_CODE comvexDecomposition(const Eigen::MatrixX4d &bd,
                                     const Eigen::Matrix3Xd &pc,
                                     const vector<double> & pc_radius,
                                     const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b);


        void getPolytope(Polytope &optimized_poly);
    };
}
#endif
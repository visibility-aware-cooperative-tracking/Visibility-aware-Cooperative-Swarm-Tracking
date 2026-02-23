#ifndef _MARS_UTILS_TYPE_H_
#define _MARS_UTILS_TYPE_H_


#include <ciri_utils/ellipsoid.h>
#include <ciri_utils/lbfgs.h>


namespace optimization_utils {

    using namespace geometry_utils;
    using namespace type_utils;

    class MVIE {
    public:
        MVIE() = default;

        ~MVIE() = default;

        static void chol3d(const Eigen::Matrix3d &A,
                           Eigen::Matrix3d &L);

        static bool smoothedL1(const double &mu,
                               const double &x,
                               double &f,
                               double &df);

        static double costMVIE(void *data,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &grad);

        // R is also assumed to be a rotation matrix
        static bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                       Ellipsoid &ellipsoid);

    };
}

#endif

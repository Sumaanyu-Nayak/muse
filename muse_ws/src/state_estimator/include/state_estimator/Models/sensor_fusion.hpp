#ifndef KF_SENSOR_FUSION_HPP
#define KF_SENSOR_FUSION_HPP

#include "state_estimator/Observers/EKF.hpp"
#include <unsupported/Eigen/MatrixFunctions>

namespace state_estimator
{
                                            //   N1,N2,N3,N4,N5,N6,N7 
    class KFSensorFusion : public EKFBase<double,6, 3, 3, 6, 3, 6, 6>
    {
    public:
        KFSensorFusion (double t0, const Eigen::Matrix<double,6,1> &xhat0, 
                       const Eigen::Matrix<double,6,6> &P0, const Eigen::Matrix<double,6,6> &Q, 
                       const Eigen::Matrix<double,3,3> &R, bool lidar, bool slippage);
        ~KFSensorFusion();

        void predict(double t, const Eigen::Matrix<double,3,1> &u) override;
        void update(double t, const Eigen::Matrix<double,3,1> &z);
        void setMatricesSF(bool slippage);

    protected:
        Eigen::Matrix<double,6,1> calc_f(double t, const Eigen::Matrix<double,6,1> &x, const Eigen::Matrix<double,3,1> &u) override;
        Eigen::Matrix<double,6,6> calc_F(double t, const Eigen::Matrix<double,6,1> &x, const Eigen::Matrix<double,3,1> &u) override;
        Eigen::Matrix<double,3,1> calc_h(double t, const Eigen::Matrix<double,6,1> &x);
        Eigen::Matrix<double,3,6> calc_H(double t, const Eigen::Matrix<double,6,1> &x);
        
        // predict
        double dt;
        Eigen::Matrix<double,6,1> fx{Eigen::Matrix<double,6,1>::Zero()};
        Eigen::Matrix<double,6,6> F{Eigen::Matrix<double,6,6>::Zero()};
        Eigen::Matrix<double,6,6> G{Eigen::Matrix<double,6,6>::Zero()};
        Eigen::Matrix<double,12,12> M{Eigen::Matrix<double,12,12>::Zero()};
        Eigen::Matrix<double,12,12> s{Eigen::Matrix<double,12,12>::Zero()};
        Eigen::Matrix<double,6,6> Phi{Eigen::Matrix<double,6,6>::Zero()};
        Eigen::Matrix<double,6,6> Gamma{Eigen::Matrix<double,6,6>::Zero()};

        // update
        Eigen::Matrix<double,6,1> xtilde{Eigen::Matrix<double,6,1>::Zero()};
        Eigen::Matrix<double,3,1> yhat{Eigen::Matrix<double,3,1>::Zero()};
        Eigen::Matrix<double,3,6> H{Eigen::Matrix<double,3,6>::Zero()};
        Eigen::Matrix<double,6,3> K{Eigen::Matrix<double,6,3>::Zero()};

        // calc_f
        Eigen::Matrix<double,3,1> v{Eigen::Matrix<double,3,1>::Zero()};

    }; // end class KFSensorFusion
} // end namespace state_estimator

#endif
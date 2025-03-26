#include "state_estimator/Models/sensor_fusion.hpp"

namespace state_estimator
{
    KFSensorFusion::KFSensorFusion(double t0, const Eigen::Matrix<double,6,1> &xhat0,
                                   const Eigen::Matrix<double,6,6> &P0, const Eigen::Matrix<double,6,6> &Q, 
                                   const Eigen::Matrix<double,3,3> &R, bool lidar, bool slippage)
    : EKFBase(t0,xhat0,P0,Q,R)
    {
        // setMatricesSF(slippage);
    }

    KFSensorFusion::~KFSensorFusion()
    {
    }

    void KFSensorFusion::predict(double t, const Eigen::Matrix<double,3,1> &u)
    {
        dt = t-this->t_prev;

        fx = calc_f(t,this->xhat, u);
        F = calc_F(t,this->xhat,u);

        G = Eigen::Matrix<double,6,6>::Identity();

        for (int i=0;i<3;i++)
        {
            G(i,i)=-1;
        }

        M.block(0,0,6,6) = F*dt;
        M.block(0,6,6,6) = G*dt;
        M.block(6,0,6,12) = Eigen::Matrix<double,6,12>::Zero(); 
        s = M.exp();
        Phi = s.block(0,0,6,6);
        Gamma = s.block(0,6,6,6);

        this->P = Phi*this->P*Phi.transpose() + Gamma*this->Q*Gamma.transpose();  //Zero order hold
        this->xhat = xhat + fx*dt; 
        this->fixP();

        this->t_prev=t;
    }

    void KFSensorFusion::update(double t, const Eigen::Matrix<double,3,1> &z)
    {
        yhat = calc_h(t,xhat);
        H = calc_H(t,xhat);

        K = P*H.transpose()*(H*P*H.transpose()+R).inverse();

        xtilde = K*(z-yhat); // z from leg odometry

        xhat = xhat + xtilde;
        P = (I-K*H)*P;

        this->fixP();
    }

    Eigen::Matrix<double,6,1> KFSensorFusion::calc_f(double t, const Eigen::Matrix<double,6,1> &x, const Eigen::Matrix<double,3,1> &u)
    {
        v = x.tail(3);  // x = [x^n v^n]'

        fx << v, u;
        return fx;
    }

    Eigen::Matrix<double,6,6> KFSensorFusion::calc_F(double t, const Eigen::Matrix<double,6,1> &x, const Eigen::Matrix<double,3,1> &u)
    {
        F <<    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        return F;
    
    } 

    Eigen::Matrix<double,3,1> KFSensorFusion::calc_h(double t, const Eigen::Matrix<double,6,1> &x)
    {
        yhat << x.tail(3);                   // estimated velocity of the base

        return yhat;

    }

    Eigen::Matrix<double,3,6> KFSensorFusion::calc_H(double t, const Eigen::Matrix<double,6,1> &x)
    {
        H << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        return H;
    }  
  
} // namespace state_estimator
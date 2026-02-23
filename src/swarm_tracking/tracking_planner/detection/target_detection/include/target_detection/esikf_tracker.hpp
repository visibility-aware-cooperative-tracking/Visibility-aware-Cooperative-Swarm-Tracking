#ifndef ESIKF_TRACKER_HPP
#define ESIKF_TRACKER_HPP

#include <Eigen/Eigen>
#include <mutex>
#include <ros/ros.h>

#define init_cov 0.1;

using namespace Eigen;


struct Meas {
  Eigen::Vector3d pos_;
  double t_stamp_;
  double meas_noise_;

  Meas(const Eigen::Vector3d& pos, 
       const double &t_stamp, 
       const double &meas_noise) {
    this->pos_ = pos;
    this->t_stamp_ = t_stamp;
    this->meas_noise_ = meas_noise;
  };

};
typedef std::vector<Meas> MeasData;


class ESIKF{
public:
    ESIKF(){};

    ~ESIKF(){};

    void setMeasData(MeasData *meas_data)
    {
        meas_data_ = meas_data;
    }

    void setDim(const int &dim_x = 6, const int &dim_w = 3, const int &dim_z = 3)
    {
        x_.resize(dim_x);
        x_.setZero();
        w_.resize(dim_w);
        w_.setZero();
        delta_x_.resize(dim_x);
        delta_x_.setZero();
        F_x_.resize(dim_x, dim_x);
        F_x_.setIdentity();
        F_w_.resize(dim_x, dim_w);
        F_w_.setZero();
        Q_.resize(dim_w, dim_w);
        Q_.setIdentity() * init_cov;
        R_.resize(dim_z, dim_z);
        R_.setIdentity() * init_cov;
        P_.resize(dim_x, dim_x);
        P_.setIdentity() * init_cov;
        K_.resize(dim_x, dim_z);
        K_.setZero();
        H_.resize(dim_z, dim_x);
        H_.setIdentity();
        z_.resize(dim_z);
        z_.setZero();
        last_predict_time_ = 0.0;
        last_update_time_ = 0.0;
        last_teammate_update_time_ = 0.0;
    }

    void init(const Vector3d &meas_state, const double &init_time){
        x_.head<3>() = meas_state;
        x_.tail<3>().setZero();
        w_ = Vector3d(0.012,0.012,0.012); 
        Q_ = w_.asDiagonal();
        F_x_ = Matrix<double,6,6>::Identity();
        F_x_.block<3,3>(0,3) = Matrix3d::Identity() * 0.001;
        F_w_.block<3,3>(0,0) = Matrix3d::Zero();
        F_w_.block<3,3>(3,0) = Matrix3d::Identity() * 0.001;
        H_ = Matrix<double,3,6>::Zero();
        H_.block<3,3>(0,0) = Matrix3d::Identity();
        last_predict_time_ = init_time - 0.001;
        last_update_time_ = init_time - 0.001;
        last_teammate_update_time_ = init_time - 0.001;
    }

    void reset( const Vector3d &meas, const double &reset_time) {
        z_ = meas;
        x_.setZero();
        x_.head<3>() = meas;
        P_.setZero();
        last_update_time_ = reset_time;
    }

    void predict(const double &predict_time){

        double dt = predict_time - last_predict_time_;
        delta_x_.block<3,1>(0,0) = x_.block<3,1>(3,0) * dt;
        delta_x_.block<3,1>(3,0) = Vector3d::Zero();
        F_x_.block<3,3>(0,3) = Matrix3d::Identity() * dt;
        F_w_.block<3,3>(3,0) = Matrix3d::Identity() * dt;
        //State prediction
        x_ += delta_x_;
        //Covariance prediction
        P_ = F_x_ * P_ * F_x_.transpose() + F_w_ * Q_ * F_w_.transpose();
        last_predict_time_ = predict_time;
    }

    void update(const Vector3d &meas, const double &meas_noise, const int &iter_num = 1) {

        z_ = meas;
        R_.setIdentity();
        R_ *= meas_noise;
        R_(2,2) = meas_noise * 5; 
        MatrixXd x_predicted = x_;

        for (int i = 0; i < iter_num; ++i) {
            MatrixXd residual = z_ - H_ * x_;
            //Kalman Gain
            K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
            //state update
            MatrixXd vec = x_ - x_predicted;
            delta_x_ = K_ * residual - vec + K_ * H_ * vec;
            x_ += delta_x_;

            //converge check
            if (delta_x_.norm() < 0.0001 || i == iter_num - 1){
                // Covariance update
                P_ = P_ - K_ * H_ * P_;
                break;
            }
        }
    }

    void updateMeas(const double &update_time, const int &iter_num = 1)
    {
        if(meas_data_->empty()) return;
        last_update_time_ = update_time;
        for(auto &meas : *meas_data_)
        {
            set_H_vel(meas.t_stamp_ - update_time);
            update(meas.pos_, meas.meas_noise_, iter_num);
        }
        return;
    }

    MatrixXd get_state_pos(){
        return x_.head<3>();
    }
    
    MatrixXd get_state_vel(){
        return x_.tail<3>();
    }

    void set_H_vel(const double &H_vel_value){
        H_(0,3) = H_(1,4) = H_(2,5) = H_vel_value;
    }

    double last_predict_time_, last_update_time_, last_teammate_update_time_;
private:
    VectorXd x_, w_, delta_x_;
    MatrixXd F_x_, F_w_;
    MatrixXd Q_, R_;
    MatrixXd P_, K_, H_;
    VectorXd z_;
    MeasData *meas_data_{NULL};
};


#endif

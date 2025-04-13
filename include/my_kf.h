#ifndef MY_KF_H
#define MY_KF_H

#include <iostream>
#include <vector>
#include <string>
#include "Eigen/Dense"
#include <Eigen/StdVector>
#include <random>




class KalmanFilter{ 
        Eigen::Vector2d x_; // initial state vector
        Eigen::Vector2d x_hat_; // Predicted

        Eigen::Matrix2d A_; // state transition matrix

        Eigen::Matrix2d P_; // initial state covariance matrix
        Eigen::Matrix2d P_hat_; // Predicted

        Eigen::Matrix2d Q_; // process noise covariance matrix

        Eigen::MatrixXd H_; // measurement matrix

        double R_; // measurement noise covariance
        
        Eigen::MatrixXd K_; // Kalman gain matrix

        double dt_;   // measurement time period

        double Iterations_;

        // Random number generator for measurement + noise
        std::random_device rd_;
        std::mt19937 gen_;

    public:

    

    struct Record_values{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Necessary for fixed-size Eigen members
        Eigen::Vector2d x_estimates;
        Eigen::Matrix2d P_estimates;
        Eigen::Vector2d x_predict;
        Eigen::Matrix2d P_predict;
        double measurement;

        // Constructor
        Record_values(const Eigen::Vector2d& x_est, // we use reference to avoid
                      const Eigen::Matrix2d& P_est, // unnecessary copy 
                      const Eigen::Vector2d& x_pred,
                      const Eigen::Matrix2d& P_pred,
                      const double& meas):
                            x_estimates(x_est),
                            P_estimates(P_est),
                            x_predict(x_pred),
                            P_predict(P_pred),
                            measurement(meas){}
        Record_values(): 
            x_estimates(Eigen::Vector2d::Zero()),
            P_estimates(Eigen::Matrix2d::Identity()),
            x_predict(Eigen::Vector2d::Zero()),
            P_predict(Eigen::Matrix2d::Identity()),
            measurement(0.0){}
    };  
    
  

    

    typedef std::vector<Record_values, 
                        Eigen::aligned_allocator<Record_values>>
                        Record_val_vec;   
                        
        // Initialization
        KalmanFilter(Eigen::Vector2d x_in, 
                        Eigen::Matrix2d P_in,
                        Eigen::Matrix2d Q_in, 
                        Eigen::MatrixXd H_in,
                        double dt_in,
                        Eigen::MatrixXd A_in,  
                        double R_in, 
                        double Interations ); // Constructor
        KalmanFilter();

        // Destructor
        ~KalmanFilter();

        // Copy Constructor
        KalmanFilter(const KalmanFilter &kf);
        
        void Predict();

        // Random number generator for measurement + noise
        double GenerateNoise(const double mu, const double sigma);

        // Updating using measurement from sensor
        Record_values Update(const double& z);

        friend std::ostream& operator<<(std::ostream& os, const KalmanFilter& kf);   

        // Setters
        void set_x0(const Eigen::Vector2d& x_in){ x_ = x_in;}
        
        void set_P0(const Eigen::Matrix2d& P_in){ P_ = P_in;}
        
        void set_Q(const Eigen::Matrix2d& Q_in){ Q_ = Q_in;}
        
        void set_H(const Eigen::MatrixXd& H_in){ H_ = H_in;}
        
        void set_R(const double& R_in){ R_ = R_in;}

        void set_A(const Eigen::Matrix2d& A_in){ A_ = A_in;}
        
        bool set_dt(const double& dt_in){ return ((dt_ = dt_in)&&(A_(0, 1) = dt_));}
        
        void set_iterations(const double& Iterations){ Iterations_ = Iterations;}
        
        // Getters
        double get_iterations() const { return Iterations_;}
        Eigen::Vector2d get_x() const { return x_;}
        double get_dt() const { return dt_;}

        // Plot graph of results
        static void plotResults(
                    const KalmanFilter::Record_val_vec& results, 
                    const std::vector<double>& time_steps,
                    bool plot_x = true,
                    bool plot_p = false,
                    bool plot_meas = true);
};

#endif // MY_KF_H


#include "my_kf.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

/*
double type parameter are initialized as is, 
but changed to vector (Eigen::VectorXd) for 
calculations with vectors and matrix parameters 
in the Prediction and Update fuction of the KalmanFilter class
*/



using namespace std;

KalmanFilter::KalmanFilter(Eigen::Vector2d x_in, 
                            Eigen::Matrix2d P_in,
                            Eigen::Matrix2d Q_in,  
                            Eigen::MatrixXd H_in,
                            double dt_in,
                            Eigen::MatrixXd A_in,
                            double R_in,
                            double Iterations):
                            x_(x_in), P_(P_in),
                            Q_(Q_in), R_(R_in),
                            H_(H_in), dt_(dt_in),
                            A_(A_in), Iterations_(Iterations), gen_(rd_())
{
    /* Initialization of internal Vector and Matrix variable 
    (Example instance of size and shape for input variables)
    
    Initialize the state vector 
    x = MatrixXd::Zero(2, 1);

    Initialize the state transition matrix 
    A = MatrixXd::Identity(2, 2);

    Initialize the measurement matrix
    H = MatrixXd::Zero(1, 2);
    H(0, 0) = 1.0;
    H(0, 1) = 1.0;

    Initialize the process noise covariance matrix
    Q = MatrixXd::Identity(4, 4) * 0.1;

    Initialize the measurement noise covariance matrix
    R = MatrixXd::Identity(2, 2) * 0.1;

    Initialize the covariance matrix
    P = MatrixXd::Identity(4, 4) * 1000.0;
    */
   
}
KalmanFilter::KalmanFilter():
          x_(Eigen::Vector2d::Zero(2)),
          P_(Eigen::Matrix2d::Identity(2,2)),
          Q_(Eigen::Matrix2d::Identity(2,2)),
          H_(Eigen::MatrixXd::Identity(2,2)),
          dt_(0.1),
          A_(Eigen::MatrixXd::Identity(2,2)),
          R_(10),
          Iterations_(100),
          gen_(rd_()){}
          
KalmanFilter::~KalmanFilter()
{
    // Destructor
}

KalmanFilter::KalmanFilter(const KalmanFilter &kf) :
    x_(kf.x_),
    P_(kf.P_),
    Q_(kf.Q_), 
    R_(kf.R_),
    H_(kf.H_),
    dt_(kf.dt_),
    A_(kf.A_),
    Iterations_(kf.Iterations_){}


void KalmanFilter::Predict(){
    x_hat_ = A_ * x_;
    P_hat_ = A_ * P_ * A_.transpose() + Q_; 
}

double KalmanFilter::GenerateNoise(const double mean, const double sigma)
{
    std::normal_distribution<double> distribution(mean, std::sqrt(sigma));
    return distribution(gen_);
}


KalmanFilter::Record_values KalmanFilter::Update(const double& z)
{
    Eigen::VectorXd z_vec(1);
    Eigen::VectorXd R_vec(1);
    z_vec << z;
    R_vec << R_;
    
    auto y = z_vec - (H_ * x_hat_);
    auto S = (H_ * P_hat_ * H_.transpose()) + R_vec;
    auto K = P_hat_ * H_.transpose() * S.inverse();
    x_ = x_hat_ + K * y;
    P_ = P_hat_ - K * H_ * P_hat_;
    return Record_values(x_, P_, x_hat_, P_hat_, z); // returns a variable of 
                                        // type UpdateOut{x_estimate, 
                                        // P_estimate, x_predict, P_predict, 
                                        // measurement}     
}



std::ostream& operator<<(std::ostream& os, const KalmanFilter& kf)
{
    os << "State vector x: \n" << kf.x_.transpose() << std::endl;
    os << "Error covariance P: \n" << kf.P_ << std::endl;
    os << "Process Noise Matrix Q: \n" << kf.Q_ << std::endl;
    os << "Measurement Matrix H: \n" << kf.H_ << std::endl;
    os << "Measurement Noise Covariance R: \n" << kf.R_ << std::endl;
    os << "State transition Matrix A: \n" << kf.A_ << std::endl;
    os << "Measurement time period dt: \n" << kf.dt_ << std::endl;
    os << "Iterations: \n" << kf.Iterations_ << std::endl;
    return os;
}

void KalmanFilter::plotResults(const KalmanFilter::Record_val_vec& results, 
                                                            const std::vector<double>& time_steps,
                                                            bool plot_x,
                                                            bool plot_p,
                                                            bool plot_meas){
    if(results.empty()){
        std::cerr << "Results are empty! No data to plot." << std::endl;
        return;
    }

    std::vector<double> x_estimates_1;
    std::vector<double> x_estimates_2;
    std::vector<double> p_estimates_1;
    std::vector<double> p_estimates_2;
    std::vector<double> measurement;

    for (const auto& record : results){
        x_estimates_1.push_back(record.x_estimates(0));     // first parameter
        x_estimates_2.push_back(record.x_estimates(1));     // second parameter
        p_estimates_1.push_back(record.P_estimates(0,0));     
        p_estimates_2.push_back(record.P_estimates(1,1));     
        measurement.push_back(record.measurement);
    }

    if (plot_x){
        plt::plot(time_steps, x_estimates_1, "-b",{{"label","position estimates"}});
        plt::plot(time_steps, x_estimates_2, "-r", {{"label", "velocity estimates"}});
    }

    if(plot_p){
        plt::plot(time_steps, p_estimates_1, "g", {{"label", "position variance"}});
        plt::plot(time_steps, p_estimates_2, "m", {{"label", "velocity variance"}});
    
    }

    if(plot_meas){
        plt::plot(time_steps, measurement, ":k", {{"label", "measurement"}});

    }
    plt::legend();
    plt::show();
}
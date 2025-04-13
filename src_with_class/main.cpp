#include <iostream>
#include "my_kf.h"


using namespace std;

 //Define namespace here

int main(){

    // Create class
    KalmanFilter My_kf;
    double measurement = 0;
    double noise_mean = 0;
    double noise_variance = 100;
    double noise;
    std::vector<double> time_steps;
    KalmanFilter::Record_val_vec results;

    // Initialize parameters x0, P0, Q, H, R, A, dt, Iterations    
    My_kf.set_x0((Eigen::Matrix<double, 1, 2>() << 0, 0).finished());
    My_kf.set_P0((Eigen::Matrix2d() << 5, 0, 0, 5).finished());
    My_kf.set_Q((Eigen::Matrix<double, 2, 2>() << 1, 0, 0, 3).finished());
    My_kf.set_H((Eigen::Matrix<double, 1, 2>() << 1, 0).finished());    
    My_kf.set_R(10);
    My_kf.set_dt(0.12);
    My_kf.set_iterations(1000);

    

    for(auto i = 0; i < My_kf.get_iterations(); i++)
    {
        My_kf.Predict();
        noise = My_kf.GenerateNoise(noise_mean, noise_variance);
        measurement += 0.4 + noise;
        cout << measurement << endl;
        results.emplace_back(My_kf.Update(measurement));
        time_steps.emplace_back(i * My_kf.get_dt());
    }

    KalmanFilter::plotResults(results, time_steps);
    std::vector<double> x_estimates;
    // // Extract x_estimates from results
    // for (const auto& result : results) {
    //     x_estimates.emplace_back(result.x_estimates[0]);
    // }   
    
    
    return 0;
}
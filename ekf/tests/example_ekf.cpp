#include "system_model.h"
#include "orientation_measurement_model.h"
#include "position_measurement_model.h"

#include "ekf/ekf.h"

#include <chrono>
#include <random>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <sys/time.h>

typedef double T;

// Some type shortcuts
typedef example::State<T> State;
typedef example::Control<T> Control;
typedef example::SystemModel<State,Control> SystemModel;

typedef example::PositionMeasurement<T> PositionMeasurement;
typedef example::OrientationMeasurement<T> OrientationMeasurement;
typedef example::PositionMeasurementModel<State, PositionMeasurement> PositionModel;
typedef example::OrientationMeasurementModel<State, OrientationMeasurement> OrientationModel;

cv::Point2i cv_offset(double x, double y, int image_width = 2000, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 100) + image_width / 2;
  output.y = image_height - int(y * 100) - image_height / 3;
  return output;
}

int main(int argc, char** argv)
{
    // Simulated (true) system state
    State x;
    x.setZero();

    State x_truth;
    x_truth.setZero();

    std::vector<State> viz_x;
    std::vector<State> viz_ekf;
    
    // Control input
    Control u;
    Control u_truth;
    // System
    SystemModel sys;
    
    // Measurement models
    // Set position landmarks at (-1, -1) and (10, 15)
    PositionModel pm(-1, -1, 10, 15);
    OrientationModel om;
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Extended Kalman Filter
    filters::ExtendedKalmanFilter<State> ekf;
    
    // Init filters with true system state
    ekf.init(x);
    
    // Standard-Deviation of noise added to all state vector components during state transition
    T systemNoise = 0.01;
    // Standard-Deviation of noise added to all measurement vector components in orientation measurements
    T orientationNoise = 0.001;
    // Standard-Deviation of noise added to all measurement vector components in distance measurements
    T distanceNoise = 0.025;
    
    // Simulate for 100 steps
    const size_t N = 100;
    for(auto i = 1; i <= N; i++)
    {
        // Generate some control input
        u_truth.v() = 0.5;
        u_truth.dtheta() = 0.1;
        
        // Simulate system
        x_truth = sys.f(x_truth, u_truth);

        viz_x.push_back(x_truth);

        u.v() = u_truth.v() + systemNoise*noise(generator);
        u.dtheta() = u_truth.dtheta() + systemNoise*noise(generator);

        x = sys.f(x,u);
                
        // Predict state for current time-step using the filters
        auto x_ekf = ekf.predict(sys, u);
        
        // Orientation measurement
        {
            // We can measure the orientation every 5th step
            OrientationMeasurement orientation = om.h(x);
            
            // Measurement is affected by noise as well
            orientation.theta() += orientationNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(om, orientation);
        }
        
        // Position measurement
        {
            // We can measure the position every 10th step
            PositionMeasurement position = pm.h(x);
            
            // Measurement is affected by noise as well
            position.d1() += distanceNoise * noise(generator);
            position.d2() += distanceNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(pm, position);
            viz_ekf.push_back(x_ekf);
        }
        
        // Print to stdout as csv format
        // std::cout   << x.x() << "," << x.y() << "," << x.theta() << ","
                    // << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.theta() << std::endl;

        // Visualization
        cv::Mat bg(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
        for (size_t i = 0; i < viz_x.size(); i++) {
            cv::circle(bg,
                        cv_offset(viz_x.at(i).x(), viz_x.at(i).y(), bg.cols, bg.rows),
                        20,
                        cv::Scalar(0, 255, 0),
                        -1);
        }
        cv::circle(bg,
            cv_offset(viz_x.back().x(), viz_x.back().y(), bg.cols, bg.rows),
            30,
            cv::Scalar(0, 0, 255),
            -1);
        for (size_t i = 0; i < viz_ekf.size(); i++) {
            cv::circle(bg,
                        cv_offset(viz_ekf.at(i).x(), viz_ekf.at(i).y(), bg.cols, bg.rows),
                        20,
                        cv::Scalar(255, 0, 0),
                        -1);
        cv::circle(bg,
            cv_offset(viz_ekf.back().x(), viz_ekf.back().y(), bg.cols, bg.rows),
            30,
            cv::Scalar(0, 255, 255),
            -1);
        }

        decltype(bg) outImg;
        cv::resize(bg, outImg, cv::Size(), 0.2, 0.2);
        cv::imshow("ekf", outImg);
        cv::waitKey(1);
    }
    
    return 0;
}

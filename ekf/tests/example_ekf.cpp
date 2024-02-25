#include "system_model.h"
#include "orientation_measurement_model.h"
#include "position_measurement_model.h"

#include "ekf/ekf.h"

#include <chrono>
#include <random>
#include <vector>
#include <sys/time.h>
#include <fstream>
#include <filesystem>

typedef double T;

// Some type shortcuts
typedef example::State<T> State;
typedef example::Control<T> Control;
typedef example::SystemModel<State,Control> SystemModel;

typedef example::PositionMeasurement<T> PositionMeasurement;
typedef example::OrientationMeasurement<T> OrientationMeasurement;
typedef example::PositionMeasurementModel<State, PositionMeasurement> PositionModel;
typedef example::OrientationMeasurementModel<State, OrientationMeasurement> OrientationModel;

void saveData(const std::vector<State>& viz_x, const std::vector<State>& viz_ekf, const char* output_file) {
    std::ofstream f{output_file};
    f << "x_truth_x,x_truth_y,x_truth_theta,x_ekf_x,x_ekf_y,x_ekf_theta\n";  // Header

    for (size_t i = 0; i < viz_x.size(); i++) {
        f << viz_x[i].x() << "," << viz_x[i].y() << "," << viz_x[i].theta() << ","
          << viz_ekf[i].x() << "," << viz_ekf[i].y() << "," << viz_ekf[i].theta() << std::endl;
    }

    f.close();
}

int main(int argc, char** argv) {
    State x;
    x.setZero();

    State x_truth;
    x_truth.setZero();

    std::vector<State> viz_x;
    std::vector<State> viz_ekf;

    Control u;
    Control u_truth;
    SystemModel sys;

    PositionModel pm(-1, -1, 10, 15);
    OrientationModel om;

    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<T> noise(0, 1);

    filters::ExtendedKalmanFilter<State> ekf;
    ekf.init(x);

    T systemNoise = 0.1;
    T orientationNoise = 0.001;
    T distanceNoise = 0.025;

    const size_t N = 100;

    const char* output_file = "../ekf/tests/data.csv";
    if (std::filesystem::exists(output_file)) {
        std::filesystem::remove(output_file);
    }

    for (auto i = 1; i <= N; i++) {
        u_truth.v() = 0.8 - std::sin(T(2) * T(M_PI) / T(N));
        u_truth.dtheta() = -std::sin(T(2) * T(M_PI) / T(N)) * (1 - 2 * (i > 50));

        x_truth = sys.f(x_truth, u_truth);
        viz_x.push_back(x_truth);

        u.v() = u_truth.v() + systemNoise * noise(generator);
        u.dtheta() = u_truth.dtheta() + systemNoise * noise(generator);

        x = sys.f(x, u);

        auto x_ekf = ekf.predict(sys, u);

        OrientationMeasurement orientation = om.h(x);
        orientation.theta() += orientationNoise * noise(generator);
        x_ekf = ekf.update(om, orientation);

        PositionMeasurement position = pm.h(x);
        position.d1() += distanceNoise * noise(generator);
        position.d2() += distanceNoise * noise(generator);
        x_ekf = ekf.update(pm, position);
        viz_ekf.push_back(x_ekf);
    }

    saveData(viz_x, viz_ekf, output_file);

    return 0;
}

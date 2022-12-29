#pragma once

#include "ekf/ekf.h"


namespace example
{

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement : public filters::Vector<T, 1>
{
public:
    //! Orientation
    static constexpr size_t THETA = 0;
    
    T theta()  const { return (*this)[ THETA ]; }
    T& theta() { return (*this)[ THETA ]; }
};

/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<class State, class Measurement>
class OrientationMeasurementModel : public filters::LinearizedMeasurementModel<State, Measurement>
{
public:
    //! State type shortcut definition
    typedef example::State<typename State::Scalar> S;
    
    //! Measurement type shortcut definition
    typedef  example::OrientationMeasurement<typename Measurement::Scalar> M;
    
    OrientationMeasurementModel()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        // Measurement is given by the actual robot orientation
        measurement.theta() = x.theta();
        
        return measurement;
    }
};

} // namespace Robot

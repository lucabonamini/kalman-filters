#pragma once

#include "types.h"

#include <type_traits>

namespace filters {

template<class StateType, class MeasurementType>
class MeasurementModel : public CovarianceBase<MeasurementType>  {
  static_assert(std::is_fundamental<typename StateType::Scalar>::value, "State vector must be of fundamental type");
  static_assert(std::is_fundamental<typename MeasurementType::Scalar>::value, "Measurement vector must be of fundamental type");
  static_assert(StateType::RowsAtCompileTime > 0, "State vector must contain at least 1 element");
  static_assert(MeasurementType::RowsAtCompileTime >= 0, "Measurement vector must contain at least 0 element");
  static_assert(std::is_same<typename StateType::Scalar, typename MeasurementType::Scalar>::value, "State and Measurement types must be the same");
  public:
  using State = StateType;
  using Measurement = MeasurementType;
  virtual Measurement h(const State& x) const = 0;
  protected:
  virtual ~MeasurementModel() = default;
};

template<class StateType>
class ExtendedKalmanFilter;

template<class StateType, class MeasurementType>
class LinearizedMeasurementModel : public MeasurementModel<StateType, MeasurementType> {
  friend class ExtendedKalmanFilter<StateType>;
  public:
  using Base = MeasurementModel<StateType, MeasurementType>;
  using typename Base::State;
  using typename Base::Measurement;
  protected:
  LinearizedMeasurementModel() {
    H.setIdentity();
    V.setIdentity();
  }
  virtual ~LinearizedMeasurementModel() = default;
  virtual void updateJacobians(const State& x) {}
  Jacobian<Measurement, State> H;
  Jacobian<Measurement, Measurement> V;
};
} // namespace filters
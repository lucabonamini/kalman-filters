#pragma once

#include "filter_base.h"

namespace filters {
template<class StateType>
class ExtendedKalmanFilter : public FilterBase<StateType>, public CovarianceBase<StateType> {
  public:
  using Base = FilterBase<StateType>;
  using CovBase = CovarianceBase<StateType>;

  using Base::x;
  using CovBase::P;

  using typename Base::State;

  template<class Control>
  using SystemModelType = LinearizedSystemModel<State, Control>;

  template<class Measurement>
  using MeasurementModelType = LinearizedMeasurementModel<State, Measurement>;

  template<class Measurement>
  using KalmanGain = KalmanGain<State, Measurement>;

  template<class Control>
  const State& predict(SystemModelType<Control>& s, const Control& u) {
    s.updateJacobians(x, u);
    x = s.f(x, u);
    P = (s.F * P * s.F.transpose()) + (s.W * s.getCovariance() * s.W.transpose());
    return this->getState();
  }
  template<class Measurement>
  const State& update(MeasurementModelType<Measurement>& m, const Measurement& z) {
    m.updateJacobians(x);
    Covariance<Measurement> S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
    KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();
    x += K * (z - m.h(x));
    P -= K * m.H * P;
    return this->getState();
  }
};
} // namespace filters

#pragma once

#include "types.h"

#include <type_traits>

namespace filters {
template<class StateType, class ControlType>
class SystemModel : public CovarianceBase<StateType> {
  static_assert(std::is_fundamental<typename StateType::Scalar>::value, "State vector must be of fundamental type");
  static_assert(std::is_fundamental<typename ControlType::Scalar>::value, "Control vector must be of fundamental type");
  static_assert(StateType::RowsAtCompileTime > 0, "State vector must contain at least 1 element");
  static_assert(ControlType::RowsAtCompileTime >= 0, "Control vector must contain at least 0 element");
  static_assert(std::is_same<typename StateType::Scalar, typename ControlType::Scalar>::value, "State and Control types must be the same");
  public:
  using State = StateType;
  using Control = ControlType;
  virtual ~SystemModel() = default;
  virtual State f(const State& x, const Control& u) const = 0;
};

template<class StateType>
class ExtendedKalmanFilter;

template<class StateType, class ControlType>
class LinearizedSystemModel : public SystemModel<StateType, ControlType> {
  friend class ExtendedKalmanFilter<StateType>;
  public:
  using State = typename SystemModel<StateType, ControlType>::State;
  using Control = typename SystemModel<StateType, ControlType>::Control;
  protected:
  LinearizedSystemModel() {
    F.setIdentity();
    W.setIdentity();
  }
  virtual ~LinearizedSystemModel() = default;
  virtual void updateJacobians(const State& x, const Control& u) {}
  Jacobian<State, State> F;
  Jacobian<State, State> W;
};
} // namespace filters
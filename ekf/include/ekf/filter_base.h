#pragma once

#include "system_model.h"
#include "measurement_model.h"

namespace filters {
template<class StateType>
class FilterBase {
  public:
  using State = StateType;
  const State& getState() const { return x; }
  void init(const State& initial_state) { x = initial_state; }
  protected:
  State x;
};
} // namespace filters

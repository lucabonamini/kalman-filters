#pragma once

#include <eigen3/Eigen/Dense>

namespace filters {
  template<typename T, size_t ROWS, size_t COLUMNS>
  using Matrix = Eigen::Matrix<T, ROWS, COLUMNS>;

  template<typename T, size_t ROWS>
  class Vector : public Matrix<T, ROWS, 1> {
    public:
    Vector() : Matrix<T, ROWS, 1>() {}
    template<typename OtherDerived>
    Vector(const Eigen::MatrixBase<OtherDerived>& other) : Matrix<T, ROWS, 1>(other) {}
    template<typename OtherDerived>
    Vector& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
      this->Matrix<T, ROWS, 1>::operator=(other);
      return *this;
    }
  };

  template<class M, class N>
  using Jacobian = Matrix<typename M::Scalar, M::RowsAtCompileTime, N::RowsAtCompileTime>;

  template<typename M>
  using Covariance = Matrix<typename M::Scalar, M::RowsAtCompileTime, M::RowsAtCompileTime>;

  template<class M, class N>
  using KalmanGain = Matrix<typename M::Scalar, M::RowsAtCompileTime, N::RowsAtCompileTime>;

  template<class Type>
  class CovarianceBase {
    public:
    const Covariance<Type>& getCovariance() const { return P; }
    void setCovariance(const Covariance<Type>& covariance) { P = covariance; }
    protected:
    CovarianceBase() { P.setIdentity(); }
    Covariance<Type> P;
  }; 
} // namespace filters
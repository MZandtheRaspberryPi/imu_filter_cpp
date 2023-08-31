#pragma once

#include <Eigen/Dense>
#include <iostream>

template <size_t N, size_t M, typename T>
class IMUNonLinearSystemModel {
 public:
  typedef Eigen::Matrix<T, N, 1> StateMatrix;
  typedef Eigen::Matrix<T, N, N> CovarianceMatrix;
  typedef Eigen::Matrix<T, M, 1> MeasurementMatrix;
  typedef Eigen::Matrix<T, N, N> AMatrix;
  typedef Eigen::Matrix<T, M, N> CMatrix;

  IMUNonLinearSystemModel();
  virtual ~IMUNonLinearSystemModel() = 0;
  virtual AMatrix get_a_matrix() = 0;
  virtual CMatrix get_c_matrix() = 0;
  virtual StateMatrix transition_state() = 0;
  virtual MeasurementMatrix get_measurement() = 0;
  virtual MeasurementMatrix get_expected_measurment() = 0;
};

class SaitoIMUSystemModel : public IMUNonLinearSystemModel<3, 4, double> {
 public:
  SaitoIMUSystemModel();
  ~SaitoIMUSystemModel();

  AMatrix get_a_matrix();

  CMatrix get_c_matrix();

  StateMatrix transition_state();

  MeasurementMatrix get_measurement();

  MeasurementMatrix get_expected_measurment();
};

template <size_t N, size_t M, typename T>
class NonLinearFilter {
 public:
  virtual void predict() = 0;
  virtual void update() = 0;
  virtual void rotate_sensor_to_base_frame() = 0;

  void predict_and_update();

 private:
  IMUNonLinearSystemModel<N, M, T> system_model_;
};
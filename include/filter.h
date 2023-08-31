#pragma once

#include <Eigen/Dense>
#include <iostream>

/*
static const size_t GENERIC_STATE_MATRIX_ROWS = 3;
static const size_t GENERIC_MEASUREMENT_MATRIX_ROWS = 4;
typedef Eigen::Matrix<double, GENERIC_STATE_MATRIX_ROWS, 1> StateMatrix;
typedef Eigen::Matrix<double, GENERIC_STATE_MATRIX_ROWS,
                      GENERIC_STATE_MATRIX_ROWS>
    CovarianceMatrix;
typedef Eigen::Matrix<double, GENERIC_MEASUREMENT_MATRIX_ROWS, 1>
    MeasurementMatrix;

typedef Eigen::Matrix<double, GENERIC_STATE_MATRIX_ROWS,
                      GENERIC_STATE_MATRIX_ROWS>
    AMatrix;
typedef Eigen::Matrix<double, GENERIC_MEASUREMENT_MATRIX_ROWS,
                      GENERIC_STATE_MATRIX_ROWS>
    CMatrix;
*/

template <size_t N, size_t M, typename T>
class IMUNonLinearSystemModel {
 public:
  typedef Eigen::Matrix<T, N, 1> StateMatrix;
  typedef Eigen::Matrix<T, N, N> CovarianceMatrix;
  typedef Eigen::Matrix<T, M, 1> MeasurementMatrix;
  typedef Eigen::Matrix<T, N, N> AMatrix;
  typedef Eigen::Matrix<T, M, N> CMatrix;

  virtual ~IMUNonLinearSystemModel(){};
  virtual AMatrix get_a_matrix() = 0;
  virtual CMatrix get_c_matrix() = 0;
  virtual StateMatrix transition_state() = 0;
  virtual MeasurementMatrix get_measurement() = 0;
  virtual MeasurementMatrix get_expected_measurment() = 0;
  /*
  virtual IMUNonLinearSystemModel::AMatrix get_a_matrix() = 0;
  virtual IMUNonLinearSystemModel::CMatrix get_c_matrix() = 0;
  virtual IMUNonLinearSystemModel::StateMatrix transition_state() = 0;
  virtual IMUNonLinearSystemModel::MeasurementMatrix get_measurement() = 0;
  virtual IMUNonLinearSystemModel::MeasurementMatrix */
};

class SaitoIMUSystemModel : public IMUNonLinearSystemModel<3, 4, double> {
 public:
  // ~SaitoIMUSystemModel();
  // AMatrix get_a_matrix() override;
  // CMatrix get_c_matrix() override;
  // StateMatrix transition_state() override;
  // MeasurementMatrix get_measurement() override;
  // MeasurementMatrix get_expected_measurment() override;
  ~SaitoIMUSystemModel() {}

  AMatrix get_a_matrix() {
    AMatrix a_matrix;
    return a_matrix;
  }

  CMatrix get_c_matrix() {
    CMatrix c_matrix;
    return c_matrix;
  }

  StateMatrix transition_state() {
    StateMatrix state_matrix;
    return state_matrix;
  }

  MeasurementMatrix get_measurement() {
    MeasurementMatrix measurement_matrix;
    return measurement_matrix;
  }

  MeasurementMatrix get_expected_measurment() {
    MeasurementMatrix measurement_matrix;
    return measurement_matrix;
  }
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

// template class IMUNonLinearSystemModel<3ul, 4ul, double>
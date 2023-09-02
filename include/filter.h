#pragma once

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <memory>

typedef double m_t;
typedef Eigen::Matrix<m_t, 3, 3> RotationMatrix;

#define PI 3.14159265
#define M_ERR_VAL 0.0
#define GRAVITY 9.81

m_t do_cos(const m_t& angle);
m_t do_sin(const m_t& angle);
m_t do_tan(const m_t& angle);
m_t do_arctan(const m_t& y, const m_t& x);

template <size_t N, size_t M, typename T>
class IMUNonLinearSystemModel {
 public:
  typedef Eigen::Matrix<T, N, 1> StateMatrix;
  typedef Eigen::Matrix<T, N, N> CovarianceMatrix;
  typedef Eigen::Matrix<T, M, 1> MeasurementMatrix;
  typedef Eigen::Matrix<T, N, N> AMatrix;
  typedef Eigen::Matrix<T, M, N> CMatrix;
  typedef Eigen::Matrix<T, 3, 1> SensorDataMatrix;

  IMUNonLinearSystemModel();
  virtual ~IMUNonLinearSystemModel() = 0;
  virtual AMatrix get_a_matrix(const StateMatrix& state,
                               const SensorDataMatrix& angular_rotation,
                               const m_t& delta_t) = 0;

  virtual CMatrix get_c_matrix(const StateMatrix& state) = 0;

  virtual StateMatrix transition_state(const StateMatrix& state,
                                       const SensorDataMatrix& angular_rotation,
                                       const m_t& delta_t) = 0;

  virtual MeasurementMatrix get_measurement(
      const StateMatrix& state, const SensorDataMatrix& accelerometer,
      const SensorDataMatrix& magnetometer) = 0;

  virtual MeasurementMatrix get_expected_measurment(
      const StateMatrix& state) = 0;
};

class SaitoIMUSystemModel : public IMUNonLinearSystemModel<3, 4, m_t> {
 public:
  SaitoIMUSystemModel();
  ~SaitoIMUSystemModel();

  AMatrix get_a_matrix(
      const SaitoIMUSystemModel::StateMatrix& state,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const m_t& delta_t) override;

  CMatrix get_c_matrix(const SaitoIMUSystemModel::StateMatrix& state) override;

  StateMatrix transition_state(
      const SaitoIMUSystemModel::StateMatrix& state,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const m_t& delta_t) override;

  MeasurementMatrix get_measurement(
      const SaitoIMUSystemModel::StateMatrix& state,
      const SaitoIMUSystemModel::SensorDataMatrix& accelerometer,
      const SaitoIMUSystemModel::SensorDataMatrix& magnetometer) override;

  MeasurementMatrix get_expected_measurment(
      const SaitoIMUSystemModel::StateMatrix& state) override;
};

template <typename T>
class FilterNonLinearModel {
 public:
  FilterNonLinearModel();
  ~FilterNonLinearModel();
  virtual void predict() = 0;
  virtual void update() = 0;
  virtual void rotate_sensor_to_base_frame() = 0;

  void predict_and_update();

 private:
  std::shared_ptr<T> system_model_ptr_;
};

class EKFSaitoModel : public FilterNonLinearModel<SaitoIMUSystemModel> {};
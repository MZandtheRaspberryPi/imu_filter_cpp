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
  typedef Eigen::Matrix<T, N, N> StateCovarianceMatrix;
  typedef Eigen::Matrix<T, M, 1> MeasurementMatrix;
  typedef Eigen::Matrix<T, N, N> AMatrix;
  typedef Eigen::Matrix<T, M, N> CMatrix;
  typedef Eigen::Matrix<T, 3, 1> SensorDataMatrix;
  typedef Eigen::Matrix<T, M, M> MeasurementCovarianceMatrix;

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
  struct EstimateAndCovariance {
    typename T::StateMatrix state_estimate;
    typename T::StateCovarianceMatrix covariance;
  };

  FilterNonLinearModel();
  virtual ~FilterNonLinearModel();
  virtual EstimateAndCovariance predict(
      const EstimateAndCovariance& prior_estimate_and_cov,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const m_t& delta_t) = 0;
  virtual EstimateAndCovariance update() = 0;
  virtual typename T::SensorDataMatrix rotate_sensor_to_base_frame(
      typename T::SensorDataMatrix sensor_data) = 0;

  EstimateAndCovariance predict_and_update();

 protected:
  std::unique_ptr<T> system_model_ptr_;
  SaitoIMUSystemModel::StateCovarianceMatrix q_;
  SaitoIMUSystemModel::MeasurementCovarianceMatrix r_;
};

class EKFSaitoModel : public FilterNonLinearModel<SaitoIMUSystemModel> {
 public:
  EKFSaitoModel(const SaitoIMUSystemModel::StateCovarianceMatrix& Q,
                const SaitoIMUSystemModel::MeasurementCovarianceMatrix& R);
  ~EKFSaitoModel();
  EKFSaitoModel::EstimateAndCovariance predict(
      const EKFSaitoModel::EstimateAndCovariance& prior_estimate_and_cov,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const m_t& delta_t);

  EKFSaitoModel::EstimateAndCovariance update(
      const EKFSaitoModel::EstimateAndCovariance& prior_estimate_and_cov,
      const SaitoIMUSystemModel::SensorDataMatrix& accelerometer,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const SaitoIMUSystemModel::SensorDataMatrix& magnetometer,
      const m_t& delta_t);
};
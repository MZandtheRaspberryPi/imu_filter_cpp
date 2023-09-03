#pragma once

#include "filter_base.h"

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

class EKFSaitoModel : public FilterNonLinearModel<SaitoIMUSystemModel> {
 public:
  EKFSaitoModel(const RotationMatrix& sensor_to_base,
                const SaitoIMUSystemModel::StateCovarianceMatrix& Q,
                const SaitoIMUSystemModel::MeasurementCovarianceMatrix& R);
  ~EKFSaitoModel();
  EKFSaitoModel::EstimateAndCovariance predict(
      const EKFSaitoModel::EstimateAndCovariance& prior_estimate_and_cov,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const m_t& delta_t) override;

  EKFSaitoModel::EstimateAndCovariance update(
      const EKFSaitoModel::EstimateAndCovariance& prior_estimate_and_cov,
      const SaitoIMUSystemModel::SensorDataMatrix& accelerometer,
      const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
      const SaitoIMUSystemModel::SensorDataMatrix& magnetometer,
      const m_t& delta_t) override;
};
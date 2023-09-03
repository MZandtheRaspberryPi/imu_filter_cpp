#include "filter_saito_ekf.h"

int main() {
  SaitoIMUSystemModel::StateCovarianceMatrix Q;
  Q.setIdentity();

  SaitoIMUSystemModel::MeasurementCovarianceMatrix R;
  R.setIdentity();
  R *= 0.1;

  EKFSaitoModel::EstimateAndCovariance estimate_and_cov;
  estimate_and_cov.state_estimate.setZero();
  estimate_and_cov.covariance.setIdentity();

  RotationMatrix sensor_to_base;
  sensor_to_base.setIdentity();

  EKFSaitoModel ekf_saito(sensor_to_base, Q, R);
}

#include "filter_saito_ekf.h"

SaitoIMUSystemModel::SaitoIMUSystemModel() {}
SaitoIMUSystemModel::~SaitoIMUSystemModel() {}

SaitoIMUSystemModel::AMatrix SaitoIMUSystemModel::get_a_matrix(
    const SaitoIMUSystemModel::StateMatrix& state,
    const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
    const m_t& delta_t) {
  SaitoIMUSystemModel::AMatrix a_matrix;

  const m_t& phi = state(0, 0);
  const m_t& theta = state(1, 0);
  const m_t& psi = state(2, 0);

  const m_t& omega_x = angular_rotation(0, 0);
  const m_t& omega_y = angular_rotation(1, 0);
  const m_t& omega_z = angular_rotation(2, 0);

  const m_t cos_phi = do_cos(phi);
  const m_t sin_phi = do_sin(phi);
  const m_t tan_theta = do_tan(phi);
  const m_t cos_theta = do_cos(theta);
  const m_t sin_theta = do_sin(theta);

  a_matrix(0, 0) = delta_t * omega_y * cos_phi * tan_theta -
                   delta_t * omega_z * sin_phi * tan_theta + 1;
  a_matrix(0, 1) = delta_t * omega_y * sin_phi * (pow(tan_theta, 2) + 1) +
                   delta_t * omega_z * cos_phi * (pow(tan_theta, 2) + 1);
  a_matrix(0, 2) = 0;
  a_matrix(1, 0) = -delta_t * omega_y * sin_phi - delta_t * omega_z * cos_phi;
  a_matrix(1, 1) = 1;
  a_matrix(1, 2) = 0;
  if (cos_theta != 0) {
    a_matrix(2, 0) = delta_t * omega_y * cos_phi / cos_theta -
                     delta_t * omega_z * sin_phi / cos_theta;
  } else {
    a_matrix(2, 0) = M_ERR_VAL;
  }

  if (cos_theta != 0) {
    a_matrix(2, 1) =
        delta_t * omega_y * sin_phi * sin_theta / (pow(cos_theta, 2)) +
        delta_t * omega_z * sin_theta * cos_phi / (pow(cos_theta, 2));
  } else {
    a_matrix(2, 1) = M_ERR_VAL;
  }

  a_matrix(2, 2) = 1;

  return a_matrix;
}

SaitoIMUSystemModel::CMatrix SaitoIMUSystemModel::get_c_matrix(
    const SaitoIMUSystemModel::StateMatrix& state) {
  SaitoIMUSystemModel::CMatrix c_matrix;

  const m_t& phi = state(0, 0);
  const m_t& theta = state(1, 0);
  const m_t& psi = state(2, 0);

  const m_t cos_theta = do_cos(theta);
  const m_t sin_theta = do_sin(theta);
  const m_t cos_phi = do_cos(phi);
  const m_t sin_phi = do_sin(phi);

  c_matrix(0, 0) = 0;
  c_matrix(0, 1) = 0;
  c_matrix(0, 2) = 1;
  c_matrix(1, 0) = 0;
  c_matrix(1, 1) = -GRAVITY * cos_theta;
  c_matrix(1, 2) = 0;
  c_matrix(2, 0) = GRAVITY * cos_phi * cos_theta;
  c_matrix(2, 1) = -GRAVITY * sin_phi * sin_theta;
  c_matrix(2, 2) = 0;
  c_matrix(3, 0) = -GRAVITY * sin_phi * cos_theta;
  c_matrix(3, 1) = -GRAVITY * sin_theta * cos_phi;
  c_matrix(3, 2) = 0;

  return c_matrix;
}

SaitoIMUSystemModel::StateMatrix SaitoIMUSystemModel::transition_state(
    const SaitoIMUSystemModel::StateMatrix& state,
    const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
    const m_t& delta_t) {
  SaitoIMUSystemModel::StateMatrix new_state;
  const m_t& phi = state(0, 0);
  const m_t& theta = state(1, 0);
  const m_t& psi = state(2, 0);

  const m_t& omega_x = angular_rotation(0, 0);
  const m_t& omega_y = angular_rotation(1, 0);
  const m_t& omega_z = angular_rotation(2, 0);

  const m_t cos_phi = do_cos(phi);
  const m_t sin_phi = do_sin(phi);
  const m_t sin_psi = do_sin(psi);
  const m_t tan_theta = do_tan(phi);
  const m_t cos_theta = do_cos(theta);
  const m_t sin_theta = do_sin(theta);


  new_state(0, 0) = phi + delta_t * omega_x +
                    delta_t * (sin_phi * tan_theta * omega_y) +
                    cos_phi * tan_theta * omega_z * delta_t;
  new_state(1, 0) =
      theta + delta_t * (cos_phi * omega_y) - delta_t * (sin_psi * omega_z);
  new_state(2, 0) = psi + delta_t * (sin_phi * 1 / cos_theta * omega_y) +
                    delta_t * (cos_phi * 1 / cos_theta * omega_z);

  return new_state;
}

SaitoIMUSystemModel::MeasurementMatrix SaitoIMUSystemModel::get_measurement(
    const SaitoIMUSystemModel::StateMatrix& state,
    const SaitoIMUSystemModel::SensorDataMatrix& accelerometer,
    const SaitoIMUSystemModel::SensorDataMatrix& magnetometer) {
  SaitoIMUSystemModel::MeasurementMatrix measurement;

  const m_t& phi = state(0, 0);
  const m_t& theta = state(1, 0);
  const m_t& psi = state(2, 0);

  const m_t& a_x = accelerometer(0, 0);
  const m_t& a_y = accelerometer(1, 0);
  const m_t& a_z = accelerometer(2, 0);

  const m_t& m_b_x = magnetometer(0, 0);
  const m_t& m_b_y = magnetometer(1, 0);
  const m_t& m_b_z = magnetometer(2, 0);

  RotationMatrix rotation_matrix;

  m_t calculated_phi = do_arctan(a_y, a_z);
  m_t calculated_theta = do_arctan(-a_x, sqrt(pow(a_y, 2) + pow(a_z, 2)));

  /*
  m_t cos_theta = do_cos(theta);
  m_t sin_theta = do_sin(theta);
  m_t sin_phi = do_sin(phi);
  m_t cos_phi = do_cos(phi);
  */
  m_t cos_theta = do_cos(calculated_theta);
  m_t sin_theta = do_sin(calculated_theta);
  m_t sin_phi = do_sin(calculated_phi);
  m_t cos_phi = do_cos(calculated_phi);


  rotation_matrix(0, 0) = cos_theta;
  rotation_matrix(0, 1) = sin_phi * sin_theta;
  rotation_matrix(0, 2) = cos_phi * sin_theta;

  rotation_matrix(1, 0) = 0;
  rotation_matrix(1, 1) = cos_phi;
  rotation_matrix(1, 2) = -sin_phi;

  rotation_matrix(2, 0) = -sin_theta;
  rotation_matrix(2, 1) = sin_phi * cos_theta;
  rotation_matrix(2, 2) = cos_phi * cos_theta;

  SaitoIMUSystemModel::SensorDataMatrix rotated_magnetometer =
      rotation_matrix * magnetometer;

  const m_t& rotated_m_y = rotated_magnetometer(1, 0);
  const m_t& rotated_m_x = rotated_magnetometer(0, 0);

  const m_t new_psi = do_arctan(rotated_m_y, rotated_m_x);

  measurement(0, 0) = new_psi;
  measurement(Eigen::seq(1, 3), 0) = accelerometer(Eigen::seq(0, 2), 0);

  return measurement;
}

SaitoIMUSystemModel::MeasurementMatrix
SaitoIMUSystemModel::get_expected_measurment(
    const SaitoIMUSystemModel::StateMatrix& state) {
  SaitoIMUSystemModel::MeasurementMatrix expected_measurement;
  const m_t& phi = state(0, 0);
  const m_t& theta = state(1, 0);
  const m_t& psi = state(2, 0);

  const m_t sin_theta = do_sin(theta);
  const m_t cos_theta = do_cos(theta);
  const m_t sin_phi = do_sin(phi);
  const m_t cos_phi = do_cos(phi);

  // we expect just gravity acting as acceleration
  // and hence we rotate gravity to body frame below
  expected_measurement(1, 0) = GRAVITY * (-1 * sin_theta);
  expected_measurement(2, 0) = GRAVITY * sin_phi * cos_theta;
  expected_measurement(3, 0) = GRAVITY * cos_phi * cos_theta;
  // and heading is a part of our measurement
  expected_measurement(0, 0) = psi;
  return expected_measurement;
}

/*
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
*/

EKFSaitoModel::EKFSaitoModel(

    const RotationMatrix& sensor_to_base,
    const SaitoIMUSystemModel::StateCovarianceMatrix& Q,
    const SaitoIMUSystemModel::MeasurementCovarianceMatrix& R)
    : FilterNonLinearModel<SaitoIMUSystemModel>(sensor_to_base) {
  system_model_ptr_ = std::make_unique<SaitoIMUSystemModel>();
  q_ = Q;
  r_ = R;
}

EKFSaitoModel::~EKFSaitoModel() {}

EKFSaitoModel::EstimateAndCovariance EKFSaitoModel::predict(
    const EKFSaitoModel::EstimateAndCovariance& prior_estimate_and_cov,
    const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
    const m_t& delta_t) {

  SaitoIMUSystemModel::SensorDataMatrix transformed_angular_rotation = rotate_sensor_to_base_frame(angular_rotation);
  SaitoIMUSystemModel::StateMatrix mu_t_given_t_minus_one;
  mu_t_given_t_minus_one.setZero();
  mu_t_given_t_minus_one =
      system_model_ptr_->transition_state(prior_estimate_and_cov.state_estimate,
                                          transformed_angular_rotation, delta_t);
  SaitoIMUSystemModel::AMatrix a_matrix = system_model_ptr_->get_a_matrix(
      prior_estimate_and_cov.state_estimate, transformed_angular_rotation, delta_t);

  SaitoIMUSystemModel::StateCovarianceMatrix sigma_t_given_t_minus_one =
      a_matrix * (prior_estimate_and_cov.covariance * a_matrix.transpose()) +
      q_;

  EKFSaitoModel::EstimateAndCovariance estimate_and_cov;
  estimate_and_cov.state_estimate = mu_t_given_t_minus_one;
  estimate_and_cov.covariance = sigma_t_given_t_minus_one;
  return estimate_and_cov;
}

EKFSaitoModel::EstimateAndCovariance EKFSaitoModel::update(
    const EKFSaitoModel::EstimateAndCovariance& estimate_and_cov,
    const SaitoIMUSystemModel::SensorDataMatrix& accelerometer,
    const SaitoIMUSystemModel::SensorDataMatrix& angular_rotation,
    const SaitoIMUSystemModel::SensorDataMatrix& magnetometer,
    const m_t& delta_t) {

  SaitoIMUSystemModel::SensorDataMatrix transformed_accelerometer = rotate_sensor_to_base_frame(accelerometer);
  SaitoIMUSystemModel::SensorDataMatrix transformed_angular_rotation = rotate_sensor_to_base_frame(angular_rotation);
  SaitoIMUSystemModel::SensorDataMatrix transformed_magnetometer = rotate_sensor_to_base_frame(magnetometer);

  SaitoIMUSystemModel::MeasurementMatrix expected_measurement =
      system_model_ptr_->get_expected_measurment(
          estimate_and_cov.state_estimate);

  //std::cout << "state matrix" << std::endl;
  //std::cout << estimate_and_cov.state_estimate << std::endl;
  SaitoIMUSystemModel::MeasurementMatrix measurement =
      system_model_ptr_->get_measurement(estimate_and_cov.state_estimate,
                                         transformed_accelerometer, transformed_magnetometer);
  //std::cout << "expected measurement:\n" << expected_measurement << std::endl;
  //std::cout << "measurement:\n" << measurement << std::endl;

  SaitoIMUSystemModel::MeasurementMatrix error_vs_estimate =
      measurement - expected_measurement;

  SaitoIMUSystemModel::CMatrix c_matrix =
      system_model_ptr_->get_c_matrix(estimate_and_cov.state_estimate);

  Eigen::Matrix<m_t, 3, 4> sigma_ct =
      estimate_and_cov.covariance * c_matrix.transpose();
  SaitoIMUSystemModel::MeasurementCovarianceMatrix c_sigma_ct =
      c_matrix * sigma_ct;
  SaitoIMUSystemModel::MeasurementCovarianceMatrix c_sigma_ct_plus_R =
      c_sigma_ct + r_;
  SaitoIMUSystemModel::MeasurementCovarianceMatrix error_scaling_paranthesis =
      c_sigma_ct_plus_R.inverse();
  Eigen::Matrix<m_t, 3, 4> error_scaling =
      estimate_and_cov.covariance *
      (c_matrix.transpose() * error_scaling_paranthesis);

  SaitoIMUSystemModel::StateMatrix mu_t_given_t =
      estimate_and_cov.state_estimate + error_scaling * error_vs_estimate;
  //std::cout << "mu t given t " << std::endl;
  //std::cout << mu_t_given_t << std::endl;
  SaitoIMUSystemModel::StateCovarianceMatrix sigma_t_given_t =
      estimate_and_cov.covariance -
      error_scaling * (c_matrix * estimate_and_cov.covariance);

  EKFSaitoModel::EstimateAndCovariance update_estimate_and_cov;
  update_estimate_and_cov.state_estimate = mu_t_given_t;
  update_estimate_and_cov.covariance = sigma_t_given_t;
  return update_estimate_and_cov;
}

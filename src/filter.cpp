#include "filter.h"

m_t do_cos(const m_t& angle) { return cos(angle); }
m_t do_sin(const m_t& angle) { return sin(angle); }
m_t do_tan(const m_t& angle) { return tan(angle); }
m_t do_arctan(const m_t& y, const m_t& x) { return atan2(y, x); }

SaitoIMUSystemModel::SaitoIMUSystemModel() {}
SaitoIMUSystemModel::~SaitoIMUSystemModel() {}

SaitoIMUSystemModel::AMatrix get_a_matrix(
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

SaitoIMUSystemModel::CMatrix get_c_matrix(
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

SaitoIMUSystemModel::StateMatrix transition_state(
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

  new_state[0, 0] = phi + delta_t * omega_x +
                    delta_t * (sin_phi * tan_theta * omega_y) +
                    cos_phi * tan_theta * omega_z * delta_t;
  new_state[1, 0] =
      theta + delta_t * (cos_phi * omega_y) - delta_t * (sin_psi * omega_z);
  new_state[2, 0] = psi + delta_t * (sin_phi * 1 / cos_theta * omega_y) +
                    delta_t * (cos_phi * 1 / cos_theta * omega_z);

  return new_state;
}

SaitoIMUSystemModel::MeasurementMatrix get_measurement(
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

  m_t cos_theta = do_cos(theta);
  m_t sin_theta = do_sin(theta);
  m_t sin_phi = do_sin(phi);
  m_t cos_phi = do_cos(phi);

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

  // had to flip signs here to get it to line up... where in paper it was neg
  // m_y, pos m_x.
  const m_t& rotated_m_y = rotated_magnetometer(1, 0);
  const m_t& rotated_m_x = -1 * rotated_magnetometer(0, 0);

  const m_t new_psi = do_arctan(rotated_m_y, rotated_m_x);

  measurement(0, 0) = new_psi;
  measurement(Eigen::seq(1, 3), 0) = accelerometer(Eigen::seq(0, 2), 0);

  return measurement;
}

SaitoIMUSystemModel::MeasurementMatrix get_expected_measurment(
    const SaitoIMUSystemModel::StateMatrix& state) {
  SaitoIMUSystemModel::MeasurementMatrix expected_measurement;

  const m_t& phi = state(0, 0);
  const m_t& theta = state(1, 0);
  const m_t& psi = state(2, 0);

  const m_t sin_theta = do_sin(theta);
  const m_t cos_theta = do_cos(theta);
  const m_t sin_phi = do_sin(phi);
  const m_t cos_phi = do_cos(phi);

  expected_measurement(1, 0) = GRAVITY * (-1 * sin_theta);
  expected_measurement(2, 0) = GRAVITY * sin_phi * cos_theta;
  expected_measurement(3, 0) = GRAVITY * cos_phi * cos_theta;
  expected_measurement(0, 0) = psi;
  return expected_measurement;
}

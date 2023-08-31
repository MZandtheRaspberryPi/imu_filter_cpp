#include "filter.h"

SaitoIMUSystemModel::SaitoIMUSystemModel() {}
SaitoIMUSystemModel::~SaitoIMUSystemModel() {}

SaitoIMUSystemModel::AMatrix get_a_matrix() {
  SaitoIMUSystemModel::AMatrix a_matrix;
  return a_matrix;
}

SaitoIMUSystemModel::CMatrix get_c_matrix() {
  SaitoIMUSystemModel::CMatrix c_matrix;
  return c_matrix;
}

SaitoIMUSystemModel::StateMatrix transition_state() {
  SaitoIMUSystemModel::StateMatrix state_matrix;
  return state_matrix;
}

SaitoIMUSystemModel::MeasurementMatrix get_measurement() {
  SaitoIMUSystemModel::MeasurementMatrix measurement_matrix;
  return measurement_matrix;
}

SaitoIMUSystemModel::MeasurementMatrix get_expected_measurment() {
  SaitoIMUSystemModel::MeasurementMatrix measurement_matrix;
  return measurement_matrix;
}

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
#define GRAVITY -9.81

m_t do_cos(const m_t& angle);
m_t do_sin(const m_t& angle);
m_t do_tan(const m_t& angle);
m_t do_arctan(const m_t& y, const m_t& x);

m_t rad_to_degrees(const m_t& angle);

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

  IMUNonLinearSystemModel(){};
  virtual ~IMUNonLinearSystemModel(){};
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

template <typename T>
class FilterNonLinearModel {
 public:
  struct EstimateAndCovariance {
    typename T::StateMatrix state_estimate;
    typename T::StateCovarianceMatrix covariance;
  };

  FilterNonLinearModel(const RotationMatrix& sensor_to_base) {
    sensor_to_base_rotation_ = sensor_to_base;
  }
  ~FilterNonLinearModel() {}
  virtual EstimateAndCovariance predict(
      const EstimateAndCovariance& prior_estimate_and_cov,
      const typename T::SensorDataMatrix& angular_rotation,
      const m_t& delta_t) = 0;
  virtual EstimateAndCovariance update(
      const EstimateAndCovariance& prior_estimate_and_cov,
      const typename T::SensorDataMatrix& accelerometer,
      const typename T::SensorDataMatrix& angular_rotation,
      const typename T::SensorDataMatrix& magnetometer, const m_t& delta_t) = 0;

  typename T::SensorDataMatrix rotate_sensor_to_base_frame(
      const typename T::SensorDataMatrix& sensor_data) {
    return sensor_to_base_rotation_ * sensor_data;
  }

  EstimateAndCovariance predict_and_update();

 protected:
  std::unique_ptr<T> system_model_ptr_;
  typename T::StateCovarianceMatrix q_;
  typename T::MeasurementCovarianceMatrix r_;
  RotationMatrix sensor_to_base_rotation_;
};

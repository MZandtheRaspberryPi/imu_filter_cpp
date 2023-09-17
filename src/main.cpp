#include "filter_saito_ekf.h"
#include "util.h"
#include "websocket_broadcaster.h"
#include "websocket_listener.h"

bool EXIT_FLAG = false;
const uint16_t LOOP_DELAY_MS = 2;

void my_signal_handler(int s) {
  printf("Caught signal %d\n", s);
  EXIT_FLAG = true;
}

void setup_sigint_handler(struct sigaction &sig_int_handler) {
  sig_int_handler.sa_handler = my_signal_handler;
  sigemptyset(&sig_int_handler.sa_mask);
  sig_int_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_int_handler, NULL);
}

imu_msgs::ImuMsg get_filter_imu_msg(
    const imu_msgs::ImuMsg &imu_msg,
    const EKFSaitoModel::EstimateAndCovariance &estimate_and_cov) {
  imu_msgs::ImuMsg filter_msg;

  filter_msg.CopyFrom(imu_msg);
  filter_msg.set_filter_timestamp(get_timestamp());

  imu_msgs::Triad *euler_angles = filter_msg.mutable_euler_angles_filter();
  euler_angles->set_x(rad_to_degrees(estimate_and_cov.state_estimate(0, 0)));
  euler_angles->set_y(rad_to_degrees(estimate_and_cov.state_estimate(1, 0)));
  euler_angles->set_z(rad_to_degrees(estimate_and_cov.state_estimate(2, 0)));

  imu_msgs::CovarianceMatrix *cov_matrix =
      filter_msg.mutable_cov_matrix_filter();

  int num_rows = static_cast<int>(estimate_and_cov.covariance.rows());
  int num_cols = static_cast<int>(estimate_and_cov.covariance.cols());

  for (int i = 0; i < num_rows; i++) {
    imu_msgs::MatrixRow *row = cov_matrix->add_row();
    for (int j = 0; j < num_cols; j++) {
      row->add_val(estimate_and_cov.covariance(i, j));
    }
  }
  return filter_msg;
}

int main(int argc, char *argv[]) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  struct sigaction sig_int_handler;
  setup_sigint_handler(sig_int_handler);

  delay(10000);

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
  sensor_to_base(0, 0) = -1;
  sensor_to_base(2, 2) = -1;

  EKFSaitoModel ekf_saito(sensor_to_base, Q, R);

  ListenerClient listener_server = ListenerClient();
  std::string ip_address = "broadcaster";
  uint16_t port = 9000;
  listener_server.connect(ip_address, port);
  listener_server.start_non_blocking();

  BroadcastServer broadcast_server = BroadcastServer();
  broadcast_server.start_listening(9001);

  uint64_t last_timestamp = 0;

  while (!EXIT_FLAG) {
    if (listener_server.has_msg()) {
      imu_msgs::ImuMsg msg = listener_server.get_msg();
      if (!msg.IsInitialized()) {
        continue;
      }

      if (!msg.has_timestamp()) {
        continue;
      }
      uint64_t cur_timestamp = msg.timestamp();

      if (last_timestamp == 0) {
        last_timestamp = cur_timestamp;
        continue;
      }

      m_t delta_t = static_cast<m_t>((cur_timestamp - last_timestamp)) /
                    static_cast<m_t>(1000);
      last_timestamp = cur_timestamp;

      if (msg.has_angular_acceleration()) {
        SaitoIMUSystemModel::SensorDataMatrix angular_accel{
            msg.angular_acceleration().x(), msg.angular_acceleration().y(),
            msg.angular_acceleration().z()};
        EKFSaitoModel::EstimateAndCovariance estimate_given_t_minus_one =
            ekf_saito.predict(estimate_and_cov, angular_accel, delta_t);
        estimate_and_cov = estimate_given_t_minus_one;
      }

      if (msg.has_angular_acceleration() && msg.has_linear_acceleration() &&
          msg.has_magnetometer_vector()) {
        SaitoIMUSystemModel::SensorDataMatrix angular_accel{
            msg.angular_acceleration().x(), msg.angular_acceleration().y(),
            msg.angular_acceleration().z()};
        SaitoIMUSystemModel::SensorDataMatrix accelerometer{
            msg.linear_acceleration().x(), msg.linear_acceleration().y(),
            msg.linear_acceleration().z()};

        SaitoIMUSystemModel::SensorDataMatrix magnetometer{
            msg.magnetometer_vector().x(), msg.magnetometer_vector().y(),
            msg.magnetometer_vector().z()};

        EKFSaitoModel::EstimateAndCovariance estimate_given_t =
            ekf_saito.update(estimate_and_cov, accelerometer, angular_accel,
                             magnetometer, delta_t);

        estimate_and_cov = estimate_given_t;
      }

      /*
      if (estimate_and_cov.state_estimate.array().isNaN().all() ||
          estimate_and_cov.covariance.array().isNaN().all())
      {
        estimate_and_cov.state_estimate.setZero();
        estimate_and_cov.covariance.setIdentity();
      }
      */

      imu_msgs::ImuMsg filter_msg =
          get_filter_imu_msg(imu_msg, estimate_and_cov);

      std::cout << "our x: " << static_cast<int>(euler_angles->x());
      std::cout << " y: " << static_cast<int>(euler_angles->y());
      std::cout << " z: " << static_cast<int>(euler_angles->z()) << std::endl;
      std::cout << "their x: "
                << static_cast<int>(filter_msg.euler_angles().x());
      std::cout << " y: " << static_cast<int>(filter_msg.euler_angles().y());
      std::cout << " z: " << static_cast<int>(filter_msg.euler_angles().z())
                << std::endl
                << std::endl;

      broadcast_server.send_imu_msg(filter_msg);

      std::string debug_str = filter_msg.DebugString();
      // std::cout << debug_str << std::endl;
    }

    delay(LOOP_DELAY_MS);
  }

  listener_server.stop();
  broadcast_server.stop_listening();

  return 0;
}

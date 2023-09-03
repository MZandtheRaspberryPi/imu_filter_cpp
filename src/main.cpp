#include "filter_saito_ekf.h"
#include "util.h"
#include "websocket_broadcaster.h"
#include "websocket_listener.h"

bool EXIT_FLAG = false;
const uint16_t LOOP_DELAY_MS = 10;

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

int main(int argc, char *argv[]) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  struct sigaction sig_int_handler;
  setup_sigint_handler(sig_int_handler);

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

      m_t delta_t = (cur_timestamp - last_timestamp) / 1000;

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

      imu_msgs::ImuMsg filter_msg;

      filter_msg.CopyFrom(msg);
      filter_msg.set_filter_timestamp(get_timestamp());

      imu_msgs::Triad *euler_angles = filter_msg.mutable_euler_angles_filter();
      euler_angles->set_x(estimate_and_cov.state_estimate(0, 0));
      euler_angles->set_y(estimate_and_cov.state_estimate(1, 0));
      euler_angles->set_z(estimate_and_cov.state_estimate(2, 0));

      size_t msg_size = filter_msg.ByteSizeLong();
      uint8_t *msg_arr = new uint8_t[msg_size];
      filter_msg.SerializeToArray(msg_arr, msg_size);
      broadcast_server.send_message(msg_arr, msg_size);
      delete[] msg_arr;

      std::string debug_str = filter_msg.DebugString();
      std::cout << debug_str << std::endl;
    }

    delay(LOOP_DELAY_MS);
  }

  listener_server.stop();
  broadcast_server.stop_listening();

  return 0;
}
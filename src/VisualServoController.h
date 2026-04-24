#pragma once

#include <QColor>
#include <QImage>
#include <QString>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

struct VisualServoOptions {
  double tag_size = 0.05;
  double desired_factor = 8.0;
  bool tag_z_aligned = false;
  std::string robot_ip = "192.168.1.1";
  std::string eMc_filename;
  std::string intrinsic_filename;
  std::string camera_name = "Camera";
  int quad_decimate = 2;
  bool verbose = false;
  bool adaptive_gain = false;
  bool task_sequencing = false;
  double convergence_threshold = 0.00005;
  int mode = 1;
};

class VisualServoController
{
public:
  explicit VisualServoController(const VisualServoOptions &options);
  ~VisualServoController();

  bool initialize(QString &error_message);
  bool step(QString &error_message);
  void shutdown();

  void setMotionEnabled(bool enabled);
  bool motionEnabled() const { return send_velocities_; }

  void adjustDesiredFactor(double delta);
  void moveToRecoveryPose(QString &error_message);

  const QImage &frame() const { return frame_rgb_; }
  QString statusLine1() const { return status_line_1_; }
  QString statusLine2() const { return status_line_2_; }
  QString statusLine3() const { return status_line_3_; }
  QColor statusLine3Color() const { return status_line_3_color_; }
  QString operatorStatus() const { return operator_status_; }
  QColor operatorStatusColor() const { return operator_status_color_; }

  double desiredFactor() const { return opt_desired_factor_; }
  double desiredDistanceCm() const { return 100.0 * opt_tag_size_ * opt_desired_factor_; }
  bool converged() const { return has_converged_; }

private:
  void updateDesiredPose();
  void refreshStatusLines(int target_tag_id);
  void updateTrajectory(const std::vector<vpImagePoint> &corners, double now_ms);
  void renderFrame(const std::vector<vpImagePoint> *corners);
  void appendStatusLine3(const QString &text, const QColor &color);
  void applySafetySupervisor(vpColVector &v_c, bool tag_ok, const vpHomogeneousMatrix &c_M_o,
                             bool &contact_detected, bool &collision_detected,
                             bool &force_threshold_exceeded, bool &robot_error);
  void updateArmMovingState(bool moving, bool force = false);
  bool anyLevelTriggered(const std::array<double, 7> &levels) const;
  bool anyLevelTriggered(const std::array<double, 6> &levels) const;
  double commandNorm(const vpColVector &v) const;
  double forceNorm3(const std::array<double, 6> &wrench) const;
  double torqueNorm3(const std::array<double, 6> &wrench) const;
  bool postStateUpdate(const std::string &json_payload) const;

private:
  VisualServoOptions options_;

  double opt_tag_size_;
  double opt_desired_factor_;
  bool opt_tag_z_aligned_;
  std::string opt_robot_ip_;
  std::string opt_eMc_filename_;
  std::string opt_intrinsic_filename_;
  std::string opt_camera_name_;
  int opt_quad_decimate_;
  bool opt_verbose_;
  bool opt_adaptive_gain_;
  bool opt_task_sequencing_;
  double convergence_threshold_;
  int opt_mode_;

  double scan_backoff_secs_ = 3.0;
  double backoff_speed_ = 0.02;
  double bias_window_secs_ = 2.5;
  double k_ang_ = vpMath::rad(0.06);
  double max_linear_ = 0.05;
  double max_angular_ = vpMath::rad(20);
  double servo_max_linear_ = 0.02;
  double servo_max_angular_ = vpMath::rad(8);
  double orientation_stop_thresh_ = vpMath::rad(45);
  double vel_smooth_alpha_ = 0.3;
  double joint_limit_margin_ = vpMath::rad(10);
  double workspace_x_min_ = 0.20;
  double workspace_x_max_ = 0.75;
  double workspace_y_min_ = -0.55;
  double workspace_y_max_ = 0.55;
  double workspace_z_min_ = 0.10;
  double workspace_z_max_ = 0.85;
  double min_tag_distance_ = 0.20;
  double max_external_force_ = 20.0;
  double max_external_torque_ = 6.0;
  vpColVector recovery_joint_pose_;

  bool initialized_ = false;
  bool has_converged_ = false;
  bool send_velocities_ = false;
  bool servo_started_ = false;
  bool desired_pose_initialized_ = false;
  bool last_arm_moving_state_ = false;
  bool have_last_ = false;

  int no_tag_counter_ = 0;
  double lost_start_ms_ = -1.0;
  double t_prev_ = 0.0;
  double u_prev_ = 0.0;
  double v_prev_ = 0.0;
  double s_prev_ = 0.0;
  double t_last_ = 0.0;
  double u_last_ = 0.0;
  double v_last_ = 0.0;
  double s_last_ = 0.0;
  double t_init_servo_ = 0.0;
  double target_phase_start_ms_ = 0.0;
  size_t target_seq_idx_ = 0;

  QString status_line_1_;
  QString status_line_2_;
  QString status_line_3_;
  QColor status_line_3_color_ = QColor(Qt::white);
  QString operator_status_;
  QColor operator_status_color_ = QColor(Qt::white);

  vpRealSense2 rs_;
  vpCameraParameters cam_;
  vpDetectorAprilTag detector_;
  vpPoseVector e_P_c_;
  vpHomogeneousMatrix e_M_c_;
  vpHomogeneousMatrix cd_M_o_;
  vpHomogeneousMatrix c_M_o_;
  vpHomogeneousMatrix o_M_o_;
  vpServo task_;
  vpRobotFranka robot_;
  vpColVector joint_min_;
  vpColVector joint_max_;
  vpColVector v_c_filtered_;
  vpImage<unsigned char> frame_gray_;
  QImage frame_rgb_;

  std::vector<int> target_sequence_;
  std::vector<vpFeaturePoint> p_;
  std::vector<vpFeaturePoint> pd_;
  std::vector<vpPoint> point_;
  std::vector<std::deque<std::pair<double, vpImagePoint>>> traj_corners_;
  std::vector<vpImagePoint> last_corners_;
};

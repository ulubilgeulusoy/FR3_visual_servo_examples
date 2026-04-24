#include "VisualServoController.h"

#include <QPainter>
#include <QPen>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

namespace {
constexpr double kArmMovingCommandThreshold = 1e-3;

QColor toQColor(const vpColor &color)
{
  return QColor(color.R, color.G, color.B, color.A);
}

QPointF toQPointF(const vpImagePoint &p)
{
  return QPointF(p.get_u(), p.get_v());
}
} // namespace

VisualServoController::VisualServoController(const VisualServoOptions &options)
  : options_(options),
    opt_tag_size_(options.tag_size),
    opt_desired_factor_(options.desired_factor),
    opt_tag_z_aligned_(options.tag_z_aligned),
    opt_robot_ip_(options.robot_ip),
    opt_eMc_filename_(options.eMc_filename),
    opt_intrinsic_filename_(options.intrinsic_filename),
    opt_camera_name_(options.camera_name),
    opt_quad_decimate_(options.quad_decimate),
    opt_verbose_(options.verbose),
    opt_adaptive_gain_(options.adaptive_gain),
    opt_task_sequencing_(options.task_sequencing),
    convergence_threshold_(options.convergence_threshold),
    opt_mode_(options.mode),
    recovery_joint_pose_(7, 0.0),
    detector_(vpDetectorAprilTag::TAG_36h11),
    target_sequence_({1, 2, 1}),
    p_(4),
    pd_(4),
    point_(4),
    v_c_filtered_(6, 0.0),
    frame_gray_(480, 640),
    frame_rgb_(640, 480, QImage::Format_RGB32)
{
  recovery_joint_pose_[1] = vpMath::rad(-45);
  recovery_joint_pose_[3] = vpMath::rad(-135);
  recovery_joint_pose_[5] = vpMath::rad(90);
  recovery_joint_pose_[6] = vpMath::rad(45);
}

VisualServoController::~VisualServoController()
{
  shutdown();
}

bool VisualServoController::initialize(QString &error_message)
{
  try {
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    rs_.open(config);

    if (opt_intrinsic_filename_.empty()) {
      cam_ = rs_.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    }
    else if (!vpIoTools::checkFilename(opt_intrinsic_filename_)) {
      error_message = QString("Camera parameters file %1 doesn't exist.")
                        .arg(QString::fromStdString(opt_intrinsic_filename_));
      return false;
    }
    else {
      vpXmlParserCamera parser;
      if (parser.parse(cam_, opt_intrinsic_filename_, opt_camera_name_,
                       vpCameraParameters::perspectiveProjWithDistortion) != vpXmlParserCamera::SEQUENCE_OK &&
          parser.parse(cam_, opt_intrinsic_filename_, opt_camera_name_,
                       vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
        error_message = QString("Unable to parse camera parameters from %1.")
                          .arg(QString::fromStdString(opt_intrinsic_filename_));
        return false;
      }
    }

    detector_.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    detector_.setDisplayTag(true);
    detector_.setAprilTagQuadDecimate(opt_quad_decimate_);
    detector_.setZAlignedWithCameraAxis(opt_tag_z_aligned_);

    e_P_c_[0] = 0.0337731;
    e_P_c_[1] = -0.00535012;
    e_P_c_[2] = -0.0523339;
    e_P_c_[3] = -0.247294;
    e_P_c_[4] = -0.306729;
    e_P_c_[5] = 1.53055;
    if (!opt_eMc_filename_.empty()) {
      e_P_c_.loadYAML(opt_eMc_filename_, e_P_c_);
    }
    e_M_c_ = vpHomogeneousMatrix(e_P_c_);

    for (size_t i = 0; i < p_.size(); ++i) {
      task_.addFeature(p_[i], pd_[i]);
    }
    task_.setServo(vpServo::EYEINHAND_CAMERA);
    task_.setInteractionMatrixType(vpServo::CURRENT);
    if (opt_adaptive_gain_) {
      vpAdaptiveGain lambda(1.5, 0.4, 30);
      task_.setLambda(lambda);
    }
    else {
      task_.setLambda(0.5);
    }

    point_[0].setWorldCoordinates(-opt_tag_size_ / 2., -opt_tag_size_ / 2., 0);
    point_[1].setWorldCoordinates(+opt_tag_size_ / 2., -opt_tag_size_ / 2., 0);
    point_[2].setWorldCoordinates(+opt_tag_size_ / 2., +opt_tag_size_ / 2., 0);
    point_[3].setWorldCoordinates(-opt_tag_size_ / 2., +opt_tag_size_ / 2., 0);

    robot_.connect(opt_robot_ip_);
    joint_min_ = robot_.getJointMin();
    joint_max_ = robot_.getJointMax();
    robot_.set_eMc(e_M_c_);
    robot_.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    target_phase_start_ms_ = vpTime::measureTimeMs();
    t_init_servo_ = target_phase_start_ms_;

    updateDesiredPose();
    refreshStatusLines(-1);
    renderFrame(nullptr);

    initialized_ = true;
    return true;
  }
  catch (const std::exception &e) {
    error_message = QString::fromStdString(e.what());
    return false;
  }
}

bool VisualServoController::step(QString &error_message)
{
  if (!initialized_) {
    error_message = "Controller is not initialized.";
    return false;
  }

  try {
    const double t_start = vpTime::measureTimeMs();
    int target_tag_id = -1;
    if (opt_mode_ == 2) {
      if (t_start - target_phase_start_ms_ >= 5000.0) {
        target_seq_idx_ = (target_seq_idx_ + 1) % target_sequence_.size();
        target_phase_start_ms_ = t_start;
      }
      target_tag_id = target_sequence_[target_seq_idx_];
    }

    rs_.acquire(frame_gray_);

    std::vector<vpHomogeneousMatrix> c_M_o_vec;
    const bool ret = detector_.detect(frame_gray_, opt_tag_size_, cam_, c_M_o_vec);
    int target_idx = -1;
    if (ret && opt_mode_ == 2) {
      const std::vector<int> tags_id = detector_.getTagsId();
      for (size_t i = 0; i < tags_id.size() && i < c_M_o_vec.size(); ++i) {
        if (tags_id[i] == target_tag_id) {
          target_idx = static_cast<int>(i);
          break;
        }
      }
    }

    refreshStatusLines(target_tag_id);

    vpColVector v_c(6);
    bool tag_ok = false;
    size_t use_idx = 0;
    if (opt_mode_ == 1 && ret && c_M_o_vec.size() == 1) {
      tag_ok = true;
      use_idx = 0;
    }
    else if (opt_mode_ == 2 && ret && target_idx >= 0) {
      tag_ok = true;
      use_idx = static_cast<size_t>(target_idx);
    }

    std::vector<vpImagePoint> corners;
    if (tag_ok) {
      no_tag_counter_ = 0;
      c_M_o_ = c_M_o_vec[use_idx];
      lost_start_ms_ = -1.0;

      if (!desired_pose_initialized_) {
        std::vector<vpHomogeneousMatrix> secure_o_M_o(2), secure_cd_M_c(2);
        secure_o_M_o[1].buildFrom(0, 0, 0, 0, 0, M_PI);
        for (size_t i = 0; i < 2; ++i) {
          secure_cd_M_c[i] = cd_M_o_ * secure_o_M_o[i] * c_M_o_.inverse();
        }
        o_M_o_ = (std::fabs(secure_cd_M_c[0].getThetaUVector().getTheta()) <
                  std::fabs(secure_cd_M_c[1].getThetaUVector().getTheta()))
                   ? secure_o_M_o[0]
                   : secure_o_M_o[1];
        desired_pose_initialized_ = true;
        updateDesiredPose();
      }

      corners = detector_.getPolygon(use_idx);
      last_corners_ = corners;

      const double now_ms = vpTime::measureTimeMs();
      double u_cent = 0.0, v_cent = 0.0, perim = 0.0;
      for (size_t i = 0; i < corners.size(); ++i) {
        u_cent += corners[i].get_u();
        v_cent += corners[i].get_v();
        perim += vpImagePoint::distance(corners[i], corners[(i + 1) % corners.size()]);
      }
      u_cent /= corners.size();
      v_cent /= corners.size();

      if (have_last_) {
        t_prev_ = t_last_;
        u_prev_ = u_last_;
        v_prev_ = v_last_;
        s_prev_ = s_last_;
      }
      t_last_ = now_ms;
      u_last_ = u_cent;
      v_last_ = v_cent;
      s_last_ = perim;
      have_last_ = true;

      for (size_t i = 0; i < corners.size(); ++i) {
        vpFeatureBuilder::create(p_[i], cam_, corners[i]);
        vpColVector c_P;
        point_[i].changeFrame(c_M_o_, c_P);
        p_[i].set_Z(c_P[2]);
      }

      if (opt_task_sequencing_) {
        if (!servo_started_) {
          if (send_velocities_) {
            servo_started_ = true;
          }
          t_init_servo_ = vpTime::measureTimeMs();
        }
        v_c = task_.computeControlLaw((vpTime::measureTimeMs() - t_init_servo_) / 1000.);
      }
      else {
        v_c = task_.computeControlLaw();
      }

      for (size_t i = 0; i < 3; ++i) {
        v_c[i] = std::max(-servo_max_linear_, std::min(servo_max_linear_, v_c[i]));
      }
      for (size_t i = 3; i < 6; ++i) {
        v_c[i] = std::max(-servo_max_angular_, std::min(servo_max_angular_, v_c[i]));
      }

      vpHomogeneousMatrix cd_M_c_current = cd_M_o_ * o_M_o_ * c_M_o_.inverse();
      double theta_err_deg = vpMath::deg(cd_M_c_current.getRotationMatrix().getThetaUVector().getTheta());
      if (theta_err_deg > vpMath::deg(orientation_stop_thresh_)) {
        v_c = 0;
        appendStatusLine3("Orientation |error| >45 deg: holding position", QColor(255, 165, 0));
      }

      double error = task_.getError().sumSquare();
      if (error < convergence_threshold_) {
        has_converged_ = true;
        appendStatusLine3("Servo task has converged", QColor(Qt::white));
      }

      updateTrajectory(corners, now_ms);
    }
    else {
      if (lost_start_ms_ < 0.0) {
        lost_start_ms_ = vpTime::measureTimeMs();
      }
      const double dt_lost = (vpTime::measureTimeMs() - lost_start_ms_) / 1000.0;
      v_c = 0;
      v_c.resize(6);
      if (dt_lost < scan_backoff_secs_) {
        v_c[2] = -std::min(backoff_speed_, max_linear_);
      }
      else {
        double wx = 0.0, wy = 0.0, wz = 0.0;
        if (have_last_ && t_prev_ > 0.0 && (t_last_ - t_prev_) > 1e-3) {
          double dt_obs = (t_last_ - t_prev_) / 1000.0;
          double du_dt = std::max(-2000.0, std::min(2000.0, (u_last_ - u_prev_) / dt_obs));
          double dv_dt = std::max(-2000.0, std::min(2000.0, (v_last_ - v_prev_) / dt_obs));
          double bias_wz = 2.5 * (k_ang_ * du_dt);
          double bias_wx = 2.5 * (-k_ang_ * dv_dt);
          double bias_wy = 0.5 * k_ang_ * du_dt;
          if (dt_lost <= bias_window_secs_) {
            wx = bias_wx;
            wy = bias_wy;
            wz = bias_wz;
          }
          else {
            double decay = std::exp(-0.8 * (dt_lost - bias_window_secs_));
            wx = bias_wx * decay;
            wy = bias_wy * decay;
            wz = bias_wz * decay;
          }
        }
        wx = std::max(-max_angular_, std::min(max_angular_, wx));
        wy = std::max(-max_angular_, std::min(max_angular_, wy));
        wz = std::max(-max_angular_, std::min(max_angular_, wz));
        v_c[3] = wx;
        v_c[4] = wy;
        v_c[5] = wz;
        appendStatusLine3(opt_mode_ == 2 && target_tag_id >= 0
                            ? QString("Looking for Apriltag id %1 (36h11)").arg(target_tag_id)
                            : QString((dt_lost <= bias_window_secs_) ? "No tag: pure biased turn..."
                                                                     : "No tag: decaying biased turn..."),
                          QColor(255, 165, 0));
      }
    }

    if (!send_velocities_) {
      v_c = 0;
      v_c_filtered_ = 0;
    }
    else {
      for (unsigned int i = 0; i < v_c.size(); ++i) {
        const double prev = v_c_filtered_[i];
        v_c_filtered_[i] = vel_smooth_alpha_ * v_c[i] + (1.0 - vel_smooth_alpha_) * prev;
      }
      v_c = v_c_filtered_;
    }

    bool contact_detected = false;
    bool collision_detected = false;
    bool force_threshold_exceeded = false;
    bool robot_error = false;
    applySafetySupervisor(v_c, tag_ok, c_M_o_, contact_detected, collision_detected,
                          force_threshold_exceeded, robot_error);

    updateArmMovingState(send_velocities_ && commandNorm(v_c) > kArmMovingCommandThreshold);
    robot_.setVelocity(vpRobot::CAMERA_FRAME, v_c);

    renderFrame(tag_ok ? &last_corners_ : nullptr);
    return true;
  }
  catch (const std::exception &e) {
    error_message = QString::fromStdString(e.what());
    return false;
  }
}

void VisualServoController::shutdown()
{
  if (!initialized_) {
    return;
  }

  try {
    robot_.setRobotState(vpRobot::STATE_STOP);
  }
  catch (...) {
  }
  updateArmMovingState(false, true);
  initialized_ = false;
}

void VisualServoController::setMotionEnabled(bool enabled)
{
  send_velocities_ = enabled;
  if (enabled) {
    operator_status_.clear();
  }
}

void VisualServoController::adjustDesiredFactor(double delta)
{
  opt_desired_factor_ = std::max(3.0, std::min(20.0, opt_desired_factor_ + delta));
  updateDesiredPose();
}

void VisualServoController::moveToRecoveryPose(QString &error_message)
{
  if (!initialized_) {
    error_message = "Controller is not initialized.";
    return;
  }

  try {
    send_velocities_ = false;
    v_c_filtered_ = 0;
    robot_.setVelocity(vpRobot::CAMERA_FRAME, vpColVector(6, 0.0));
    robot_.setRobotState(vpRobot::STATE_STOP);
    robot_.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot_.setPosition(vpRobot::JOINT_STATE, recovery_joint_pose_);
    robot_.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    operator_status_ = "Recovery pose reached. Use START to resume servoing.";
    operator_status_color_ = QColor(Qt::white);
    error_message.clear();
  }
  catch (const std::exception &e) {
    error_message = QString::fromStdString(e.what());
  }
}

void VisualServoController::updateDesiredPose()
{
  cd_M_o_ = vpHomogeneousMatrix(vpTranslationVector(0, 0, opt_tag_size_ * opt_desired_factor_),
                                vpRotationMatrix({ 1, 0, 0, 0, -1, 0, 0, 0, -1 }));

  if (!desired_pose_initialized_) {
    return;
  }

  for (size_t i = 0; i < point_.size(); ++i) {
    vpColVector c_P, p_img;
    point_[i].changeFrame(cd_M_o_ * o_M_o_, c_P);
    point_[i].projection(c_P, p_img);
    pd_[i].set_x(p_img[0]);
    pd_[i].set_y(p_img[1]);
    pd_[i].set_Z(c_P[2]);
  }
}

void VisualServoController::refreshStatusLines(int target_tag_id)
{
  status_line_1_ = "START/STOP controls motion   HOME sends recovery pose";
  status_line_2_ = QString("Distance target: %1 cm   Level %2")
                     .arg(desiredDistanceCm(), 0, 'f', 1)
                     .arg(opt_desired_factor_, 0, 'f', 1);
  status_line_3_.clear();
  status_line_3_color_ = QColor(Qt::white);
  if (opt_mode_ == 2 && target_tag_id >= 0) {
    appendStatusLine3(QString("Targeting Apriltag id %1 (36h11)").arg(target_tag_id), QColor(Qt::white));
  }
}

void VisualServoController::updateTrajectory(const std::vector<vpImagePoint> &corners, double now_ms)
{
  if (traj_corners_.empty()) {
    traj_corners_.resize(corners.size());
  }
  for (size_t i = 0; i < corners.size(); ++i) {
    if (!traj_corners_[i].empty()) {
      if (vpImagePoint::distance(corners[i], traj_corners_[i].back().second) > 1.) {
        traj_corners_[i].push_back({now_ms, corners[i]});
      }
    }
    else {
      traj_corners_[i].push_back({now_ms, corners[i]});
    }
    while (!traj_corners_[i].empty() && (now_ms - traj_corners_[i].front().first) > 2000.0) {
      traj_corners_[i].pop_front();
    }
  }
}

void VisualServoController::renderFrame(const std::vector<vpImagePoint> *corners)
{
  frame_rgb_ = QImage(frame_gray_.getWidth(), frame_gray_.getHeight(), QImage::Format_RGB32);
  for (unsigned int y = 0; y < frame_gray_.getHeight(); ++y) {
    QRgb *scanline = reinterpret_cast<QRgb *>(frame_rgb_.scanLine(static_cast<int>(y)));
    for (unsigned int x = 0; x < frame_gray_.getWidth(); ++x) {
      const int value = frame_gray_[y][x];
      scanline[x] = qRgb(value, value, value);
    }
  }

  QPainter painter(&frame_rgb_);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QPen green_pen(QColor(0, 255, 0));
  green_pen.setWidth(2);
  QPen red_pen(QColor(255, 0, 0));
  red_pen.setWidth(1);
  painter.setFont(QFont(QStringLiteral("Monospace"), 10));

  if (corners && corners->size() == 4) {
    painter.setPen(green_pen);
    QPolygonF polygon;
    for (const auto &corner : *corners) {
      polygon << toQPointF(corner);
    }
    painter.drawPolygon(polygon);

    painter.setPen(red_pen);
    for (size_t i = 0; i < corners->size(); ++i) {
      painter.drawText(toQPointF((*corners)[i]) + QPointF(12.0, 12.0), QString::number(static_cast<int>(i)));
      vpImagePoint ip;
      vpMeterPixelConversion::convertPoint(cam_, pd_[i].get_x(), pd_[i].get_y(), ip);
      painter.drawLine(QPointF(ip.get_u() - 6, ip.get_v()), QPointF(ip.get_u() + 6, ip.get_v()));
      painter.drawLine(QPointF(ip.get_u(), ip.get_v() - 6), QPointF(ip.get_u(), ip.get_v() + 6));
      painter.drawText(QPointF(ip.get_u() + 10, ip.get_v() + 10), QString::number(static_cast<int>(i)));
    }
  }

  painter.setPen(green_pen);
  for (const auto &traj : traj_corners_) {
    for (size_t i = 1; i < traj.size(); ++i) {
      painter.drawLine(toQPointF(traj[i - 1].second), toQPointF(traj[i].second));
    }
  }
}

void VisualServoController::appendStatusLine3(const QString &text, const QColor &color)
{
  status_line_3_ = text;
  status_line_3_color_ = color;
}

void VisualServoController::applySafetySupervisor(vpColVector &v_c, bool tag_ok, const vpHomogeneousMatrix &c_M_o,
                                                  bool &contact_detected, bool &collision_detected,
                                                  bool &force_threshold_exceeded, bool &robot_error)
{
  bool hard_stop = false;
  bool workspace_hold = false;
  bool proximity_hold = false;
  bool joint_margin_hold = false;

  vpColVector q;
  robot_.getPosition(vpRobot::JOINT_STATE, q);
  for (unsigned int i = 0; i < std::min<unsigned int>(7, q.size()); ++i) {
    if ((q[i] - joint_min_[i]) < joint_limit_margin_ || (joint_max_[i] - q[i]) < joint_limit_margin_) {
      joint_margin_hold = true;
      hard_stop = true;
      appendStatusLine3("Safety stop: joint-limit margin reached", QColor(255, 165, 0));
      break;
    }
  }

  vpPoseVector o_P_c;
  robot_.getPosition(vpRobot::CAMERA_FRAME, o_P_c);
  vpHomogeneousMatrix o_M_c(o_P_c);
  const vpTranslationVector &o_t_c = o_M_c.getTranslationVector();
  if (!hard_stop &&
      (o_t_c[0] < workspace_x_min_ || o_t_c[0] > workspace_x_max_ ||
       o_t_c[1] < workspace_y_min_ || o_t_c[1] > workspace_y_max_ ||
       o_t_c[2] < workspace_z_min_ || o_t_c[2] > workspace_z_max_)) {
    workspace_hold = true;
    hard_stop = true;
    appendStatusLine3("Safety stop: camera left workspace bounds", QColor(255, 165, 0));
  }

  if (tag_ok && !hard_stop && c_M_o[2][3] <= min_tag_distance_) {
    proximity_hold = true;
    hard_stop = true;
    appendStatusLine3("Safety stop: camera too close to target", QColor(255, 165, 0));
  }

  franka::RobotState internal_state = robot_.getRobotInternalState();
  contact_detected = anyLevelTriggered(internal_state.joint_contact) ||
                     anyLevelTriggered(internal_state.cartesian_contact);
  collision_detected = anyLevelTriggered(internal_state.joint_collision) ||
                       anyLevelTriggered(internal_state.cartesian_collision);
  force_threshold_exceeded = forceNorm3(internal_state.O_F_ext_hat_K) > max_external_force_ ||
                             torqueNorm3(internal_state.O_F_ext_hat_K) > max_external_torque_;
  robot_error = static_cast<bool>(internal_state.current_errors);

  if (!hard_stop && (contact_detected || collision_detected || force_threshold_exceeded || robot_error)) {
    hard_stop = true;
    if (collision_detected) {
      appendStatusLine3("Safety stop: collision detected by Franka", QColor(255, 0, 0));
    }
    else if (contact_detected) {
      appendStatusLine3("Safety stop: contact detected by Franka", QColor(255, 0, 0));
    }
    else if (force_threshold_exceeded) {
      appendStatusLine3("Safety stop: external force/torque threshold exceeded", QColor(255, 0, 0));
    }
    else {
      appendStatusLine3("Safety stop: Franka reported control error", QColor(255, 0, 0));
    }
  }

  if (hard_stop) {
    v_c = 0;
    v_c_filtered_ = 0;
    send_velocities_ = false;
  }

  if (workspace_hold || proximity_hold || joint_margin_hold) {
    operator_status_.clear();
  }
}

void VisualServoController::updateArmMovingState(bool moving, bool force)
{
  if (!force && moving == last_arm_moving_state_) {
    return;
  }
  postStateUpdate(std::string("{\"arm_moving\":") + (moving ? "1" : "0") + "}");
  last_arm_moving_state_ = moving;
}

bool VisualServoController::anyLevelTriggered(const std::array<double, 7> &levels) const
{
  return std::any_of(levels.begin(), levels.end(), [](double level) { return level > 0.0; });
}

bool VisualServoController::anyLevelTriggered(const std::array<double, 6> &levels) const
{
  return std::any_of(levels.begin(), levels.end(), [](double level) { return level > 0.0; });
}

double VisualServoController::commandNorm(const vpColVector &v) const
{
  double sum = 0.0;
  for (unsigned int i = 0; i < v.size(); ++i) {
    sum += v[i] * v[i];
  }
  return std::sqrt(sum);
}

double VisualServoController::forceNorm3(const std::array<double, 6> &wrench) const
{
  return std::sqrt(wrench[0] * wrench[0] + wrench[1] * wrench[1] + wrench[2] * wrench[2]);
}

double VisualServoController::torqueNorm3(const std::array<double, 6> &wrench) const
{
  return std::sqrt(wrench[3] * wrench[3] + wrench[4] * wrench[4] + wrench[5] * wrench[5]);
}

bool VisualServoController::postStateUpdate(const std::string &json_payload) const
{
  std::ostringstream cmd;
  cmd << "curl -s -X POST http://127.0.0.1:8765/state "
      << "-H 'Content-Type: application/json' "
      << "-d '" << json_payload << "' >/dev/null 2>&1";
  return std::system(cmd.str().c_str()) == 0;
}

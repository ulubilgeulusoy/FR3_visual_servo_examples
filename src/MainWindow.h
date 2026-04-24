#pragma once

#include <QMainWindow>

class QLabel;
class QPushButton;
class QTimer;

class VisualServoController;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(VisualServoController *controller, QWidget *parent = nullptr);
  ~MainWindow() override = default;

protected:
  void closeEvent(QCloseEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

private slots:
  void onTick();
  void onMotionButtonClicked();
  void onZoomOutClicked();
  void onZoomInClicked();
  void onHomeClicked();
  void onQuitClicked();

private:
  void refreshUi();
  void updateImage();
  void updateStatusLabels();

private:
  VisualServoController *controller_;
  QLabel *image_label_;
  QLabel *status_line_1_label_;
  QLabel *status_line_2_label_;
  QLabel *status_line_3_label_;
  QPushButton *motion_button_;
  QPushButton *zoom_out_button_;
  QPushButton *zoom_in_button_;
  QPushButton *home_button_;
  QPushButton *quit_button_;
  QTimer *timer_;
  bool allow_close_ = false;
};

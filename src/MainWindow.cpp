#include "MainWindow.h"

#include "VisualServoController.h"

#include <QCloseEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

MainWindow::MainWindow(VisualServoController *controller, QWidget *parent)
  : QMainWindow(parent),
    controller_(controller)
{
  setWindowTitle("FR3 Visual Servo");
  setWindowFlags(Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint);
  resize(1100, 840);

  auto *central = new QWidget(this);
  auto *root_layout = new QVBoxLayout(central);
  root_layout->setContentsMargins(14, 14, 14, 14);
  root_layout->setSpacing(10);

  auto *top_bar = new QHBoxLayout();
  top_bar->addStretch();
  quit_button_ = new QPushButton("QUIT", this);
  quit_button_->setFixedWidth(120);
  quit_button_->setStyleSheet("QPushButton { background: #7a0f0f; color: white; font-weight: 700; padding: 8px 12px; }");
  connect(quit_button_, &QPushButton::clicked, this, &MainWindow::onQuitClicked);
  top_bar->addWidget(quit_button_);
  root_layout->addLayout(top_bar);

  image_label_ = new QLabel(this);
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(420, 300);
  image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  image_label_->setStyleSheet("QLabel { background: black; border: 1px solid #1f3a5f; }");
  root_layout->addWidget(image_label_, 1);

  auto *status_panel = new QWidget(this);
  status_panel->setStyleSheet("QWidget { background: #0c0c0c; border-top: 2px solid #173f6b; }");
  auto *status_layout = new QVBoxLayout(status_panel);
  status_layout->setContentsMargins(16, 12, 16, 12);
  status_layout->setSpacing(8);

  status_line_1_label_ = new QLabel(this);
  status_line_2_label_ = new QLabel(this);
  status_line_3_label_ = new QLabel(this);
  status_line_1_label_->setWordWrap(true);
  status_line_2_label_->setWordWrap(true);
  status_line_3_label_->setWordWrap(true);
  status_line_1_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  status_line_2_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  status_line_3_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  status_line_1_label_->setStyleSheet("QLabel { color: white; font: 600 16px 'DejaVu Sans Mono'; }");
  status_line_2_label_->setStyleSheet("QLabel { color: #ffd54a; font: 600 16px 'DejaVu Sans Mono'; }");
  status_line_3_label_->setStyleSheet("QLabel { color: white; font: 600 16px 'DejaVu Sans Mono'; }");
  status_layout->addWidget(status_line_1_label_);
  status_layout->addWidget(status_line_2_label_);
  status_layout->addWidget(status_line_3_label_);
  status_layout->setSizeConstraint(QLayout::SetMinimumSize);

  auto *controls_layout = new QHBoxLayout();
  controls_layout->setSpacing(12);
  home_button_ = new QPushButton("HOME", this);
  zoom_out_button_ = new QPushButton("ZOOM OUT", this);
  zoom_in_button_ = new QPushButton("ZOOM IN", this);
  motion_button_ = new QPushButton("START", this);

  home_button_->setStyleSheet("QPushButton { background: #1f7a38; color: white; font-weight: 700; padding: 10px 16px; }");
  zoom_out_button_->setStyleSheet("QPushButton { background: #124a9c; color: white; font-weight: 700; padding: 10px 16px; }");
  zoom_in_button_->setStyleSheet("QPushButton { background: #124a9c; color: white; font-weight: 700; padding: 10px 16px; }");
  motion_button_->setStyleSheet("QPushButton { background: #1f7a38; color: white; font-weight: 700; padding: 10px 16px; }");

  connect(home_button_, &QPushButton::clicked, this, &MainWindow::onHomeClicked);
  connect(zoom_out_button_, &QPushButton::clicked, this, &MainWindow::onZoomOutClicked);
  connect(zoom_in_button_, &QPushButton::clicked, this, &MainWindow::onZoomInClicked);
  connect(motion_button_, &QPushButton::clicked, this, &MainWindow::onMotionButtonClicked);

  controls_layout->addWidget(home_button_);
  controls_layout->addStretch();
  controls_layout->addWidget(zoom_out_button_);
  controls_layout->addWidget(zoom_in_button_);
  controls_layout->addWidget(motion_button_);
  status_layout->addLayout(controls_layout);

  root_layout->addWidget(status_panel);

  setCentralWidget(central);

  timer_ = new QTimer(this);
  timer_->setInterval(33);
  connect(timer_, &QTimer::timeout, this, &MainWindow::onTick);
  timer_->start();

  refreshUi();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  if (!allow_close_) {
    event->ignore();
    return;
  }
  controller_->shutdown();
  event->accept();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
  QMainWindow::resizeEvent(event);
  updateImage();
}

void MainWindow::onTick()
{
  QString error_message;
  if (!controller_->step(error_message)) {
    timer_->stop();
    QMessageBox::critical(this, "Visual Servo Error", error_message);
    return;
  }
  refreshUi();
}

void MainWindow::onMotionButtonClicked()
{
  controller_->setMotionEnabled(!controller_->motionEnabled());
  refreshUi();
}

void MainWindow::onZoomOutClicked()
{
  controller_->adjustDesiredFactor(1.0);
  refreshUi();
}

void MainWindow::onZoomInClicked()
{
  controller_->adjustDesiredFactor(-1.0);
  refreshUi();
}

void MainWindow::onHomeClicked()
{
  QString error_message;
  controller_->moveToRecoveryPose(error_message);
  if (!error_message.isEmpty()) {
    QMessageBox::warning(this, "Recovery Pose", error_message);
  }
  refreshUi();
}

void MainWindow::onQuitClicked()
{
  allow_close_ = true;
  close();
}

void MainWindow::refreshUi()
{
  updateStatusLabels();
  updateImage();

  if (controller_->motionEnabled()) {
    motion_button_->setText("STOP");
    motion_button_->setStyleSheet("QPushButton { background: #b36a00; color: white; font-weight: 700; padding: 10px 16px; }");
  }
  else {
    motion_button_->setText("START");
    motion_button_->setStyleSheet("QPushButton { background: #1f7a38; color: white; font-weight: 700; padding: 10px 16px; }");
  }
}

void MainWindow::updateImage()
{
  const QImage &frame = controller_->frame();
  if (frame.isNull()) {
    return;
  }
  QPixmap pixmap = QPixmap::fromImage(frame);
  image_label_->setPixmap(pixmap.scaled(image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void MainWindow::updateStatusLabels()
{
  status_line_1_label_->setText(controller_->statusLine1());
  status_line_2_label_->setText(controller_->statusLine2());
  const QString tertiary = controller_->operatorStatus().isEmpty()
                             ? controller_->statusLine3()
                             : controller_->operatorStatus();
  const QColor tertiary_color = controller_->operatorStatus().isEmpty()
                                  ? controller_->statusLine3Color()
                                  : controller_->operatorStatusColor();
  status_line_3_label_->setText(tertiary);
  status_line_3_label_->setStyleSheet(QString("QLabel { color: %1; font: 600 16px 'DejaVu Sans Mono'; }")
                                        .arg(tertiary_color.name()));
}

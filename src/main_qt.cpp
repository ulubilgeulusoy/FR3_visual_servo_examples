#include <iostream>

#include <QApplication>
#include <QMessageBox>

#include <visp3/core/vpConfig.h>

#include "MainWindow.h"
#include "VisualServoController.h"

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_FRANKA) && defined(VISP_HAVE_PUGIXML)
namespace {
void printHelp(const char *argv0)
{
  std::cout << "SYNOPSYS\n  " << argv0
            << " [--ip <controller ip>] [--intrinsic <xml file>] [--camera-name <name>]"
            << " [--tag-size <size>] [--tag-quad-decimate <decimation factor>] [--tag-z-aligned]"
            << " [--eMc <extrinsic transformation file>] [--mode <1|2>] [--adaptive-gain]"
            << " [--task-sequencing] [--no-convergence-threshold] [--verbose] [--help] [-h]\n\n";
}

bool parseOptions(int argc, char **argv, VisualServoOptions &options, bool &show_help, QString &error_message)
{
  show_help = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--tag-size" && i + 1 < argc) {
      options.tag_size = std::stod(argv[++i]);
    }
    else if (arg == "--desired-factor" && i + 1 < argc) {
      options.desired_factor = std::stod(argv[++i]);
    }
    else if (arg == "--tag-quad-decimate" && i + 1 < argc) {
      options.quad_decimate = std::stoi(argv[++i]);
    }
    else if (arg == "--tag-z-aligned") {
      options.tag_z_aligned = true;
    }
    else if (arg == "--ip" && i + 1 < argc) {
      options.robot_ip = argv[++i];
    }
    else if (arg == "--intrinsic" && i + 1 < argc) {
      options.intrinsic_filename = argv[++i];
    }
    else if (arg == "--camera-name" && i + 1 < argc) {
      options.camera_name = argv[++i];
    }
    else if (arg == "--eMc" && i + 1 < argc) {
      options.eMc_filename = argv[++i];
    }
    else if (arg == "--verbose") {
      options.verbose = true;
    }
    else if (arg == "--mode" && i + 1 < argc) {
      options.mode = std::stoi(argv[++i]);
      if (options.mode != 1 && options.mode != 2) {
        options.mode = 1;
      }
    }
    else if (arg == "--adaptive-gain") {
      options.adaptive_gain = true;
    }
    else if (arg == "--task-sequencing") {
      options.task_sequencing = true;
    }
    else if (arg == "--no-convergence-threshold") {
      options.convergence_threshold = 0.0;
    }
    else if (arg == "--help" || arg == "-h") {
      show_help = true;
      return true;
    }
    else {
      error_message = QString("Unsupported command line option: %1").arg(QString::fromStdString(arg));
      return false;
    }
  }
  return true;
}
} // namespace
#endif

int main(int argc, char **argv)
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x and rebuild ViSP." << std::endl;
  return EXIT_SUCCESS;
#elif !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka and rebuild ViSP." << std::endl;
  return EXIT_SUCCESS;
#elif !defined(VISP_HAVE_PUGIXML)
  std::cout << "Build ViSP with pugixml support enabled." << std::endl;
  return EXIT_SUCCESS;
#else
  QApplication app(argc, argv);

  VisualServoOptions options;
  bool show_help = false;
  QString error_message;
  if (!parseOptions(argc, argv, options, show_help, error_message)) {
    printHelp(argv[0]);
    std::cerr << error_message.toStdString() << std::endl;
    return EXIT_FAILURE;
  }
  if (show_help) {
    printHelp(argv[0]);
    return EXIT_SUCCESS;
  }

  VisualServoController controller(options);
  if (!controller.initialize(error_message)) {
    QMessageBox::critical(nullptr, "Initialization Error", error_message);
    return EXIT_FAILURE;
  }

  MainWindow window(&controller);
  window.show();
  const int rc = app.exec();
  controller.shutdown();
  return rc;
#endif
}

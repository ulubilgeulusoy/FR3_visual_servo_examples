# Third-Party Notices

This repository is licensed under `GPL-2.0-or-later`.

The project depends on the following third-party software:

## ViSP (Visual Servoing Platform)
- Role: Core visual servoing framework used directly by this project.
- Website: https://visp.inria.fr/
- License note: ViSP is distributed under GNU GPL v2 or later (and also offers a commercial/professional license from Inria). Because this repository links against ViSP, this repository is licensed under GPL-2.0-or-later.

## Qt5
- Role: GUI framework (`Widgets`, `Core`, `Gui`).
- Website: https://www.qt.io/
- License note: Qt is typically available under LGPL/commercial terms depending on modules and distribution model. Users are responsible for complying with the applicable Qt license terms for their build/distribution.

## libfranka
- Role: Franka robot communication/control dependency (used through ViSP and runtime capabilities).
- Website: https://github.com/frankaemika/libfranka
- License note: libfranka is provided under its own upstream license terms; users must comply with the upstream license in their deployments.

## Intel RealSense SDK 2.x (librealsense)
- Role: RealSense camera integration dependency (used through ViSP and runtime capabilities).
- Website: https://github.com/IntelRealSense/librealsense
- License note: Intel RealSense SDK is provided under its own upstream license terms; users must comply with the upstream license in their deployments.

## CMake
- Role: Build system generator required to configure/build this project.
- Website: https://cmake.org/
- License note: CMake is distributed under its own upstream license terms.

## Disclaimer
This file is a convenience notice and not a substitute for reading the full upstream license texts. Refer to each dependency's repository and distributed license files for authoritative terms.

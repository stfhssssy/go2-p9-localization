# Third-party notices

This release contains an integration of existing robotics software. It is not a new implementation of Point-LIO and is not endorsed by the upstream authors. Preserve the upstream license files and copyright headers with the corresponding source snapshot. This inventory records the notices observed in the actual P9 source tree; a package.xml license field alone is not sufficient to describe all bundled files.

## Point-LIO / Unitree L1 / autonomy_stack_go2

- Source chain: [HKU-MARS Point-LIO](https://github.com/hku-mars/Point-LIO), [Unitree point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar), and [jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2).
- P9 package: `point_lio_unilidar`; inspected source version and historical binary hashes are recorded in [provenance.md](docs/provenance.md).
- The package's `LICENSE` contains the GNU General Public License, version 2, June 1991. Its `package.xml` declares `BSD`; this is an upstream metadata discrepancy. This release does **not** represent the whole Point-LIO tree as BSD-only and does not remove the GPL text or embedded notices.
- Preserve the complete corresponding source, license texts, notices, and recorded build changes alongside any distributed binary. The private publication setting is not a replacement for these materials.
- The upstream adaptation README identifies Point-LIO and its paper, *Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry*. The inspected ROS package lists Lingbo Meng as maintainer/author; autonomy_stack_go2 credits Guofei Chen, Botao He, Guanya Shi, and Ji Zhang.

## IKFoM, manifold toolkit, and ikd-Tree

- Bundled `include/IKFoM/LICENSE` also contains GPL version 2 text. Do not discard it when copying the Point-LIO package.
- Individual IKFoM/manifold headers additionally retain BSD-style notices, including University of Hong Kong copyright (2019–2023), Dongjiao He attribution, and Universität Bremen copyright (2008–2011) and contributor notices. Their redistribution conditions, non-endorsement terms, and disclaimers remain in the source files. Preserve both the directory license and the file-level notices rather than rewriting the tree to a single inferred license.
- Bundled `include/ikd-Tree/ikd_Tree.cpp` identifies Yixi Cai as author of the incremental k-d tree. The inspected embedded copy has no separate ikd-Tree LICENSE file; retain the attribution and containing source license material, and do not invent a more permissive grant for this embedded copy. Upstream project: [HKU-MARS ikd-Tree](https://github.com/hku-mars/ikd-Tree).

## Other software used by the build/runtime

ROS 2, CycloneDDS, PCL, Eigen, Python, NumPy, SciPy, rosbag2, and the operating-system packages retain their respective licenses and notices. The image's installed distribution metadata and `/usr/share/doc` copyright files are part of the dependency record; this file does not relicense them. The base image `go2-control:arm64-v4` is a separately supplied dependency whose provenance and image ID must be retained in the release record.

RealSense/librealsense and realsense-ros are separate sensor-driver components. The recommended deployment uses the competition host's RSUSB driver and does not imply that its installation or the competition calibration file is bundled here. If a future image adds a driver, add its actual source version, build flags and license notices to the release inventory.

The historical Zhang `terrain_analysis` and `terrain_analysis_ext` packages declare BSD in package.xml. Listing their historical binary hashes does not imply that those packages are included in every P9 image. If they are bundled, retain their source and any applicable upstream notices and describe them in the build manifest.

Competition access material, team keys, VPN credentials and per-robot calibration files are not third-party code to redistribute with this release. The project documents interface requirements without publishing those access materials.

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

## Zhang Ji terrain and planning packages in the navigation release

This navigation release additionally vendors `terrain_analysis`, `terrain_analysis_ext`, `local_planner`, `far_planner`, and `visibility_graph_msg` derived from [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2/tree/43d5f54b389b251713f0097893c30fa76c870d54). The base reference revision is `43d5f54b389b251713f0097893c30fa76c870d54`. The actual source is the server archive `planner-source-reference.tar`, SHA256 `885a46f0a1c1e558fe99a5c3d85c738b21847fe054ea3bf10ee6ac3fc14efac0`; that server copy has no Git metadata and already contains compatibility changes. `provenance/planning-upstream.json` separately records base Git blobs, original server-file hashes, final release hashes, and local interface changes. The server copy is not represented as a clean upstream checkout.

All five package manifests declare `BSD`. No package-level license text was present in these inspected upstream subdirectories; this inventory reports the observed metadata without inventing a BSD variant or replacing file-level notices. FAR source retains `Copyright (C) 2021 Fan Yang - All rights reserved` and author contact notices. Preserve these headers, original package metadata, source provenance, and any applicable dependency notices with the distributed source and binaries. Upstream authors do not endorse this integration.

The local changes include Jazzy header/build compatibility, corrected `std_srvs` metadata, preserved terrain input timestamps, and removal of direct Unitree SportClient/API command sending from the built path follower. The FAR route-planning and local path-following implementations remain attributed to their original authors. The new ROS integration and explicit command guard are additions; this release does not claim a new FAR algorithm.

The retained `local_planner/include/nlohmann` headers identify JSON for Modern C++ 3.11.2, copyright 2013–2022 Niels Lohmann, with SPDX `MIT`; preserve their notices and the embedded third-party Hedley notices. Retained Unitree helper headers include the Unitree Robotics copyright notice. Those legacy helper files remain in the source snapshot for provenance; their presence does not mean that the navigation entrypoint enables direct robot actuation.

Competition access material, team keys, VPN credentials and per-robot calibration files are not third-party code to redistribute with this release. The project documents interface requirements without publishing those access materials.

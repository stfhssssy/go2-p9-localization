# P9 TARE native-architecture build

This recipe adds the official TARE planner to the already released calibrated P9
navigation image. It supports `linux/arm64` and `linux/amd64`. The base image must
match the target architecture and ROS 2 Jazzy. It does not add Gazebo.

The original TARE checkout ships x86-64 OR-Tools binaries. We replace that SDK
with the exact same **9.8.3296** version from Google's official release, using the
architecture-specific archive. TARE's `CMakeLists.txt`, TSP wrapper, first-solution
strategy (`PATH_CHEAPEST_ARC`) and coverage optimization code are unchanged by
this platform port. `P9_TARE.patch` remains the previously tested ROS integration
patch; it is checked separately by SHA-256.

From the release directory:

```bash
python3 build/fetch_build_inputs.py
docker image inspect go2-p9-nav:arm64-20261110 --format '{{.Id}} {{.Architecture}}'
docker build --platform linux/arm64 --target tare-build \
  --build-arg BASE_IMAGE=go2-p9-nav:arm64-20261110 \
  --build-arg BASE_IMAGE_ID="$(docker image inspect go2-p9-nav:arm64-20261110 --format '{{.Id}}')" \
  --build-arg BUILD_JOBS=2 \
  -f build/Dockerfile.tare -t go2-p9-tare-build:arm64-20261112 .
```

Build the complete deployable image from the same directory (inside the committed
repository's `exploration/` directory):

```bash
p9_commit=$(git rev-parse HEAD)
docker build --platform linux/arm64 --target release \
  --build-arg BASE_IMAGE=go2-p9-nav:arm64-20261110 \
  --build-arg BASE_IMAGE_ID="$(docker image inspect go2-p9-nav:arm64-20261110 --format '{{.Id}}')" \
  --build-arg SOURCE_REVISION="$p9_commit" --build-arg BUILD_JOBS=2 \
  -f build/Dockerfile.tare -t go2-p9-explore:arm64-20261112 .
```

The release stage also builds the pinned official Unitree ROS message packages,
then verifies calibration loading, control guards and command help. See
`build/unitree_interfaces/SOURCE_PROVENANCE.json` for unchanged message hashes.

For AMD64, replace `arm64` with `amd64` and use the matching base image. The `tare-build` target
compiles TARE before the release's runtime scripts are copied. The final release
target adds the live/replay CLI and control adapter. A cross-host build uses
registered QEMU binfmt interpreters; the produced binaries are still native ARM64
machine code. It is not a real-robot runtime performance benchmark.

The build checks the input archives, ELF architecture and runtime linkage, then
executes the exact upstream TSP wrapper on a four-node square tour and checks all
nodes and total distance. Results, source hashes, package versions and build log
are inside `/opt/p9-tare-provenance`. Runtime overlay: `/tare_ws/install`; OR-Tools
libraries: `/tare_ws/src/tare_planner/or-tools/lib`.

Source and license preservation:

- TARE commit and its full source remain under `/tare_ws`. Its `package.xml`
  declares BSD; this pinned checkout has no standalone `LICENSE` file. Existing
  source notices are preserved without inventing an upstream license text.
- Official OR-Tools SDK and `share/doc/ortools/LICENSE` remain under its extracted
  directory, and the original archive is retained under `/opt/p9-tare-build-inputs`.
- The unmodified upstream x86 SDK is retained under
  `/opt/p9-tare-build-inputs/upstream-ortools-amd64` for provenance.
- Base navigation source, licenses, calibration settings and manifest are inherited.
- Large archives are not committed to GitHub. `sources.json`, `SHA256SUMS` and
  `fetch_build_inputs.py` reproduce and verify them.

All third-party copyright and license terms remain applicable. The source pin and
artifact hashes make the main algorithm inputs reproducible; apt packages are
recorded after installation and are not locked to an archived Ubuntu apt snapshot.

The separate `TARE_FRONTIER_PARAMETER.patch` fixes an upstream ROS 2 parameter
initialization defect: `kUseFrontier` was read but never declared. Missing reads
could leave two boolean members indeterminate. The patch declares the parameter
and makes missing reads fail startup; it respects both explicit true and false.
This restores the intended frontier setting without replacing the coverage or
TSP algorithms. It is applied after the retained base build for an incremental
rebuild. Validation exercises actual node parameter services with both values.

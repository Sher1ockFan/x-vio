# mono_imu_vio: modified ORB-SLAM3 core

This is a modified single-camera inertial fork, based on upstream commit
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`.

Only `System::IMU_MONOCULAR` and `TrackMonocular(image, timestamp, imu_batch)`
are supported. Unsupported input APIs, settings readers, tracking branches,
stereo/RGB-D frame construction, and examples have been removed. Shared backend
structures used by keyframes, optimization and serialization remain.

See the project root README and `provenance/` for build instructions and change
records. The upstream authors' copyright and GPLv3 license remain in `LICENSE`
and the source headers. This fork is not the unmodified upstream release.

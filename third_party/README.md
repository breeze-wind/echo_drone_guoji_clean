# Third-party Sources

This directory vendors source dependencies needed to build the Foxy workspace.

| Directory | Upstream | Snapshot |
|---|---|---|
| `livox_ros_driver2` | `https://github.com/Livox-SDK/livox_ros_driver2.git` | `13eb05e` |
| `Livox-SDK2` | `https://github.com/Livox-SDK/Livox-SDK2.git` | `f5d9375` |

`Livox-SDK2` is installed into `/usr/local` and is ignored by colcon. The ROS2
workspace builds `livox_ros_driver2`, which depends on the installed SDK2
headers and shared library.

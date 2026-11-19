#!/usr/bin/env bash
set -eo pipefail

# Remove inherited production/custom overlays before sourcing only ROS and the
# newly copied archive workspace. local_setup avoids any recorded parent chain.
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH CMAKE_MODULE_PATH
unset COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD PKG_CONFIG_PATH
unset ROS_PACKAGE_PATH PYTHONHOME PYTHONSTARTUP PYTHONUSERBASE VIRTUAL_ENV CONDA_PREFIX
unset OMP_NUM_THREADS OPENBLAS_NUM_THREADS MKL_NUM_THREADS NUMEXPR_NUM_THREADS
unset BLIS_NUM_THREADS VECLIB_MAXIMUM_THREADS XLA_FLAGS JAX_PLATFORMS JAX_PLATFORM_NAME
unset TF_NUM_INTRAOP_THREADS TF_NUM_INTEROP_THREADS TELEOP_XR_ALLOW_NO_ROS
export PATH=/opt/venv-teleop-xr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source /test/ros2_ws/install/local_setup.bash
export PYTHONPATH="/test/vendor/python:/test/app${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/logs/ros HOME=/tmp/original-vr-home XDG_CACHE_HOME=/tmp/original-vr-cache
export ORIGINAL_VR_PORT="${ORIGINAL_VR_PORT:-4446}"
mkdir -p "$HOME" "$XDG_CACHE_HOME" "$ROS_LOG_DIR"

# Refuse a copied install that unexpectedly reintroduces another workspace.
/usr/bin/python3 -B - <<'PY'
import os
for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'PYTHONPATH', 'LD_LIBRARY_PATH'):
    for entry in filter(None, os.environ.get(key, '').split(':')):
        allowed = entry in ('/test/vendor/python', '/test/app') or any(
            entry == root or entry.startswith(root + '/')
            for root in ('/opt/ros/humble', '/test/ros2_ws/install'))
        if not allowed:
            raise SystemExit('Unexpected inherited path in ' + key + ': ' + entry)
PY
exec /usr/bin/python3 -B /test/ops/supervisor.py

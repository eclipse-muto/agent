cmake_minimum_required(VERSION 3.8)
project(muto_paho_mqtt_vendor NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Pinned paho-mqtt version required by muto_agent (uses the 2.x callback API).
set(PAHO_MQTT_VERSION "2.1.0" CACHE STRING "paho-mqtt version to vendor")

# Optional: directory containing a pre-downloaded wheel for offline builds
# (pip download paho-mqtt==2.1.0 --no-deps -d <dir>).
set(PAHO_MQTT_WHEEL_DIR "" CACHE PATH "Local directory with paho-mqtt wheel (offline build)")

# Resolves PYTHON_INSTALL_DIR (e.g. lib/python3.10/site-packages) and registers
# the PYTHONPATH environment hook so the vendored module is found after sourcing.
_ament_cmake_python_register_environment_hook()

set(_pip_args
  install
  --no-deps
  --no-compile
  --upgrade
  --disable-pip-version-check
  --no-warn-script-location
)
if(PAHO_MQTT_WHEEL_DIR)
  list(APPEND _pip_args --no-index --find-links "${PAHO_MQTT_WHEEL_DIR}")
endif()

# Install into a staging directory inside the build tree, then install that
# tree with CMake so the files are tracked in install_manifest.txt.
set(_stage_dir "${CMAKE_CURRENT_BINARY_DIR}/site-packages")
set(_stamp "${CMAKE_CURRENT_BINARY_DIR}/paho_mqtt_${PAHO_MQTT_VERSION}.stamp")

add_custom_command(
  OUTPUT "${_stamp}"
  COMMAND ${CMAKE_COMMAND} -E remove_directory "${_stage_dir}"
  COMMAND ${CMAKE_COMMAND} -E env PIP_BREAK_SYSTEM_PACKAGES=1
    ${Python3_EXECUTABLE} -m pip ${_pip_args}
    --target "${_stage_dir}"
    "paho-mqtt==${PAHO_MQTT_VERSION}"
  COMMAND ${CMAKE_COMMAND} -E touch "${_stamp}"
  COMMENT "Vendoring paho-mqtt==${PAHO_MQTT_VERSION}"
  VERBATIM
)
add_custom_target(paho_mqtt_vendor ALL DEPENDS "${_stamp}")

install(
  DIRECTORY "${_stage_dir}/"
  DESTINATION "${PYTHON_INSTALL_DIR}"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "bin" EXCLUDE
)

ament_package()

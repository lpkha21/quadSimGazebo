# quadSimGazebo

Quadcopter firmware simulation in **Gazebo Harmonic (gz-sim)** using a **Pixhawk x500**-style model and **SITL**.

> Goal: run a multicopter control stack / firmware in software (SITL) while Gazebo simulates physics + sensors, so you can test control, estimation, and basic autonomy without real hardware.

### OS
- Recommended: **Ubuntu 22.04+** (Linux is easiest for Gazebo Harmonic)

## Running the code
### starting from base directory
```bash
mkdir -p build
cd build
cmake ../sitl_firmware
make -j
./sitl_firmware
cd ..
export GZ_SIM_RESOURCE_PATH=$PWD/models:$PWD/worlds:${GZ_SIM_RESOURCE_PATH}
gz sim worlds/empty.sdf


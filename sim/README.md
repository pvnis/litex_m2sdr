# LiteX-M2SDR Simulation Testbench

This directory contains the simulation environment for testing LiteX-M2SDR modules (e.g., header, scheduler, or custom DUTs). The testbench is modular and config-driven, allowing you to run tests on any top-level module.

## Overview of the Simulation Pipeline

The simulation pipeline consists of several key components that work together to run tests, manage configurations, and generate outputs:

- **`testbench.py`**: The core, generalized testbench script. It dynamically imports and instantiates any top module (DUT) specified via `--top-module`. Handles stimulus generation, test execution, and simulation orchestration using LiteX's `run_simulation`.

- **`ExperimentManager.py`**: Manages experiment lifecycle, including creating VCD output directories, generating timestamps, and saving config reports/YAML files for reproducibility.

- **`ConfigLoader.py`**: Loads and parses YAML configuration files (e.g., `config_yaml/alltest_config.yaml`). Provides methods to access global parameters, test definitions, and generate reports.

- **`testbench_helpers.py`**: Contains utility functions for simulation stimuli, such as `wait()`, `drive_packet()`, and CSR operations (e.g., `write_manual_time()`, `read_time()`). These are imported and used in `testbench.py`.

- **Top Module Files**: Custom files defining the DUT (e.g., `sim/header_sim/top.py` with `Top` class). Each top inherits from `LiteXModule` and implements the module-specific logic (e.g., header extraction, scheduling).

- **Config Files**: YAML files in `config_yaml/` (e.g., `alltest_config.yaml`, `minimal_config.yaml`) define tests, global parameters, and packet stimuli.

- **Output**: VCD files for waveform analysis, saved in module-specific directories (e.g., `sim/header_sim/vcd_outputs/<timestamp>/`).

## Prerequisites
- Python 3.x
- LiteX framework
- GTKWave (for waveform visualization)
- PyYAML (for config parsing)
- YAML config files (e.g., `config_yaml/alltest_config.yaml`)

## Running the Testbench
The generalized `testbench.py` script orchestrates simulations for any top module. It uses a config-driven approach to define tests, parameters, and stimuli.

### Basic Usage
```bash
python3 testbench.py --top-module <module.path.TopClass> [options]
```

### Command-Line Options
- `--top-module`: (Required) Dotted path to the top module class (e.g., `sim.header_sim.top.Top` for header, `sim.scheduler_sim.top.Top` for scheduler).
- `--config-file`: YAML config file for tests (default: `config_yaml/alltest_config.yaml`).
- `--gtk`: Open the latest VCD in GTKWave after simulation.
- `--vcd-dir`: Directory for VCD files (defaults to module-specific folder, e.g., `sim/header_sim/vcd_outputs`).
- `--xp-name`: Experiment name (used in folder/VCD naming; defaults to timestamp).

### Examples
```bash
# Run header simulation with default config and auto-open GTKWave
python3 testbench.py --top-module sim.header_sim.top.Top --gtk

# Run scheduler simulation with custom experiment name
python3 testbench.py --top-module sim.scheduler_sim.top.Top --xp-name my_scheduler_test

# Run custom DUT with specific config and VCD directory
python3 testbench.py --top-module sim.my_custom_dut.top.Top --config-file config_yaml/custom_config.yaml --vcd-dir custom_vcd --xp-name custom_exp

# Override VCD directory manually
python3 testbench.py --top-module sim.header_sim.top.Top --vcd-dir /path/to/custom/vcd
```

### Config Files
- Tests are defined in YAML files (e.g., `config_yaml/alltest_config.yaml`).
- Supports static/dynamic packet generation, timestamps, and global parameters (e.g., `frames_per_packet`, `sys_freq`).
- Example structure:
  ```yaml
  global:
    frames_per_packet: 1024
    sys_freq: 50000000
  tests:
    test1:
      description: "Basic header test"
      packets:
        - id: 1
          timestamp: 100
  ```

## VCD Output and Visualization
- VCD files are saved in a module-specific directory (e.g., `sim/header_sim/vcd_outputs/<timestamp>/` for `--top-module sim.header_sim.top.Top`).
- Use GTKWave to view waveforms.

### Viewing Locally
```bash
# After running with --gtk, or manually:
gtkwave <path/to/vcd/file.vcd>
```

### Send VCD via SSH for Remote Visualization
On the client (e.g., WSL) after running the testbench:
```bash
# Download and open in GTKWave
scp sens@sensnuc6:~/litex_m2sdr/sim/<module_dir>/vcd_outputs/<xp_name>/<xp_name>.vcd .
gtkwave <xp_name>.vcd
```
- `<module_dir>`: Derived from `--top-module` (e.g., `header_sim` for `sim.header_sim.top.Top`).
- `<xp_name>`: Your `--xp-name` or auto-generated timestamp.

## Adding a New Top Module
1. Create a new directory/file structure (e.g., `sim/my_module/top.py`) with your `Top` class.
2. Ensure it inherits from `LiteXModule` and accepts params like `frames_per_packet`.
3. Run: `python3 testbench.py --top-module sim.my_module.top.Top`

## Pipeline Flow
1. **Config Loading**: `ConfigLoader` parses YAML and provides test/parameter access.
2. **Experiment Setup**: `ExperimentManager` creates directories and prepares for VCD/report output.
3. **Top Instantiation**: `testbench.py` dynamically imports and instantiates the specified `Top` class.
4. **Test Execution**: `Testbench` class runs stimuli via `stimulus()` and `drive_packet()`, using helpers from `testbench_helpers.py`.
5. **Simulation**: LiteX's `run_simulation` generates VCD and executes the DUT.
6. **Post-Simulation**: Reports are saved, and GTKWave can be launched if requested.

For issues or contributions, refer to the main project README.
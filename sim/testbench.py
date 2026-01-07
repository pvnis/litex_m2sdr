#!/usr/bin/env python3

from litex.gen import *
from ExperimentManager import ExperimentManager

import os, sys, argparse, subprocess, importlib

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, BASE_DIR)

sys_freq = 50000000  # sys clock for the sim (20 ns period)

class Testbench():
    """Main testbench orchestrator using config-driven approach."""
    
    def __init__(self, dut, config):

        self.dut = dut
        self.config = config
        self.wait_initial_cycles = self.config.get_global_param("wait_initial_cycles", 20)
        self.wait_final_cycles = self.config.get_global_param("wait_final_cycles", 500)
        self.frames_per_packet = self.config.get_global_param("frames_per_packet", 1024)
        self.data_width = self.config.get_global_param("data_width", 64)
        self.max_packets = self.config.get_global_param("max_packets", 8)
        self.current_time = 0

    
    def run_all_tests(self):
        """Run all tests from config."""
        print("\n[TB] ========== TESTBENCH ==========")
        self.config.list_tests()
        print("\n")
        for test_id ,test_name in enumerate(self.config.get_all_tests()):            
            yield from self.run_test(test_name, test_id + 1)
        
        print("\n[TB] ========== ALL TESTS COMPLETED ==========\n")
        yield
    
    def run_test(self, test_name, test_id):
        """Run a single test by name."""
        test_cfg = self.config.get_test(test_name)

        print("-" * 70)
        print(f"\n[TEST] {test_name}")
        print(f"[TEST] {test_cfg['description']}")
        print("-" * 70)
        
        # Initial wait
        yield from wait(self.wait_initial_cycles)
        self.current_time += self.wait_initial_cycles
        
        # Check if this is a dynamic packet test (num_packets) or static (packets list)
        if "num_packets" in test_cfg:
            yield from self._run_dynamic_packets(test_cfg, test_id)
        else:
            yield from self._run_static_packets(test_cfg, test_id)
        
        # Final wait
        wait_final = test_cfg.get("wait_after_all_packets", self.wait_final_cycles)
        yield from wait(wait_final)
        self.current_time += wait_final
        
        print(f"[TEST] {test_name}: DONE\n")
    
    def _run_static_packets(self, test_cfg, test_id):
        """Run test with a static list of packets."""
        packets = test_cfg.get("packets", [])
        
        for pkt_cfg in packets:
            pkt_id = pkt_cfg.get("id", 0)
            description = pkt_cfg.get("description", f"Packet {pkt_id}")
            
            # Determine timestamp (absolute or relative)
            if "timestamp" in pkt_cfg:
                ts = pkt_cfg["timestamp"]
            elif "timestamp_offset" in pkt_cfg:
                ts = self.current_time + pkt_cfg["timestamp_offset"]
            else:
                ts = self.current_time
            
            print(f"  {description}")
            yield from drive_packet(self.dut, ts_when_due=ts, packet_id=pkt_id, test_id = test_id)
            self.current_time += self.frames_per_packet
            
            # Wait between packets
            wait_between = self.config.get_global_param("wait_between_packets", 10)
            yield from wait(wait_between)
            self.current_time += wait_between
    
    def _run_dynamic_packets(self, test_cfg, test_id):
        """Run test with dynamically generated packets."""
        num_packets = test_cfg.get("num_packets", 1)
        base_offset = test_cfg.get("base_timestamp_offset", 100)
        increment = test_cfg.get("timestamp_increment", 100)
        desc_template = test_cfg.get("description_template", "Packet {id}: ts=base+{ts_offset}")
        
        base_time = self.current_time + base_offset
        
        for pkt_id in range(1, num_packets + 1):
            ts_offset = (pkt_id - 1) * increment
            ts = base_time + ts_offset
            description = desc_template.format(id=pkt_id, ts_offset=ts_offset)
            
            print(f"  {description}")
            yield from drive_packet(self.dut, ts_when_due=ts, packet_id=pkt_id, test_id = test_id)
            self.current_time += self.frames_per_packet
            
            # Wait between packets
            wait_between = self.config.get_global_param("wait_between_packets", 10)
            yield from wait(wait_between)
            self.current_time += wait_between

def stimulus(tb, test_name="all"):
    """Main stimulus function."""
    if test_name == "all":
        yield from tb.run_all_tests()
    else:
        yield from tb.run_test(test_name)

def drive_packet(dut, ts_when_due, packet_id=0, test_id = 1, header=0xDEADBEEF, frames_per_packet=1024, verbose=False):
    """
    Drive one packet into sched.sink with a given timestamp.
    
    Args:
        dut: Device under test
        ts_when_due: Timestamp for this packet
        packet_id: Packet identifier (for logging)
        header: Header word (default 0xDEADBEEF)
        frames_per_packet: Number of frames in the packet
    """
    print(f"      [drive] Packet {packet_id}: ts={ts_when_due}, frames={frames_per_packet}")
    
    yield dut.sink.valid.eq(1)
    yield dut.enable.eq(1)
    sink_ready = (yield dut.sink.ready)
    
    # Wait until FIFO can accept
    while sink_ready == 0:
        print(f"      [drive] Waiting for sink.ready...")
        yield
        sink_ready = (yield dut.sink.ready)
    
    # Drive the frames
    for i in range(frames_per_packet):
        if verbose:        print(f"[stim] Driving word {i}") 
        yield dut.sink.valid.eq(1)
        yield dut.sink.first.eq(i == 0)
        yield dut.sink.last.eq(i == (frames_per_packet - 1))
        if i == 0:
            yield dut.sink.data.eq(header)
        elif i == 1:
            yield dut.sink.data.eq(ts_when_due)
        else:
            yield dut.sink.data.eq((test_id << 32) | (packet_id << 16) | i)
        
        # Wait for handshake
        while True:
            ready = (yield dut.sink.ready)
            yield
            if ready:
                break
    
    yield dut.sink.valid.eq(0)
    print(f"      [drive] Packet {packet_id}: Complete\n")

def write_manual_time(dut, new_time):
    """Write a new manual time to the scheduler."""
    print(f"[CSR] Writing manual time: {new_time}")
    yield dut._write_time.storage.eq(new_time)
    yield dut.control.fields.write.eq(1)
    yield
    yield dut.control.fields.write.eq(0)
    print(f"[CSR] Manual time write complete.\n")

def read_time(dut):
    """Write a new manual time to the scheduler."""
    print(f"[CSR] Reading current time")
    yield dut.control.fields.read.eq(1)
    yield
    yield dut.control.fields.read.eq(0)
    print(f"[CSR] Time read complete.\n")

def read_current_ts(dut):
    """Read the current timestamp from the scheduler."""
    print(f"[CSR] Reading current timestamp")
    yield dut.control.fields.read_current_ts.eq(1)
    yield
    yield dut.control.fields.read_current_ts.eq(0)
    current_ts = (yield dut._current_ts.status)
    print(f"[CSR] Current timestamp read complete: {current_ts}\n")
    return current_ts

def read_fifo_level(dut):
    """Read the current FIFO level from the scheduler."""
    print(f"[CSR] Reading FIFO level")
    fifo_level = (yield dut._fifo_level.status)
    print(f"[CSR] FIFO level read complete: {fifo_level}\n")
    return fifo_level

def wait(wait_cycles):
    """Wait for N cycles."""
    for _ in range(wait_cycles):
        yield

def main():
    argparser = argparse.ArgumentParser(
        description="Testbench - LiteX-M2SDR Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
            Examples:
            # Run with header top, VCD in sim/header_sim
            python testbench.py --top sim.header_sim.top.Top
            
            # Run with scheduler top, VCD in sim/scheduler_sim
            python testbench.py --top sim.scheduler_sim.scheduler_tb.Top --xp-name mytest
            
            # Override VCD dir manually (optional)
            python testbench.py --top sim.scheduler_sim.scheduler_tb.Top --vcd-dir custom_vcd
            
            # Open latest experiment VCD in GTKWave automatically
            python testbench.py --top sim.scheduler_sim.scheduler_tb.Top --gtk
        """
    )
    argparser.add_argument("--config-file",     default="config_yaml/alltest_config.yaml", help="YAML configuration file for tests")
    argparser.add_argument("--gtk",             action="store_true",           help="Open GTKWave at end of simulation")
    argparser.add_argument("--vcd-dir",         default=None,                  help="Directory to store VCD files (overrides module-based default)")
    argparser.add_argument("--xp-name",         default="",                    help="Name of the experiment (used in folder and VCD naming)")
    argparser.add_argument("--top",      required=True,                help="Dotted path to the top module class (e.g., sim.scheduler_sim.scheduler_tb.Top)")
    args = argparser.parse_args()

    #handle config file path (must end in _config.yaml and be in config_yaml folder)
    if args.config_file.endswith("_config.yaml") and os.path.exists(args.config_file):
        config_file = args.config_file
    else:
        raise Exception("config file name must end in _config.yaml \nExample: mytest_config.yaml")
    
    # Dynamically import the top module
    try:
        module_path, class_name = args.top.rsplit('.', 1)
        top_module = importlib.import_module(module_path)
        Top = getattr(top_module, class_name)
    except (ImportError, AttributeError) as e:
        raise Exception(f"Failed to import top module '{args.top}': {e}")
    
    # Extract up to the second-to-last dot for the directory (e.g., sim.scheduler_sim)
    vcd_dir_parts = module_path.split('.')
    if vcd_dir_parts[0] == 'sim':
        vcd_dir_parts = vcd_dir_parts[1:]
        vcd_dir_parts.pop() # remove the last part (module name)
        vcd_dir_parts.append("vcd_outputs")
        vcd_dir = os.path.join(*vcd_dir_parts) #replace the . with / 

    else:
        raise Exception("Top module must be inside the 'sim' package.")

    experiment = ExperimentManager(experiment_name=args.xp_name, config_file=config_file, vcd_dir=vcd_dir)
    frames_per_packet = experiment.config.get_global_param("frames_per_packet", 1024)
    data_width = experiment.config.get_global_param("data_width", 64)
    max_packets = experiment.config.get_global_param("max_packets", 8)
    sys_freq = experiment.config.get_global_param("sys_freq", 50000000)
    rfic_freq = experiment.config.get_global_param("rfic_freq", 50000000)

    top = Top(frames_per_packet, data_width, max_packets)

    tb = Testbench(dut =  top, config = experiment.config)

    # Generate VCD filename
    experiment._create_vcd_dir()
    vcd_path = experiment.get_vcd_path()
    experiment.generate_report()
    print("Top created, starting simulation...")
    print(f"[VCD] Output will be saved to: {vcd_path}\n")

    gens = [
        stimulus(tb, test_name="all") 
    ]
    run_simulation(
        top, gens,
        clocks={"sys": 1e9/sys_freq, "rfic": 1e9/rfic_freq},    # sys clock period
        vcd_name=vcd_path                # Use generated VCD path
    )
    
    print(f"\n[VCD] Simulation complete. VCD saved to: {vcd_path}")
    
    # Open in GTKWave if requested
    if args.gtk:
        print(f"[VCD] Opening {vcd_path} in GTKWave...")
        subprocess.run(f"gtkwave {vcd_path} gtkwave_config.sav", shell=True)
    


if __name__ == "__main__":
    main()

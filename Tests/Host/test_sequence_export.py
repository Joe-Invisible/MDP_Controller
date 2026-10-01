"""Exercise the real exporter with GDB and synthetic RAM, without a probe.

Usage: python test_sequence_export.py --gdb PATH --elf PATH
All fixture exports remain in a temporary directory, outside acquisition logs.
"""
import argparse
import os
from pathlib import Path
import re
import struct
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gdb", required=True)
    parser.add_argument("--elf", required=True)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    elf = Path(args.elf).resolve()

    def gdb(*commands):
        command = [args.gdb, "--batch", "--nx", str(elf)]
        for expression in commands:
            command.extend(["-ex", expression])
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        return result.stdout

    # Resolve SRAM addresses from the supplied ELF rather than hard-coding them.
    expressions = [
        "&motionControllerSequenceTestPassed",
        "&motionControllerSequenceTestResultCount",
        "&motionControllerSequenceTestLogCount",
        "&motionControllerSequenceTestResults[0].commandAccepted",
    ]
    output = gdb(*(f"p/x (unsigned long)({expr})" for expr in expressions))
    addresses = [int(value, 16) for value in re.findall(r"\$\d+ = (0x[0-9a-f]+)", output)]
    assert len(addresses) == len(expressions), output
    ram_base = 0x20000000
    ram = bytearray(128 * 1024)
    for address, value in zip(addresses, [b"\x01", struct.pack("<I", 1), struct.pack("<I", 1), b"\x01"]):
        offset = address - ram_base
        assert 0 <= offset <= len(ram) - len(value), hex(address)
        ram[offset:offset + len(value)] = value

    # A minimal ARM ELF core supplies synthetic SRAM and registers. GDB needs
    # SP/PC to establish a context even for file-scoped static expressions.
    registers = bytearray(148)  # ARM Linux elf_prstatus, including 18 registers.
    struct.pack_into("<I", registers, 72 + 13 * 4, ram_base + len(ram) - 16)
    struct.pack_into("<I", registers, 72 + 15 * 4, 0x08000000)
    note = struct.pack("<III", 5, len(registers), 1) + b"CORE\0\0\0\0" + registers
    ident = b"\x7fELF" + bytes([1, 1, 1, 0]) + bytes(8)
    header = struct.pack("<16sHHIIIIIHHHHHH", ident, 4, 40, 1, 0, 52, 0, 0, 52, 32, 2, 0, 0, 0)
    note_program = struct.pack("<IIIIIIII", 4, 116, 0, 0, len(note), len(note), 0, 4)
    program = struct.pack("<IIIIIIII", 1, 116 + len(note), ram_base, ram_base, len(ram), len(ram), 6, 4)
    with tempfile.TemporaryDirectory(prefix="sequence-export-fixture-") as directory:
        root = Path(directory).resolve()
        core = root / "synthetic.core"
        core.write_bytes(header + note_program + program + note + ram)
        env = os.environ.copy()
        # Scope Git ownership trust to this known checkout in this test process.
        env.update(GIT_CONFIG_COUNT="1", GIT_CONFIG_KEY_0="safe.directory", GIT_CONFIG_VALUE_0=str(project))
        prepared = subprocess.run([
            "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
            str(project / "exp/gdb_scripts/prepare_sequence_export.ps1"),
            "-OutputDirectory", str(root),
        ], env=env, capture_output=True, text=True)
        if prepared.returncode:
            raise RuntimeError(prepared.stdout + prepared.stderr)
        script = root / "export-current.gdb"
        # The core has one synthetic result/sample so both array branches execute.
        output = gdb(f"core-file {core.as_posix()}", "set $mctrl_seq_battery_v = 0", f"source {script.as_posix()}")
        assert "EXPORTED:" in output, output
        exports = list(root.glob("mctrl_seq_*.txt"))
        assert len(exports) == 1, exports
        log = exports[0].read_text()
        assert log.rstrip().endswith("=== EXPORT COMPLETE ==="), log
        assert "No symbol" not in log and "Error in sourced" not in log, log
        assert re.search(r"=== Commands and controller configuration ===\s+\$\d+ = 1\s", log), log
        assert "useLegacyStraightSteering = false" in log, log
        assert "requestedDistanceMm =" in log, "Result array not exported"
        assert "commandTimeMs =" in log, "Sample array not exported"
    print("PASS: real GDB export with synthetic SRAM, array-derived count, result/sample arrays, complete marker; no probe or motion.")


if __name__ == "__main__":
    main()

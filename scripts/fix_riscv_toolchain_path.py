from pathlib import Path

Import("env")

toolchain_dir = env.PioPlatform().get_package_dir("toolchain-riscv32-esp")

if toolchain_dir:
    toolchain_dir = Path(toolchain_dir)
    root_bin = toolchain_dir / "bin"
    nested_bin = toolchain_dir / "riscv32-esp-elf" / "bin"

    if not root_bin.is_dir() and nested_bin.is_dir():
        env.PrependENVPath("PATH", str(nested_bin))

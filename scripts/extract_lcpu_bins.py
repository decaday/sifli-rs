"""
Extract LCPU firmware and patch binary data from SiFli SDK C source files.

Reads u32 arrays from C files, converts to little-endian bytes, and saves as .bin files.
Output goes to sifli-hal/data/sf32lb52x/lcpu/.

Usage:
    python scripts/extract_lcpu_bins.py

Input files (from contrib/sdk):
    - example/rom_bin/lcpu_general_ble_img/lcpu_52x.c  -> g_lcpu_bin
    - drivers/cmsis/sf32lb52x/lcpu_patch.c              -> g_lcpu_patch_list, g_lcpu_patch_bin
    - drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c        -> g_lcpu_patch_list, g_lcpu_patch_bin
"""

import re
import struct
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
SDK_ROOT = os.path.join(REPO_ROOT, "contrib", "sdk")
OUTPUT_DIR = os.path.join(REPO_ROOT, "sifli-hal", "data", "sf32lb52x", "lcpu")


def parse_c_u32_array(filepath, symbol):
    """Parse a C uint32 array by symbol name, return list of u32 values."""
    with open(filepath, "r") as f:
        content = f.read()

    pos = content.find(symbol)
    if pos < 0:
        raise ValueError(f"symbol `{symbol}` not found in {filepath}")

    brace_start = content.index("{", pos)
    brace_end = content.index("}", brace_start)
    inner = content[brace_start + 1 : brace_end]

    values = []
    for tok in re.split(r"[,\s]+", inner):
        tok = tok.strip().rstrip("uUlL")
        if not tok:
            continue
        if tok.startswith("0x") or tok.startswith("0X"):
            values.append(int(tok, 16))
        else:
            values.append(int(tok))
    return values


def words_to_le_bytes(words):
    """Convert list of u32 words to little-endian bytes."""
    return b"".join(struct.pack("<I", w) for w in words)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    files = {
        "lcpu_firmware.bin": (
            os.path.join(SDK_ROOT, "example/rom_bin/lcpu_general_ble_img/lcpu_52x.c"),
            "g_lcpu_bin",
        ),
        "patch_a3_list.bin": (
            os.path.join(SDK_ROOT, "drivers/cmsis/sf32lb52x/lcpu_patch.c"),
            "g_lcpu_patch_list",
        ),
        "patch_a3_bin.bin": (
            os.path.join(SDK_ROOT, "drivers/cmsis/sf32lb52x/lcpu_patch.c"),
            "g_lcpu_patch_bin",
        ),
        "patch_letter_list.bin": (
            os.path.join(SDK_ROOT, "drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c"),
            "g_lcpu_patch_list",
        ),
        "patch_letter_bin.bin": (
            os.path.join(SDK_ROOT, "drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c"),
            "g_lcpu_patch_bin",
        ),
    }

    for out_name, (c_path, symbol) in files.items():
        print(f"Extracting {symbol} from {os.path.relpath(c_path, REPO_ROOT)}")
        words = parse_c_u32_array(c_path, symbol)
        data = words_to_le_bytes(words)
        out_path = os.path.join(OUTPUT_DIR, out_name)
        with open(out_path, "wb") as f:
            f.write(data)
        print(f"  -> {os.path.relpath(out_path, REPO_ROOT)} ({len(data)} bytes)")

    print("\nDone.")


if __name__ == "__main__":
    main()

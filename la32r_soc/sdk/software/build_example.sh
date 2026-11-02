#!/usr/bin/env bash
set -euo pipefail
software_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
soc_root="$(cd "$software_dir/../.." && pwd)"
example="${1:-test_v18}"
if [[ ! "$example" =~ ^[A-Za-z0-9_]+$ || ! -f "$software_dir/examples/$example/Makefile" ]]; then
  echo "Unknown example: $example" >&2; exit 1
fi
compiler="$(command -v loongarch32r-linux-gnusf-gcc || true)"
if [[ -z "$compiler" ]]; then
  echo "Run sdk/toolchains/init.sh in WSL and load ~/.soc-course-env first." >&2; exit 1
fi
gcc_dir="$(cd "$(dirname "$compiler")/.." && pwd)"
export LA32RSOC_WINDOWS_HOME="$soc_root"
make -C "$software_dir/examples/$example" -B GCC_DIR="$gcc_dir" PICOLIBC_DIR="$soc_root/sdk/toolchains/picolibc"
echo "Published selected firmware to $soc_root/sdk/axi_ram.mif"
echo "Rebuild the FPGA bitstream after changing the embedded BRAM firmware."

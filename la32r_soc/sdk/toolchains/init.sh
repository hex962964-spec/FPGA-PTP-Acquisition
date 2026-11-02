#!/usr/bin/env bash
set -euo pipefail
toolchains_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
gcc_dir="$toolchains_dir/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0"
if [[ ! -x "$gcc_dir/bin/loongarch32r-linux-gnusf-gcc" ]]; then
  archive="$toolchains_dir/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0.tar.xz"
  [[ -f "$archive" ]] || wget -O "$archive" https://gitee.com/loongson-edu/la32r-toolchains/releases/download/v0.0.3/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0.tar.xz
  tar -xJf "$archive" -C "$toolchains_dir"
fi
for lib in picolibc newlib; do
  if [[ ! -f "$toolchains_dir/$lib/lib/libc.a" ]]; then
    mkdir -p "$toolchains_dir/$lib"
    if [[ "$lib" == picolibc ]]; then
      url=https://gitee.com/ffshff/la32r-picolibc/releases/download/V1.1/picolibc.tar.gz
    else
      url=https://gitee.com/ffshff/newlib-la32r/releases/download/V1.1/newlib.tar.gz
    fi
    archive="$toolchains_dir/$lib.tar.gz"
    [[ -f "$archive" ]] || wget -O "$archive" "$url"
    tar -xzf "$archive" -C "$toolchains_dir/$lib"
  fi
done
soc_root="$(cd "$toolchains_dir/../.." && pwd)"
windows_root="${LA32RSOC_WINDOWS_HOME:-$soc_root}"
if [[ ! -d "$windows_root/rtl" || ! -d "$windows_root/sdk" ]]; then
  echo "Invalid LA32RSOC_WINDOWS_HOME: $windows_root. Set it to the Windows SoC root mounted under /mnt/." >&2
  exit 1
fi
env_file="$HOME/.soc-course-env"
[[ ! -f "$env_file" ]] || cp -p "$env_file" "$env_file.before-config-$(date +%Y%m%d-%H%M%S)"
printf 'export PATH=%q:$PATH\nexport LA32RSOC_WINDOWS_HOME=%q\n' "$gcc_dir/bin" "$windows_root" > "$env_file"
for startup in "$HOME/.bashrc" "$HOME/.profile"; do
  if ! grep -Fq '.soc-course-env' "$startup"; then
    printf '\n# SoC course environment\n[ -f "$HOME/.soc-course-env" ] && . "$HOME/.soc-course-env"\n' >> "$startup"
  fi
done
source "$env_file"
loongarch32r-linux-gnusf-gcc --version | head -1
printf 'Windows SoC root: %s\n' "$LA32RSOC_WINDOWS_HOME"

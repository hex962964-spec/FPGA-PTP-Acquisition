# License scope and third-party notices

## Project contributions

The root LICENSE contains the MIT License. It applies to original software and documentation contributed by the FPGA-PTP-Acquisition project contributors, to the extent they own the relevant copyright. This includes their original synchronization and acquisition logic, control software, host software, and project-specific build and documentation contributions.

Third-party code, pre-existing platform components, vendor IP and generated vendor artifacts are excluded from this MIT grant. Their own copyright notices, license files, file headers, and applicable upstream terms continue to apply. An existing file-specific or component-specific license takes precedence for that material. No third-party ownership is claimed and no additional rights in third-party material are granted here.

Raw experimental records and composite FPGA bitstreams are not represented as wholly MIT-licensed software. For bitstreams and other combined artifacts, the licenses and usage conditions of their constituent components remain applicable.

## Included component notices

| Component | Repository location | License information retained in this repository |
|---|---|---|
| OpenLA500 processor | `la32r_soc/rtl/ip/open-la500/` | Mulan Permissive Software License, Version 2 (MulanPSL-2.0); see the component's `LICENSE` |
| CNN IP | `la32r_soc/rtl/ip/CNN/` | MulanPSL-2.0; see the component's `LICENSE` |
| FFT IP | `la32r_soc/rtl/ip/FFT/` | MIT License, copyright (c) 2025 BriMonzZY; see the component's `LICENSE` |
| GNU/LoongArch toolchain distribution | `la32r_soc/sdk/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0.tar.xz` | Separate upstream distribution; the root MIT License does not relicense its contents. Follow the licenses and redistribution requirements of the individual compiler and toolchain components |
| Newlib and Picolibc materials | `la32r_soc/sdk/toolchains/newlib/` and `picolibc/` | Retained upstream and per-file notices, including BSD-style notices in supplied headers; no single root MIT relicense is asserted for these libraries or binary archives |
| AMD/Xilinx IP and generated products | PLL, clocking and other vendor IP configuration/output files under the two FPGA projects | Retained AMD/Xilinx file notices and applicable vendor/tool/IP terms; not newly licensed as MIT by this project |
| Other pre-existing platform/BSP material | Original platform, SDK/BSP, bundled source archives and other third-party files | Follow the retained file headers and applicable upstream terms; inclusion in this repository does not make such material project-original MIT code |

Preserve the original component LICENSE files, copyright notices, and any applicable attribution or redistribution requirements when copying or redistributing material. This inventory records the licenses explicitly found in the project; it is not a replacement for the upstream license text or a certification of every bundled dependency's redistribution rights.

## 中文说明

根目录MIT许可证适用于项目贡献者拥有著作权的自研代码及文档。第三方处理器、平台/BSP、工具链、C库、厂商IP和生成文件继续遵循各自原有许可或适用条款，不因仓库根目录新增MIT许可证而被重新授权。

OpenLA500和CNN保留木兰宽松许可证第2版，FFT保留原作者MIT许可证；工具链、运行库和AMD/Xilinx材料不在本项目的MIT授权范围内。未更改任何第三方原许可证或版权声明。

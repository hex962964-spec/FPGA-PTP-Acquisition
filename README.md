# 基于FPGA时敏同步采集

本仓库为“基于FPGA时敏同步采集”参赛作品的代码与复现资料。参考主板XC7A35T，同步采集板XC7A200T，电脑运行串口与UDP整合上位机。

## 主要入口
- la32r_soc/rtl：SoC及采集模块RTL，包括处理器、AXI、64位时钟、时间戳、跨时钟缓存、PTP及UDP通路。
- la32r_soc/sdk/software/examples/test_v17：当前控制程序、FAST50、异常延时保护、时间状态、链路恢复及单元测试；bsp为基础支持。
- la32r_soc/sdk/axi_ram.mif：当前程序RAM初始化镜像。
- PTP_Master/PTP_Master.srcs：参考主板RTL、IP配置与约束。
- tools/udp_quality_receiver、上位机v1.5.html：接收服务、测试及当前网页。启动整合上位机.cmd为现有入口，HTTP8775、UDP6000。需Python3；不要同时启动多个UDP6000接收实例。
- bittest：当前第十一组两板bit及控制程序归档。main_11.c为编号归档，程序内容沿用已验证的FAST50实现。
- evidence：内部时序报告、历史串口记录及2026-10-29真实Wireshark抓包。

## 打开工程
使用Vivado 2025.2。原生工程位于la32r_soc/fpga/project/Loongson_Soc.xpr及PTP_Master/PTP_Master.xpr。
在工程根目录执行：vivado -mode batch -source prepare_projects.tcl
该辅助脚本只更新本解压副本的RAM路径并移除未附带的综合检查点引用；不运行综合、不下载板卡。然后可用原生XPR继续构建。
当前SoC原生XPR已包含3份implementation-only/LATE的CDC Tcl约束。不要使用未带这些新增约束的旧create_project.tcl代替当前XPR。
SDK编译入口为test_v17/Makefile，使用包内Linux交叉工具链分发压缩包及picolibc等依赖；需要Linux或WSL环境，并按toolchains/README.md和BSP common.mk配置，未在本次整理中重新编译。

## 已有结果与边界
30秒150条记录的内部偏差−6～+12 ns；本次Wireshark20,000包序号无缺失、重复或倒退。
内部STA建立余量+0.041 ns、保持余量+0.048 ns；LUT20.39%、BRAM75.07%。
串口统计、内部STA及抓包各自证明不同环节，不能替代独立仪器物理同步精度测量。网口芯片与板级外部时序预算仍待验收；长期运行和故障恢复仍需同组文件进一步验证。
保留第三方源码原有作者与许可信息，不将处理器和基础平台声明为自主设计。

## 归档范围
保留当前工程源码、IP配置、约束、SDK/BSP、当前固件、主机软件、测试源码和关键证据；未收录多GB原始CSV录制、工作缓存、历史bit版本及无关旧示例。
交叉工具链以原有tar.xz分发包保留，省略其重复解包目录。manifest_sha256.csv记录逐文件校验值。
本次仅整理与复制，没有修改原工程，没有重新综合、烧录或启停设备服务。

## GitHub仓库说明

本仓库以提交材料中的当前完整工程副本为基础整理。保留原生Vivado工程、RTL、约束、IP配置、SDK/BSP、交叉工具链分发包、当前固件、上位机、测试与关键实验记录；排除Vivado运行目录、缓存、仿真生成目录和可重新生成的DCP检查点。

请先克隆或下载整个仓库，再按上文执行prepare_projects.tcl并打开原生工程。构建需要重新生成Vivado运行结果。比赛RAR归档作为报名附件单独提交，解压后的文件集合与本仓库一致（不含Git内部的.git目录）。

本项目自研部分采用 [MIT License](LICENSE)，允许按许可证条款使用、修改和再分发。第三方处理器、IP、基础平台和工具链继续遵循各自原有许可及适用条款，详见 [第三方许可与适用范围](THIRD_PARTY_NOTICES.md)。

manifest_sha256.csv记录本次仓库中各文件的SHA256，校验清单不包含自身。代码整理日期为2026年11月2日；本次上传未重新综合、编译或进行板卡测试。

## 2026年11月2日统一发布

报名RAR与GitHub采用同一份文件清单和工程配置，RAR内统一放在FPGA_TSN_Acquisition目录下。解压后与仓库比较时忽略这一层目录和Git内部的.git目录，各文件SHA256应一致。RAR为真实RAR5单卷归档，无密码，采用store方式，大小低于200 MB。

网页标题已与当前开发目录一致，功能代码未改动。RAR采用本仓库已整理的Vivado工程配置及PLL输出；打开工程前按上文运行prepare_projects.tcl，以更新本机RAM路径并移除未附带的检查点引用。本次统一未重新综合、编译或烧录板卡。

## 开源许可

项目贡献者拥有著作权的自研代码及文档采用MIT许可证，完整条款见根目录LICENSE。版权声明为2026 FPGA-PTP-Acquisition contributors，保留各贡献者实际享有的权利。

第三方组件的原有许可证、版权声明及适用条款保持不变；未将第三方代码、工具链、厂商IP或生成产物整体重新授权为MIT。原始实验记录和包含多方组件的bitstream也不表述为整体MIT软件。具体范围和已识别组件见THIRD_PARTY_NOTICES.md。

GitHub与比赛RAR同时包含LICENSE、THIRD_PARTY_NOTICES.md及更新后的文件校验清单，两者内容一致。本次只增加许可及说明，没有修改功能代码或第三方许可证。

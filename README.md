<p align="center">
<img  style="width:50%;"  alt="Logo" src="assets/main_logo.png">
<br>
<em>稳定 易用 精度</em>
<br>
</p>
<p align="center">
<a href="README_EN.md">English</a>
</p>

---

# 🚀 SDK 全新升级：更强大，更易用！

这次升级为 SDK 带来了更流畅的开发体验和更强大的功能：

✨ 精简依赖 – 降低编译开销，构建更快速。  
🤖 支持 ROS2 & Python – 轻松集成现代机器人与脚本化工作流。  
⏱ 更精准的同步机制 – 提供更高精度的时间协调。  
📡 数据协议更透明(JSON) – 通信更清晰、更灵活。  
⚙️ 配置更简单 – 轻松上手，自定义更便捷。  
📜 日志功能增强 – 记录更全面，调试更高效。   
🌐 多平台灵活部署 – (使用ZeroMQ)支持嵌入式/桌面/云端多场景部署。  
🔗 支持多相机 📷、雷达⦿ 、IMU 🧭 与单 GPS 🛰 的混合信号协同管理。  
🔄 [支持多同步板](./assets/board_introduction.md) -V3/V4/MINI。  
🛡️ 安全可靠 – 更加安全的电源与接线🚫。

立即升级，体验更高效、更友好的 SDK！🚀

# News

>1. Demo程序还在开发中，敬请期待！如果熟悉ZMQ，可以尝试了解一下SDK，并自己编写Demo程序。
>2. 开发者文档正在编写中，敬请期待！
>3. 关于第四代同步板还是希望尽可能非常稳定后再上架！硬件的稳定是我们一直以来的追求。

# 支持设备

>| 设备类型        | 品牌                          |同步方式 |
>|-------------|-----------------------------|--------|
>| 工业相机(网口)    | 海康/大华/大恒/京航/...             | PWM    |
>| 工业相机(USB)   | 海康/大华/大恒/京航/...             | PWM    |
>| 第三方IMU      | Xsense全系列/...               | PWM    |
>| 3D激光        | Mid360/Mid70/RoboSense系列/... | PPS   |
>| RTK/GPS     |                           | NMEA   |
>| 主机(ARM/X86) | Intel/AMD/Jetson/RockChip/... | PTP    |

# 开始使用
## 下载安装
```bash
sudo apt-get install libzmq3-dev # 安装ZeroMQ
git clone git@github.com:InfiniteSenseLab/InfiniteSenseSDK2.0.git -b main
cd InfiniteSenseSDK2.0
mkdir build && cd build
cmake..
```
## [同步板配置](./assets/board_config.md)
## [传感器接线](./assets/connection_config.md)
## [Demo运行](./assets/run_demo.md)
## [协议解析](./assets/data_info.md)

# 常见问题

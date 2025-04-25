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
🔄 兼容性保障 – 依旧支持旧版本(V3/MINI)。  
🌐 多平台灵活部署 – (使用ZeroMQ)支持嵌入式/桌面/云端多场景部署。  
🔗 支持多相机 📷、雷达 📡、IMU 🧭 与单 GPS 🛰 的混合信号协同管理。  

立即升级，体验更高效、更友好的 SDK！🚀

### 开始之前 (Before You Begin)
目前支持的设备类型如下：
>| 设备类型        | 品牌                          |同步方式 |
>|-------------|-----------------------------|--------|
>| 工业相机(网口)    | 海康/大华/大恒/京航/...             | PWM    |
>| 工业相机(USB)   | 海康/大华/大恒/京航/...             | PWM    |
>| 第三方IMU      | Xsense全系列/...               | PWM    |
>| 3D激光        | Mid360/Mid70/RoboSense系列/... | PPS   |
>| RTK/GPS     |                           | NMEA   |
>| 主机(ARM/X86) | Intel/AMD/Jetson/RockChip/... | PTP    |

# 开始使用
1. 下载SDK源码,编译安装
```angular2html
    git clone git@github.com:InfiniteSenseLab/InfiniteSenseSDK2.0.git -b main
    sudo apt-get install libzmq3-dev # 安装ZeroMQ依赖
    cd InfiniteSenseSDK2.0
    mkdir build && cd build
    cmake..
```
2. [如何接线](./assets/connection_config.md)
3. [配置设备](./assets/firmware_config.md)
4. 运行Demo

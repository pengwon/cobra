# COBRA SCOPE

[English](README_en.md)

## 简介

这是一个基于红外测温成像的开源项目，旨在帮助工程师在调试过程中发现和解决异常。该项目通过热成像来发现系统中的热问题，辅助检测和诊断电路问题，从而提高系统的稳定性和性能。

## 特性

- **红外测温成像**：利用红外技术，可以在无需接触的情况下测量对象的温度，从而减少了对设备的干扰。
- **实时监控**：该项目支持实时监控，可以即时发现和解决问题。
- **易于使用**：我们提供了详细的使用说明和示例，使得即使是没有相关经验的用户也能够快速上手。
- **开源**：该项目是完全开源的，欢迎任何人参与和贡献。

当前的版本基于MLX90640传感器，支持Windows和Linux系统。

MLX90640是一个完全校准的32x24像素热红外阵列，采用行业标准的4引脚TO39封装，并带有数字接口。MLX90640包含768个FIR像素。集成了一个环境传感器用于测量芯片的环境温度，以及一个供应传感器用于测量VDD。所有传感器的输出，包括IR、Ta和VDD，都存储在内部RAM中，并可以通过I2C进行访问。

很多电路故障都会表现出热异常，通常情况下就是温度异常，所以我们可以通过探测温度变化和异常来寻找故障点。同时，电路的正常工作环境在相当程度上就是热环境，我们也可以通过持续监测电路板和环境温度来大致判断工作环境的稳定性以及对电路板的影响。使用MLX90640红外阵列传感器作为"眼睛"来寻找电路板上的热点。这个传感器的高像素密度和集成的环境温度测量功能使得它非常适合于我们的应用场景。

MLX90640特点：
- 小尺寸，低成本的32x24像素红外阵列
- 易于集成
- 行业标准的四引脚TO39封装
- 工厂校准
- 噪声等效温度差 (NETD) 在1Hz刷新率下为0.1K RMS
- 兼容I2C的数字接口
- 可编程刷新率 0.5Hz…64Hz
- 3.3V供电电压
- 电流消耗小于23mA
- 2个视场角选项 - 55°x35° 和 110°x75°
- 工作温度 -40°C 到 85°C
- 目标温度 -40°C 到 300°C
- 符合RoHS规定

测温范围是很广的，甚至可以记录电路板元器件焊接过程。

![MLX90640内部框图](https://imgs.boringhex.top/blog/20240611155729.png)

## 原理图

https://www.bilibili.com/video/BV1zJ4m1M7GH/

<iframe src="//player.bilibili.com/player.html?bvid=BV1zJ4m1M7GH&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

## PCB

https://www.bilibili.com/video/BV1Y7421R7ZX/

<iframe src="//player.bilibili.com/player.html?bvid=BV1Y7421R7ZX&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

用集成器件信息的库真的很方便，加上ActiveBOM加持，这次画完PCB，导出整理bom只用了不到5分钟，直接发给jlc打样，非常流畅。

![ActiveBOM](https://imgs.boringhex.top/blog/20240611182124.png)

## 外壳

https://www.bilibili.com/video/BV1s4421Q7gz/

<iframe src="//player.bilibili.com/player.html?bvid=BV1s4421Q7gz&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

外壳设计上做了三脚架固定螺孔，这个大多数三脚架和手机支架都带，可以将传感器固定在支架上持续监测。

## 成品

![成品](https://imgs.boringhex.top/blog/90b911a07356ab261847dd5fdb55f75.jpg)

## 快速开始

首先，将仓库`clone`到本地：

```bash
git clone https://github.com/pengwon/cobra.git
```

然后，进入到`sw`目录：

```bash
cd cobra/sw
```

### `python` Demo：

Matplotlib:

```bash
python python_sdk/demo.py
```

OpenCV:

```bash
python python_sdk/demo_cv.py
```

## 贡献

如果你有任何想法或建议，或者想要参与到这个项目中来，欢迎查看我们的[贡献指南](CONTRIBUTING.md)。

## 许可证

该项目采用[MIT许可证](LICENSE)。


# COBRA SCOPE

[中文](README.md)

## Introduction

This is an open-source project based on infrared temperature imaging, aimed at helping engineers discover and resolve anomalies during the debugging process. The project uses thermal imaging to detect heat issues in the system, assisting in the detection and diagnosis of circuit problems, thereby improving system stability and performance.

## Features

- **Infrared Temperature Imaging**: Utilizing infrared technology, the temperature of objects can be measured without the need for contact, reducing interference with the device.
- **Real-time Monitoring**: This project supports real-time monitoring, allowing for immediate discovery and resolution of issues.
- **Easy to Use**: We provide detailed instructions and examples, enabling users without relevant experience to quickly get started.
- **Open Source**: This project is completely open source, and we welcome anyone to participate and contribute.

The current version is based on the MLX90640 sensor and supports Windows and Linux systems.

The MLX90640 is a fully calibrated 32x24 pixel thermal infrared array in an industry-standard 4-pin TO39 package with a digital interface. The MLX90640 contains 768 FIR pixels. It integrates an environmental sensor for measuring the ambient temperature of the chip, and a supply sensor for measuring VDD. The outputs of all sensors, including IR, Ta, and VDD, are stored in internal RAM and can be accessed via I2C.

Many circuit faults will exhibit thermal anomalies, usually temperature anomalies, so we can find fault points by detecting temperature changes and anomalies. At the same time, the normal working environment of the circuit is largely a thermal environment. We can also roughly judge the stability of the working environment and its impact on the circuit board by continuously monitoring the temperature of the circuit board and the environment. The MLX90640 infrared array sensor is used as the "eye" to find hot spots on the circuit board. The high pixel density of this sensor and its integrated ambient temperature measurement function make it very suitable for our application scenarios.

MLX90640 features:
- Small size, low-cost 32x24 pixel infrared array
- Easy to integrate
- Industry-standard four-pin TO39 package
- Factory calibrated
- Noise Equivalent Temperature Difference (NETD) of 0.1K RMS at a 1Hz refresh rate
- I2C compatible digital interface
- Programmable refresh rate 0.5Hz…64Hz
- 3.3V supply voltage
- Current consumption less than 23mA
- 2 field of view options - 55°x35° and 110°x75°
- Operating temperature -40°C to 85°C
- Target temperature -40°C to 300°C
- Complies with RoHS regulations

The temperature measurement range is very wide, and it can even record the soldering process of circuit board components.

![MLX90640 Internal Diagram](https://imgs.boringhex.top/blog/20240611155729.png)

## Schematic

https://www.bilibili.com/video/BV1zJ4m1M7GH/

<iframe src="//player.bilibili.com/player.html?bvid=BV1zJ4m1M7GH&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

## PCB

https://www.bilibili.com/video/BV1Y7421R7ZX/

<iframe src="//player.bilibili.com/player.html?bvid=BV1Y7421R7ZX&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

Using a library with integrated component information is really convenient. With the help of ActiveBOM, it took less than 5 minutes to draw the PCB and export and organize the BOM. It was sent directly to JLC for prototyping, which was very smooth.

![ActiveBOM](https://imgs.boringhex.top/blog/20240611182124.png)

## Enclosure

https://www.bilibili.com/video/BV1s4421Q7gz/

<iframe src="//player.bilibili.com/player.html?bvid=BV1s4421Q7gz&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

The enclosure design includes a tripod mounting hole. Most tripods and phone holders come with this, allowing the sensor to be fixed on the stand for continuous monitoring.

## Finished Product

![Finished Product](https://imgs.boringhex.top/blog/90b911a07356ab261847dd5fdb55f75.jpg)

## Quick Start

First, clone the repository to your local machine:

```bash
git clone https://github.com/pengwon/cobra.git
```

Then, navigate to the `sw` directory:

```bash
cd cobra/sw
```

### Python Demo:

Matplotlib:

```bash
python python_sdk/demo.py
```

OpenCV:

```bash
python python_sdk/demo_cv.py
```

## Contributing

If you have any ideas or suggestions, or want to participate in this project, please check out our [contribution guide](CONTRIBUTING.md).

## License

This project is licensed under the [MIT License](LICENSE).
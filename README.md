## OpenDLV Microservice to interface with VelodyneLidar VLP16 units

This repository provides source code to interface with a VelodyneLidar VLP16
unit for the OpenDLV software ecosystem.

[![Build Status](https://travis-ci.org/chalmers-revere/opendlv-device-lidar-vlp16.svg?branch=master)](https://travis-ci.org/chalmers-revere/opendlv-device-lidar-vlp16) [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)


## Table of Contents
* [Dependencies](#dependencies)
* [Usage](#usage)
* [Build from sources on the example of Ubuntu 16.04 LTS](#build-from-sources-on-the-example-of-ubuntu-1604-lts)
* [License](#license)
* [PointCloudReading data structure](#pointcloudreading-data-structure)


## Dependencies
No dependencies! You just need a C++14-compliant compiler to compile this
project as it ships the following dependencies as part of the source distribution:

* [libcluon](https://github.com/chrberger/libcluon) - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)
* [Unit Test Framework Catch2](https://github.com/catchorg/Catch2/releases/tag/v2.1.2) - [![License: Boost Software License v1.0](https://img.shields.io/badge/License-Boost%20v1-blue.svg)](http://www.boost.org/LICENSE_1_0.txt)


## Usage
This microservice is created automatically on changes to this repository via Docker's public registry for:
* [x86_64](https://hub.docker.com/r/chalmersrevere/opendlv-device-lidar-vlp16-amd64/tags/)
* [armhf](https://hub.docker.com/r/chalmersrevere/opendlv-device-lidar-vlp16-armhf/tags/)
* [aarch64](https://hub.docker.com/r/chalmersrevere/opendlv-device-lidar-vlp16-aarch64/tags/)

To run this microservice using our pre-built Docker multi-arch images to connect
to a VelodyneLidar VLP16 unit broadcasting data to `0.0.0.0:2368` and to publish
the messages according to OpenDLV Standard Message Set into session 111 in
Google Protobuf format, simply start it as follows:

```
docker run --init --rm --net=host chalmersrevere/opendlv-device-lidar-vlp16-multi:v0.0.9 --vlp16_ip=0.0.0.0 --vlp16_port=2368 --cid=111 --verbose
```

## Build from sources on the example of Ubuntu 16.04 LTS
To build this software, you need cmake, C++14 or newer, and make. Having these
preconditions, just run `cmake` and `make` as follows:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make && make test && make install
```


## License

* This project is released under the terms of the GNU GPLv3 License



## PointCloudReading data structure

`opendlv-device-lidar-vlp16` receives the data from VLP16 (Velodyne LiDAR with 16 layers)
UDP packets as input and tranforms the payload into a more compact [PointCloudReading](https://github.com/chalmers-revere/opendlv.standard-message-set/blob/master/opendlv.odvd#L152-L158)
representation (CPC), which is a compact form of the original 3D point cloud.
Further details of this implementation can be found in the paper
"Hang Yin and Christian Berger, Mastering data complexity for autonomous driving with adaptive point clouds for urban environments, 2017 IEEE Intelligent Vehicles Symposium, 2017 (https://www.researchgate.net/publication/318093493_Mastering_Data_Complexity_for_Autonomous_Driving_with_Adaptive_Point_Clouds_for_Urban_Environments)".

Using CPC, it is possible to encode a complete scan of VPL-16 (Velodyne LiDAR with 16 layers)
into a single UDP packet, assuming 10Hz rotation rate. For every 16 points with the same azimuth,
they are re-ordered with increasing vertical angle. The top layer has a vertical angle of -30.67
degree. The bottom layer has a vertical angle of 10.67 degree. From the top layer all the way down
to the bottom layer, the vertical angle increment alternates between 1.33 and 1.34 degree. For
instance, the vertical angle of the top layer, Layer 0, is -30.67 degree, while the vertical
angles of Layer 1 and Layer 2 are -29.33 and -28 degree, respectively.

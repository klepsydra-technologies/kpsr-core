<p align="right">
  <img width="25%" height="25%"src="./images/klepsydra_logo.jpg">
</p>

[![Build](https://github.com/klepsydra-technologies/kpsr-core/actions/workflows/push.yml/badge.svg)](https://github.com/klepsydra-technologies/kpsr-core/actions/workflows/push.yml) [![codecov](https://codecov.io/gh/klepsydra-technologies/kpsr-core/branch/main/graph/badge.svg?token=ZP2NHPkCrU)](https://codecov.io/gh/klepsydra-technologies/kpsr-core)
# Installation Instructions

## System dependencies

* Ubuntu 14.04 or above
* Cmake 3.5.1 or above
* gcc for C++11 5.4.0 or above.
* Python 3.6 or above
* ROS Indigo or above (optional)
* ZMQ 3 or above (optional)
* DDS (optional)
* Doxygen (optional)
* Moxygen (<https://github.com/sourcey/moxygen>) (optional)

### Dependencies installed by kpsr-core

* Google Test (<https://github.com/klepsydra-technologies/googletest>)
* ConcurrentQueue (<https://github.com/klepsydra-technologies/concurrentqueue>)
* Cereal (<https://github.com/klepsydra-technologies/cereal>)
* Spdlog (<https://github.com/klepsydra-technologies/spdlog>)

## System requirements

```bash
sudo apt-get install build-essential
sudo apt-get install git
sudo apt-get install cmake
sudo apt-get install python3-pip
```

### Yaml-cpp

If this software package is not available in the system (it is shipped with some software, e.g. ROS), dowload it and install it from <https://github.com/jbeder/yaml-cpp>. **Please make sure it is installed in a share library mode**.

## Installation

Given ```$KLEPSYDRA_HOME```, for example ```$HOME/klepsydra```:

```bash
cd $KLEPSYDRA_HOME
git clone https://github.com/klepsydra-technologies/kpsr-core
cd kpsr-core
git submodule update --init
mkdir build
cd build
cmake ..
make
make test
sudo make install
```

This will install the klespydra core in default locations:

* `/usr/local/include/klepsydra` for kpsr-core include files
* `/usr/local/include/kpsr_3parties` for third party dependencies necessary for Klepsydra package
* `/usr/local/lib/` for kpsr-core libraries

The cmake has the following options:

* -DKPSR_WITH_YAML=true: required when used within **ROS** or when in conjunction with -DKPSR_WITH_DDS or -DKPSR_WITH_ZMQ
* -DCMAKE_INSTALL_PREFIX for specifying the Klepsydra installation location (/usr/local by default)
* -DKPSR_WITH_DOXYGEN to allow generation of documentation
* -DGTEST_PATH for the google test path (default is ./thirdparties/googletest)
* -DTHIRDPARTIES_PATH for the ConcurrentQueue and Cereal path (default is ./thirdparties)
* -DKPSR_WITH_DDS=true for building the DDS binding
* -DKPSR_WITH_ZMQ=true for building the ZeroMQ binding.
* -DKPSR_TEST_PERFORMANCE=true for building the performance tests
* -DKPSR_WITH_SOCKET for building SOCKET binding
* -DKPSR_ENABLE_EXAMPLES=true for building examples files (default false)
* -DKPSR_ENABLE_TESTS=true for building unit tests (default true)


Example

```bash
cmake -DKPSR_WITH_SOCKET=true ..
```

### ROS installation

We assume all ros environment variables are properly set up. Also kpsr-core must be installed first.
Add the Klepsydra Core's ros projects to the ROS workspace:

```bash
cd YOUR_ROS_WORKSPACE/src
ln -s PATH_TO_KPSR_CORE/serialization/modules/ros_mdlw/kpsr_ros_serialization
ln -s PATH_TO_KPSR_CORE/core/modules/ros_mdlw/kpsr_ros_core
```

Next compile and install the ros projects so that they are available in your ros workspace for future projects.

```bash
cd ..
catkin_make install
```

## Documentation

### Documentation generation

```bash
make doc
```

### Location of documentation

The last built documentation is available in [Klepsydra Robotics API DOC](./api-doc/)

# License

&copy; Copyright 2023, Klepsydra Technologies AG, all rights reserved. Licensed under the terms in [LICENSE.md](./LICENSE.md)

This software and documentation are Copyright 2023, Klepsydra Technologies AG
Limited and its licensees. All rights reserved. See [license file](./LICENSE.md) for full copyright notice and license terms.

# Contact

<https://www.klepsydra.com>
support@klepsydra.com

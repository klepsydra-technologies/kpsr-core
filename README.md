<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# Installation Instructions

## System dependencies

* Ubuntu 14.04 or above
* ConcurrentQueue (https://github.com/klepsydra-technologies/concurrentqueue)
* Cereal (https://github.com/klepsydra-technologies/cereal)
* ROS Indigo or above (optional)
* ZMQ 3 or above (optional)
* DDS (optional)
* Cmake 3.5.1 or above
* gcc for C++11 5.4.0 or above.
* Doxygen (optional)
* Moxygen (https://github.com/sourcey/moxygen) (optional)

## Klepsydra dependencies

* kpsr-serialization

## System installation

	sudo apt install build-essentials
	sudo apt install git
	sudo apt install cmake
	git clone https://github.com/google/googletest.git

### Google Test

Google-Tests has to be at the level of the part of the folder where Klepsydra core will be installed. For example, if Klepsydra is installed in:
	$KLEPSYDRA_HOME/kpsr-core

Then google tests has to be installed in:
	$KLEPSYDRA_HOME/../googletest

These locations can be overridden by including the variables `GTEST_PATH` and `THIRDPARTIES_PATH` in the kpsr-core cmake invocation.

### Cereal
	git clone https://github.com/klepsydra-technologies/cereal
	sudo mkdir $THIRDPARTIES_PATH/include
	sudo cp cereal/include/* $THIRDPARTIES_PATH/include

By default, we choose the location of installation for Cereal to be /opt/klepsydra/thirdparties.
This location can be overridden by including the variable

	THIRDPARTIES_PATH

### Concurrent queue

	git clone https://github.com/klepsydra-technologies/concurrentqueue
	sudo mkdir $THIRDPARTIES_PATH/include
	sudo cp concurrentqueue/*.h $THIRDPARTIES_PATH/include

By default, we choose the location of installation for ConcurrentQueue to be /opt/klepsydra/thirdparties.
This location can be overridden by including the variable

	THIRDPARTIES_PATH

### Yaml-cpp

If this software package is not available in the system (it is shipped with some software, e.g. ROS).

Clone and install YAML-CPP:

	git clone https://github.com/jbeder/yaml-cpp
	cd yaml-cpp
	mkdir build
	cd build
	cmake -DBUILD_SHARED_LIBS=ON ..
	make
	sudo make install

## Installation

Given ```$KLEPSYDRA_HOME```, for example ```$HOME/klepsydra```:

```
cd $KLEPSYDRA_HOME
git clone https://github.com/klepsydra-technologies/kpsr-core
cd kpsr-core
mkdir build
cd build
cmake ..
make
make test
sudo make install
```

This will install the klespydra core in

	/opt/klepsydra

The cmake has the following options:

* -DCMAKE_INSTALL_PREFIX for specifying the Klepsydra installation location (/opt/klepsydra by default)
* -DKPSR_BUILD_UTILS_PATH location of the ```kpsr_build``` repository (../build_utils by default).
* -DKPSR_WITH_DOXYGEN to allow generation of documentation
* -DGTEST_PATH for the google test path (default is ../../googletest)
* -DTHIRDPARTIES_PATH for the ConcurrentQueue and Cereal path (default is /opt/klepsydra/thirdparties)
* -DKPSR_WITH_DDS=true for building the DDS binding
* -DKPSR_WITH_ZMQ=true for building the ZeroMQ binding.
* -DKPSR_TEST_PERFORMANCE=true for building the performance tests
* -DKPSR_WITH_SOCKET for building SOCKET binding
* -DKPSR_WITH_YAML **FIXME**: required in conjunction with -DKPSR_WITH_DDS

Example

```
cmake -DKPSR_WITH_SOCKET=true ..
```


### ROS installation

In case of a new project:

```
mkdir -p YOUR_ROS_PROJECT
cd YOUR_ROS_PROJECT
source /opt/ros/melodic/setup.bash
catkin_init_workspace
```

Then add the Klepsydra ROS Core project to the ROS project:

```
cd YOUR_ROS_PROJECT
ln -s ../../core_utils/kpsr-core/modules/ros_mdlw/kpsr_ros_core
```

## Documentation

### Documentation generation

```
make doc
```

### Location of documentation

The last built documentation is available in https://github.com/klepsydra-technologies/kpsr-api-documentation

#  License

&copy; Copyright 2019-2020, Klepsydra Technologies, all rights reserved. Licensed under the terms in [LICENSE.md](./LICENSE.md)

This software and documentation are Copyright 2019-2020, Klepsydra Technologies
Limited and its licensees. All rights reserved. See [license file](./LICENSE.md) for full copyright notice and license terms.

#  Contact

https://www.klepsydra.com
support@klepsydra.com


ARG tag='ZMQ'
FROM kpsr-thirdparties:$tag as thirdparties

FROM ubuntu:18.04 AS builder

ENV THIRDPARTIES_PATH /opt/klepsydra/thirdparties/
# System Dependencies.

RUN apt update && apt-get install libssl-dev libcurl4-gnutls-dev build-essential git cmake python cppcheck doxygen python3-pip gcovr lcov curl -y --fix-missing
RUN curl -sL https://deb.nodesource.com/setup_10.x -o nodesource_setup.sh && bash nodesource_setup.sh && apt install nodejs

# Internal dependencies
COPY --from=thirdparties /opt/klepsydra/thirdparties/ $THIRDPARTIES_PATH
COPY --from=thirdparties /usr/local/include/yaml-cpp/ /usr/local/include/yaml-cpp/
COPY --from=thirdparties /usr/local/lib/cmake/yaml-cpp/ /usr/local/lib/cmake/yaml-cpp/
COPY --from=thirdparties /usr/local/lib/libyaml* /usr/local/lib/

# Install pip packages (Optimizing cache building)
# RUN pip3 install lcov_cobertura nose coverage
RUN npm install -g moxygen

WORKDIR /opt

# Klepsydra
COPY . kpsr-core

RUN cd kpsr-core \
    && git submodule update --init \
    && mkdir build \
    && cd build \
    && cmake -DKPSR_WITH_DOXYGEN=true -DKPSR_WITH_DDS=false -DKPSR_WITH_ZMQ=true\
       -DKPSR_TEST_PERFORMANCE=true -DKPSR_WITH_SOCKET=true -DKPSR_WITH_YAML=true\
       -DKPSR_WITH_CODE_METRICS=true -DCMAKE_PREFIX_PATH=/opt/klepsydra/thirdparties\
       -DCMAKE_BUILD_TYPE=Debug ..\
    && make \
    && bash ../kpsr-build/scripts/make_cppcheck.sh \
    && make test_coverage_cobertura ARGS="-V" \
    && make doc \
    && make install

ARG BUILD_ID
LABEL kpsr-core=builder
LABEL BUILD_ID=${BUILD_ID}

FROM ubuntu:18.04

COPY --from=builder /opt/klepsydra/ /opt/klepsydra/
WORKDIR /opt/

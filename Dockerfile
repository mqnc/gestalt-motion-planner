
ARG UBUNTU_VERSION=22.04

FROM ubuntu:20.04 AS base-20.04

# for adding repositories
RUN apt update
RUN apt install -y software-properties-common ca-certificates gnupg curl

# install recent compiler
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt update
RUN apt install -y gcc-11 gcc-11-base gcc-11-doc g++-11 \
  libstdc++-11-dev libstdc++-11-doc
ENV CXX="g++-11"
ENV CC="gcc-11"

# install recent cmake
RUN mkdir -p /etc/apt/keyrings && \
  curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc \
  | gpg --dearmor -o /etc/apt/keyrings/kitware.gpg && \
  echo "deb [signed-by=/etc/apt/keyrings/kitware.gpg] https://apt.kitware.com/ubuntu/ focal main" \
  > /etc/apt/sources.list.d/kitware.list && \
  apt-get update && \
  apt install -y cmake

FROM ubuntu:22.04 AS base-22.04

RUN apt update && apt install -y build-essential cmake

FROM ubuntu:24.04 AS base-24.04

RUN apt update && apt install -y build-essential cmake

FROM base-${UBUNTU_VERSION} AS deps

# install OMPL dependencies
RUN apt update && apt install -y \
  libboost-serialization-dev \
  libboost-filesystem-dev \
  # libboost-system-dev \
  libboost-program-options-dev \
  libboost-test-dev

FROM deps AS cpp_lib

# build planner lib
WORKDIR /app
COPY ./src /app/src
RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ../src/planner && make -j4

FROM scratch AS cpp_export

COPY --from=cpp_lib "/app/build/libgestaltplanner.a" .

FROM deps AS cpp_test

# test planner
# instead of using the lib the tests build from source again for easier debugging
WORKDIR /app/build/release/test
COPY ./src /app/src
RUN cmake -DCMAKE_BUILD_TYPE=Release ../../../src/test && make -j4
COPY ./models /app/models
CMD ["ctest", "--output-on-failure"]

FROM deps AS python_module

# build and install python planner
WORKDIR /app
RUN apt install -y python3-pip python3-venv
RUN python3 -m venv /venv
ENV PATH="/venv/bin:$PATH"
RUN pip3 install build
COPY ./src /app/src
COPY ./python /app/python
COPY ./pyproject.toml /app/pyproject.toml
COPY ./readme.md /app/readme.md
RUN MAKEFLAGS="-j4" python3 -m build

FROM scratch AS python_export

COPY --from=python_module "/app/dist" .

FROM python_module AS python_test

RUN cd /app/dist && pip3 install *.whl
COPY ./models /app/models
CMD ["python3", "python/demo.py"]


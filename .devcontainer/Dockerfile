ARG platform=arm64

# Cross-build for Raspberrypi 5
FROM --platform=${platform} ubuntu:noble

RUN apt-get update && apt-get install -y git build-essential cmake clangd clang-format clang-tidy rpm g++-11

WORKDIR /deps

RUN git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc

RUN mkdir /deps/grpc/build && cd /deps/grpc/build && cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      .. && make -j8 install

RUN git clone https://github.com/Marcus-Forte/Livox-SDK.git && git clone https://github.com/Livox-SDK/Livox-SDK2.git

# /deps/Livox-SDK/build already exists from repo.
RUN cd /deps/Livox-SDK/build && CXX=g++-11 cmake -DCMAKE_BUILD_TYPE=Release .. && \
      make -j8 install

RUN mkdir /deps/Livox-SDK2/build && cd /deps/Livox-SDK2/build && CXX=g++-11 cmake -DCMAKE_BUILD_TYPE=Release .. && \
            make -j8 install

RUN rm -rf /var/lib/apt/lists/* /deps
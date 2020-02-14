# 
# You can build a mock development enviornment from this Dockerfile.
#
# Note: Docker can't support running aarch container on x64 platform so this 
# environment can only used to validate compile-time correctness.
#
# You may also need to connect development container, here is a initialization 
# sample:
#
#   > service ssh start
#   > useradd -m -d /home/<user-name> -s /bin/bash -G sudo <user-name>
#   > passwd <user-name>
#
# If you connect the development container through IDEs support CMake. You may 
# need these parameters:
#
#   Vcpkg toolchain path: /tmp/vcpkg/scripts/buildsystems/vcpkg.cmake
#   CMake path: /usr/local/bin/cmake
#   CMake arguments: 
#       -DCMAKE_PREFIX_PATH="/opt/MVS/include;/opt/MVS/lib/64"
#
# docker run -itd --name ossian-dev -p 20022:22 ossian-dev-env /bin/bash
#


FROM ubuntu:18.04 as build
 
LABEL description="Ossian development container"
 
RUN apt update \ 
    && apt install -y curl file gcc g++ git build-essential tar unzip wget ninja-build yasm openssh-server rsync

RUN mkdir -p /var/run/sshd

RUN echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config && \
    ssh-keygen -A

RUN cd /tmp \
    && wget "http://download.hikvision.com/UploadFile/Soft/MVS/01%20Machine%20Vision/02%20Service%20Support/01%20Clients/MVS_STD_V2.0.0_191126(Linux%20X86).zip" \
    && unzip "MVS_STD_V2.0.0_191126(Linux X86).zip" \
    && dpkg -i MVS-2.0.0_x86_64_20191126.deb

RUN cd /tmp \
    && wget https://github.com/microsoft/CMake/releases/download/v3.16.3311152/cmake-3.16.3311152-MSVC_2-Linux-x64.sh \
    && chmod +x cmake-3.16.3311152-MSVC_2-Linux-x64.sh \
    && ./cmake-3.16.3311152-MSVC_2-Linux-x64.sh --prefix=/usr/local --skip-license \
    && rm cmake-3.16.3311152-MSVC_2-Linux-x64.sh
 
RUN cd /tmp \
    && git clone https://github.com/Microsoft/vcpkg.git -n \ 
    && cd vcpkg \
    && git checkout bc37190216256125bc7227456d5983809fec7373 \
    && ./bootstrap-vcpkg.sh -useSystemBinaries

# opencv installing begin
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install libjpeg-turbo:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install liblzma:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install zlib:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install libpng:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install libwebp[nearlossless,simd]:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install opengl:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install protobuf:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install szip:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install tiff:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install eigen3:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install ffmpeg[avresample]:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install hdf5[szip,zlib]:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install opencv[nonfree,contrib,eigen,ffmpeg]:x64-linux
# opencv installing end

RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install protobuf:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install fmt:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install spdlog:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install gtest:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 ./tmp/vcpkg/vcpkg install benchmark:x64-linux

RUN cd /tmp \
    && git clone https://github.com/microsoft/mimalloc.git -n \
    && cd mimalloc \
    && git checkout 04f1c3b1e23677ce03bd16137e73089abd552175 \
    && mkdir -p out/release \
    && cd out/release \
    && cmake ../.. \
    && make -j4 \
    && make install \
    && cd ../.. \
    && mkdir -p out/debug \
    && cd out/debug \
    && cmake -DCMAKE_BUILD_TYPE=Debug ../.. \
    && make -j4

EXPOSE 22

CMD ["bin/bash"]
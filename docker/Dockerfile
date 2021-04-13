
FROM ubuntu:18.04 as build
 
LABEL description="Ossian development container"

ENV dir_prefix=/usr/local/Ossian

RUN apt update \ 
    && apt install -y curl file gcc g++ gdb git build-essential tar unzip wget ninja-build yasm openssh-server rsync npm software-properties-common \
    && apt update \
    && add-apt-repository ppa:ubuntu-toolchain-r/test -y \
    && apt install -y gcc-10 g++-10
    
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 \
    && update-alternatives --config gcc \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100 \
    && update-alternatives --config g++

RUN npm install -g npm@latest-6 \
    && npm install -g n \
    && n stable

RUN npm install -g npm \ 
    && npm install -g quicktype

RUN apt install -y libssl-dev \
    && mkdir -p /var/run/sshd \
    && echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config \
    && ssh-keygen -A

RUN mkdir -p $dir_prefix

RUN cd $dir_prefix \
    && wget "http://download.hikvision.com/UploadFile/Soft/MVS/01%20Machine%20Vision/02%20Service%20Support/01%20Clients/MVS_STD_V2.0.0_191126(Linux%20X86).zip" \
    && unzip "MVS_STD_V2.0.0_191126(Linux X86).zip" \
    && dpkg -i MVS-2.0.0_x86_64_20191126.deb \
    && rm MVS-2.0.0_x86_64_20191126.deb "MVS_STD_V2.0.0_191126(Linux X86).zip"

RUN cd $dir_prefix \
    && wget "https://github.com/microsoft/CMake/releases/download/v3.16.3311152/cmake-3.16.3311152-MSVC_2-Linux-x64.sh" \
    && chmod +x cmake-3.16.3311152-MSVC_2-Linux-x64.sh \
    && ./cmake-3.16.3311152-MSVC_2-Linux-x64.sh --prefix=/usr/local --skip-license \
    && rm cmake-3.16.3311152-MSVC_2-Linux-x64.sh
 
RUN cd $dir_prefix \
    && git clone https://github.com/Microsoft/vcpkg.git -n \ 
    && cd vcpkg \
    && git checkout bc37190216256125bc7227456d5983809fec7373 \
    && ./bootstrap-vcpkg.sh -useSystemBinaries

# opencv installing begin
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install libjpeg-turbo:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install liblzma:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install zlib:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install libpng:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install libwebp[nearlossless,simd]:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install opengl:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install szip:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install tiff:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install eigen3:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install ffmpeg[avresample]:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install hdf5[szip,zlib]:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install opencv[nonfree,contrib,eigen,ffmpeg]:x64-linux
# opencv installing end

RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install fmt:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install spdlog:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install gtest:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install benchmark:x64-linux
RUN VCPKG_FORCE_SYSTEM_BINARIES=1 .$dir_prefix/vcpkg/vcpkg install nlohmann-json:x64-linux

RUN cd $dir_prefix \
    && git clone https://github.com/microsoft/mimalloc.git -n \
    && cd mimalloc \
    && git checkout 04f1c3b1e23677ce03bd16137e73089abd552175 \
    && mkdir -p out/release \
    && cd out/release \
    && cmake ../.. \
    && cmake --build . --config Release -j$((`nproc`+1)) \
    && cmake --build . --target install \
    && cd ../.. \
    && mkdir -p out/debug \
    && cd out/debug \
    && cmake -DCMAKE_BUILD_TYPE=Debug ../.. \
    && cmake --build . --config Debug -j$((`nproc`+1))

RUN apt install -y libcpprest-dev \
    && cd $dir_prefix \
    && git clone https://github.com/StephanXu/SignalR-Client-Cpp.git -n \
    && cd SignalR-Client-Cpp \
    && git checkout 296b1228af824c68ab8973e4dc1249e78ae547e7 \
    && git submodule update --init \
    && mkdir build.release \
    && cd build.release \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DUSE_CPPRESTSDK=true \
        -DCMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/ \
        -DCMAKE_INSTALL_PREFIX=$dir_prefix/libs/release/SignalR-Client-Cpp \
    && cmake --build . --config Release -j$((`nproc`+1)) \
    && cmake --build . --target install \
    && cd .. \
    && mkdir build.debug \
    && cd build.debug \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Debug \
        -DUSE_CPPRESTSDK=true \
        -DCMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/ \
        -DCMAKE_INSTALL_PREFIX=$dir_prefix/libs/debug/SignalR-Client-Cpp/ \
    && cmake --build . --config Debug -j$((`nproc`+1)) \
    && cmake --build . --target install

RUN cd $dir_prefix \
    && git clone https://github.com/gflags/gflags.git -n \
    && cd gflags \
    && git checkout e171aa2d15ed9eb17054558e0b3a6a413bb01067 \
    && mkdir build.release \
    && cd build.release \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=$dir_prefix/libs/release/gflags \
    && cmake --build . --config Release -j$((`nproc`+1)) \
    && cmake --build . --target install \
    && cd .. \
    && mkdir build.debug \
    && cd build.debug \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=$dir_prefix/libs/debug/gfalgs \
    && cmake --build . --config Debug -j$((`nproc`+1)) \
    && cmake --build . --target install \
    && cd .. \
    && cd ..

EXPOSE 22

ENV username=Ossian
ENV password=Ossian

CMD service ssh restart \
    && useradd -m -d /home/$username -s /bin/bash -G root $username || true \
    && (echo $password; echo $password) | passwd $username \
    && /bin/bash

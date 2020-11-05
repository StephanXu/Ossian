# Containerized development environment


You can build a mock development environment in Docker or simply pull the complete environment from [Dockerhub](https://hub.docker.com/repository/docker/stephanxu/ossian-dev-env).

> Note: Docker can't support running aarch container on x64 platform so this environment can only used to validate compile-time correctness.

## Usage

### Commands

You can run a container(host SSH on port 20022) easily by:

```bash
docker run -itd --name ossian-dev -p 20022:22 stephanxu/ossian-dev-env:<tag>
```

Crate a terminal by:

```bash
docker exec -it ossian-dev /bin/bash
```

Stop a terminal by:

```bash
docker stop ossian-dev
```

Delete a container by:

```bash
docker rm ossian-dev
```

### SSH support

By default, container will create a user in `root` group and set a password for SSH connection. The default username is `Ossian` and default password is `Ossian`. You can specify username and password through setting enviornment variables as follow:

```bash
docker run -itd --name ossian-dev -p 20022:22 -e username=<user-name> -e password=<password> stephanxu/ossian-dev-env:<tag>
```

It equivalent to run following command automatically when the container start:

```bash
service ssh start
useradd -m -d /home/<user-name> -s /bin/bash -G root <user-name>
passwd <user-name>
```

### CMake project arguments

If you connect the development container through IDEs support CMake. You may need these parameters:
|Item|Value|
|---|---|
| Vcpkg toolchain path | /usr/local/Ossian/vcpkg/scripts/buildsystems/vcpkg.cmake |
| CMake path | /usr/local/bin/cmake |
| CMake arguments | -DCMAKE_PREFIX_PATH="/opt/MVS/include;/opt/MVS/lib/64;/usr/local/Ossian/libs/release/SignalR-Client-Cpp/lib;/usr/local/Ossian/libs/release/SignalR-Client-Cpp/include;/usr/local/Ossian/libs/cpp-httplib;/usr/local/Ossian/libs/release/gflags" |

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

You may also need to connect development container, here is a initialization sample:

```bash
service ssh start
useradd -m -d /home/<user-name> -s /bin/bash -G root <user-name>
passwd <user-name>
```

### CMake project arguments

If you connect the development container through IDEs support CMake. You may need these parameters:
|Item|Value|
|---|---|
| Vcpkg toolchain path | /tmp/vcpkg/scripts/buildsystems/vcpkg.cmake |
| CMake path | /usr/local/bin/cmake |
| CMake arguments | -DCMAKE_PREFIX_PATH="/opt/MVS/include;/opt/MVS/lib/64" |

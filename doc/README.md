# 如何生成文档

文档生成依赖Doxygen。如果你还没有安装Doxygen，使用如下的命令安装Doxygen：

```shell
sudo apt-get install doxygen
```

为了绘图方便，请安装Graphviz：

```shell
sudo apt-get install graphviz
```

安装完成后，在项目根目录运行：

```shell
doxygen doc/Doxyfile
```

即可在`doc/doxygen`中查看输出的文档
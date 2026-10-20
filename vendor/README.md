# vendor

将 `sensors` 包软链到本目录，使 `import sensors` 可用：

```bash
# Linux / macOS
ln -sfn /path/to/sensors/src/sensors vendor/sensors

# 或 editable 安装 sensors 后无需软链
```

Windows（管理员 CMD）示例：

```bat
mklink /D vendor\sensors C:\path\to\sensors\src\sensors
```

不要把整个 sensors 源码树复制进本仓库。

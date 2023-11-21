# light-flow
光流例子

## 启动方式

```
# 执行 usb 规则绑定，只有在本机只有一个 1a86:7523 的时候有效
./scripts/create_udev_rules.sh

# 检查 /dev/light_flow 是否是期望的设备，执行 cat 命令，有可能看到一些 UP 字样后面跟一些乱码
cat /dev/light_flow
UP
xxxx
xx

# 启动
roslaunch light_flow light_flow.launch
```

发出的 topic 为 ```/optical_flow```
# light-flow
光流例子

## 启动方式

```
# 执行 usb 规则绑定，只有在本机只有一个 1a86:7523 的时候有效
./scripts/create_udev_rules.sh

# 修改 launch 文件中的 device 字段
roslaunch light_flow light_flow.launch
```

发出的 topic 为 ```/optical_flow```
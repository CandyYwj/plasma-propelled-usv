# arduino demo 例程使用说明



## 平台

所有例程均使用 arduino IDE 编写,使用 arduino uno 开发板测试.

## 目录结构

├─.pio
│  ├─build
│  │              
│  └─libdeps
│      └─uno (引用的第3方库文件夹)
│          ├─FlexiTimer2
│          └─SoftwareSerial
│      
├─examples (例程文件)
│  ├─A02_IIC
│  │      A02_IIC.ino
│  │      
│  ├─A05
│  │      A05.ino
│  │      
│  ├─A15_Current
│  │      A15_Current.ino
│  │      
│  ├─A15_Voltage
│  │      A15_Voltage.ino
│  │      
│  ├─ff_uart_auto
│  │      ff_uart_auto.ino
│  │      
│  ├─ff_uart_ctrl
│  │      ff_uart_ctrl.ino
│  │      
│  ├─H01
│  │      H01.ino
│  │      
│  ├─H03TRT
│  │      H03TRT.ino
│  │      
│  ├─modbus
│  │      modbus.ino
│  │      
│  ├─pwm
│  │      pwm.ino
│  │      
│  └─U02
│          U02.ino    

examples 文件夹存放例子程序, .pio 文件夹存放例子程序使用到的第 3 方库 (例子程序使用到的话,需手动加载进项目工程中),注意此文件夹是隐藏目录,如果操作系统不打开查看隐藏文件夹功能的话,是无法看到的.

## 硬件连接

arduino uno A0 接传感器 TX 脚,A1 接传感器 RX 脚,注意如果使用的是 u02 电压输出传感器, 传感器的 S 脚接开发板 A0 脚.

## 举个栗子(以 ff_uart_ctrl.ino 为例)

1. 打开例程文件,进入 examples\ff_uart_ctrl,双击打开 ff_uart_ctrl.ino
2. 检查 arduino IDE 设置

![image-20211110113541221](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110113541221.png)

确定开发板为:Arduino Uno

选择正确的端口

3. 由于 ff_uart_ctrl.ino 引用了 FlexiTimer2 库,所以需要先添加库

   ![image-20211110114022280](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110114022280.png)

![image-20211110114107033](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110114107033.png)

定位到 FlexiTimer2 文件夹,选择打开即可.

注意,如果无法查看到 .pio 文件夹,请检查

![image-20211110114316516](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110114316516.png)


4. 编译并下载

   ![](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110115133175.png)

   ![image-20211110115150206](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110115150206.png)
5. 查看数据

   5.1. 打开串口监视器

![image-20211110114440033](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110114440033.png)

```
5.2. 修改波特率为 9600 bps,即可看到解析数据
```


![image-20211110114627620](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211110114627620.png)

## 切换源程序通讯协议 (针对支持多协议的 demo),以 a05.ino demo 为例

修改前

![image-20211123174649925](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211123174649925.png)

修改后

![image-20211123174725297](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211123174725297.png)

修改完成后,重新编译下载即可

## 注意

在使用串口助手查看数据时，须使用 “UTF-8” 格式查看.

![image-20211130110753308](C:\Users\DYP-RD\AppData\Roaming\Typora\typora-user-images\image-20211130110753308.png)

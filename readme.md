## 实际测量
* 严重flicker

![real flicker](Doc\real_flicker.png)

* 中等flicker

![real flicker](Doc\real_flicker1.png)

* 轻微flicker

![real flicker](Doc\real_flicker2.png)

## 理论计算方法
![flicker](Doc\flicker_equation.png)

从实际的测试可以看出来，随着flicker程度的变化，DC值并没有明显的变化，而AC值会越来越小。

## 算法
最简单的方法，采集一堆数据，然后求最大值，最小值，平均值，最后代入上面的公式即可。但是这样抗干扰的能力太弱，电信号必然有高频噪声在。因此需要做一些简单的滤波。

这里采用的简单算法如下：
1. 采集N个周期的数据
2. 升序排序
3. 去掉最大值，最小值
4. 取前面N-1个数的平均值作为最小值
5. 取后面N-1个数的平均值作为最大值
6. 求平均值
7. 代入上面的公式

## 驱动层
很明显，需要使用ADC采样。flicker的周期约为50ms。按照10倍采样率，采样周期5ms。采集N个周期，采集的数据量将会是10*N。因此设计如下：
1. ADC 设置成Timer触发测量，DMA模式
2. Timer 设置周期5ms的PWM
3. DMA总长10*N，开DMA完成中断
4. 中断产生后，切换buffer，并发出信号量，开始算法


注意算法需要在50*N ms内完成。

## 如何自适应亮度
硅光电池对不同的亮度转换不同的电压值，经过放大器之后，输出的电压值可能会饱和或者太低。需要自适应调节反馈电阻。
另外硅光电池也会饱和。测试饱和电压约1.6V，即ADC的值在2600左右。

需要防止饱和造成最大值误差，因此当检测到最大值等于ADC的最大测量值时，减小反馈电阻。

同时，为保证测量精度，放大倍数不能太小，因此需要保证DC值在一定的范围。


## 系统性能
测量周期 50*N ms，因此测量频率大于等于20HZ。

## 通讯协议
### 物理层
1. UART通讯，波特率115200，双工，无奇偶校验，1个起始位，1个停止位
2. 每两个数据包之间至少要有1ms的间隔时间

### 组成
用C表示为
```c
__packed typedef struct
{
    uint8_t DeviceID;
    uint8_t PackageID;
    uint16_t DataLength;
    uint8_t *Data;
    uint8_t Crc8;
}PackageTypeDef;
```
1. DeviceID : 设备ID号，表明是什么设备，目前分配如下：
``` c
typedef enum
{
    TP = 0,
    FLCIKER_SENSOR,
    U_DISK
}DeviceIDTypeDef;
```
2. PackageID : 表示这个包是干啥用的，自定义
3. DataLength : 数据区长度
4. Data ： 数据
5. Crc8 : 对前面所有数据进行Crc计算得出的数据，计算方式如下：

```c
/**
* Static table used for the table_driven implementation.
*****************************************************************************/
static const crc8_t crc_table[256] = { 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8 };


uint8_t CalCrc8(const uint8_t *data, uint16_t data_len)
{
    uint8_t crc = 0xff;
    
    while (data_len--) 
    {
        crc = crc_table[(crc ^ *data) & 0xff];
        data++;
    }
    
    return crc;
}
```

### 举例

假设TP要发送一个结果数据过来，参考代码如下：

```c
typedef enum
{
    NG = 0,
    PASS = 1,
}ResultTypeDef;

typedef enum
{
    RESULT,
}IDTypeDef;


void SendResult(ResultTypeDef result)
{

    PackageTypeDef package;
    package.DeviceID = TP;
    package.PackageID = RESULT;
    package.DataLength = sizeof(ResultTypeDef);
    package.Data = (uint8_t *)&result;

    package.Crc8 = CalCrc8(...);


    UART_Send(...);
    ...
}

```

### 硬件平台

| **模块** | **规格**       | **详情**                     |
| ------ | ------------ | -------------------------- |
| MCU    | CH32V307VCT6 | RISC-V （带 FPU）144 MHz      |
| RTOS   | FreeRTOS     | 4 任务并发调度                   |
| 摄像头    | MT9V03X      | ·DVP 接口 ，设定120×160 原始分辨率   |
| 处理分辨率  | 60×80        | 行列各压缩一倍下采样                 |
| 图像帧率   | 50 HZ        | 利用VSYNC中断触发DMA传输，实现零拷贝图像采集 |
### 任务架构

| **任务**          | **职责**               | **触发方式**         | **优先级** |
| --------------- | -------------------- | ---------------- | ------- |
| Control_Task    | 硬实时控制环               | TIM6 1ms 任务通知     | 12      |
| Perception_Task | 图像采集与算法处理            | DVP 场中断通知         | 10      |
| System_Task     | 电压监测                 | 100 ms 周期唤醒       | 3       |
| UI_Task         | 屏幕显示刷新               | 40 ms 周期唤醒        | 2       |

Control_Task 作为电机控制的一把手，采用硬实时控制，并设定最高优先级；Control_Task 依赖 Perception 输出的 `image_error` 做 PID 运算，故Perception_Task次优先级。而对于System_Task、UI_Task一个考虑设备安全性，另一个考虑设备交互性，在竞赛中没有要求，故低优先级。

### BUG 现象描述
在我备战智能车竞赛-气垫船组别的过程中，遇到了一个非常难缠的问题：二值化图像偶发性地出现大面积黑屏或全白，频率大约每隔数秒一次，毫无规律。在我花了 2 天时间研究，追踪数据发现，Otsu 算法输出的动态阈值会**突然跳变到接近0或255的极端值**，持续一到两帧后自动恢复。

我只能说上一代学长留下来的图像处理代码真的是“屎”山代码。注释几乎没有，可读性奇差无比，好在 AI 帮我整理了这部分代码的所有全局变量，我才得以能够下手。
#### 主要现象
1. UI上显示的binary_image与middle_line出现随机跳变，偶发全黑或全白帧
2. 将PRIORITY_CONTROL与PRIORITY_PERCEPTION优先级互换后，跳变现象消失
3. 对整个Otsu1D计算加临界区保护后，跳变现象消失
4. 将临界区缩小至仅保护直方图累加循环后，跳变现象减少但偶有发生

### 双缓冲治不好的“竞争”
在遇到bug后，我的第一反应是数据竞争：摄像头通过DVP中断直接写入全局变量的缓冲区，而`Perception_Task`读取该缓冲区后，立刻对图像做压缩处理。而提高Perception_Task优先级后，数据似乎能正常传出，图像也不纯黑纯白跳变了。但这只是表象，因为高优先级任务被抢占的概率降低了，并未根除问题。毕竟，谁家BUG是通过调整优先级修好的？

于是我阅读了 CH32V307 手册中关于dvp模块的寄存器描述，发现硬件本身支持双缓冲区自动切换：DVP 的 `DMA_BUF0` 和 `DMA_BUF1` 寄存器可以配置两个不同的内存地址，硬件会在帧完成时自动切换目标缓冲区。

我一开始尝试给`DMA_BUF0` 和 `DMA_BUF1` 寄存器分配不同的图像栈地址，但最后却发觉这两个寄存器只在处理一帧图像时有效，第二帧就会清零，开始新循环。所以得到的内容是`mt9v03x_image0`
和 `mt9v03x_image1` 两个二维图像数组各占一半。压根不能作为双缓冲啊！

再接着，我就这样修改了dvp中断函数：
```C
    static uint32_t frame_cnt = 0;
    // 切换DMA缓冲区
    DVP->DMA_BUF0 = (uint32_t)(frame_cnt % 2 ? mt9v03x_image1 : mt9v03x_image0);
    if (perception_task_handle != NULL)
    {
        xTaskNotifyFromISR(perception_task_handle,
                           frame_cnt % 2,
                           eSetValueWithOverwrite,
                           &xHigherPriorityTaskWoken);
    }
    frame_cnt++;
```
这是两个缓冲区的定义：
```C
// 指针数组，通过索引访问
uint8_t (*frame_buffers[2])[MT9V03X_H][MT9V03X_W] = {
    &mt9v03x_image0,
    &mt9v03x_image1
};
```

理论上这能彻底隔离生产者和消费者。然而神奇的是，bug没有任何变化，依然矗立在那里？？？此时我开始怀疑，竞争也许只是表象。
### 用临界区二分定位
我接着按调用链逐层插入 `taskENTER_CRITICAL()`，来二分定位到底是哪里发生了竞争：

- 对 `analyze_image()` 整体加临界区 → 跳变消失
- 缩小到 `preprocess_image()` → 跳变消失
- 缩小到 `Otsu1D()` 直方图累加循环加临界区 → 跳变减少但**偶现**
- 对整个 `Otsu1D()` 计算过程加临界区 → 跳变几乎消失
也就是说，`Otsu1D()`大津法计算图像二值化阈值这里，出现了任务切换时的竞争，但到底是哪个变量呢？我反复猜测，终于……
浮点数版本：
```C
unsigned char Otsu1D(uint8_t img_2d[IMAGE_HEIGHT][IMAGE_WIDTH], uint8 height, uint8 width)//一维大津法，计算全局阈值
{
    float Histogram[256] = {0}; //建立一维灰度直方图,并初始化变量
    uint32 N = height * width;//像素的总数
    for (int i = 0; i < height; i++)//矩阵的行数
    {
        for (int j = 0; j < width; j++)//矩阵的列数
        {
            unsigned char gray_val = img_2d[i][j];//获取当前灰度值
            Histogram[gray_val]++;//记录（i,j）的数量
        }
    }

    // 归一化
    for (int i = 0; i < 256; i++)
    {
        Histogram[i] /= N;//归一化的每一个二元组的概率分布
    }
    
    float avg_val = 0.0;
    for (int i = 1; i < 256; i++)
    {
        avg_val += i * Histogram[i];
    }

    int threshold = 0; //阈值
    float max_variance = 0.0;
    float w = 0.0, u = 0.0;

    for (int i = 0; i < 256; i++)
    {
        w += Histogram[i];
        u += i * Histogram[i];

        // 跳过边界，避免除零
        if (w < 1e-6f || w > 1.0f - 1e-6f)
            continue;

        float t = avg_val * w - u;
        float variance = t * t / (w * (1 - w));
        if (variance > max_variance)
        {
            max_variance = variance;
            threshold = i;
        }
    }
    return threshold;
}
```
整数版本：
```C
static unsigned char otsu_threshold(unsigned char img[IMAGE_HEIGHT][IMAGE_WIDTH])
{
    /* ── 1. 建整数直方图 ── */
    uint16_t hist[256];
    memset(hist, 0, sizeof(hist));

    for (int i = 0; i < IMAGE_HEIGHT; i++)
        for (int j = 0; j < IMAGE_WIDTH; j++)
            hist[img[i][j]]++;          /* 最大值 4800，uint16 足够 */

    /* ── 2. 全局灰度加权总和 ── */
    const int32_t N = IMAGE_HEIGHT * IMAGE_WIDTH; /* 4800，编译期常量 */
    int32_t sum_total = 0;
    for (int i = 0; i < 256; i++)
        sum_total += (int32_t)i * hist[i]; /* max 1,224,000 → int32 ✓ */

    /* ── 3. 遍历阈值，最大化类间方差 ── */
    int32_t count0 = 0;
    int32_t sum0   = 0;
    int64_t best   = 0;
    int     thresh = 0;

    for (int i = 0; i < 256; i++) {
        count0 += hist[i];
        sum0   += (int32_t)i * hist[i];

        int32_t count1 = N - count0;
        if (count0 == 0 || count1 == 0) continue; /* 跳过边界无效段 */

        /*
         * t = sum0 * N - sum_total * count0
                  * 最大约 5.87e9，用 int64 承接，再右移 2 位压入 int32
                  * 以便平方后仍在 int64 范围内
         */
        int32_t ts = (int32_t)(
            ((int64_t)sum0 * N - (int64_t)sum_total * count0) >> 2
        );

        /* val = ts² / (count0 × count1)，全程 int64，无浮点 */
        int64_t val = (int64_t)ts * ts / ((int64_t)count0 * count1);

        if (val > best) {
            best   = val;
            thresh = i;
        }
    }
    return (unsigned char)thresh;
}
```
发现使用整数版本（用移位和整数除法替代浮点），bug就不再出现了。
### 真正的根因——FPU 寄存器
CH32V307 支持 RV32IMAC **并带有硬件单精度浮点单元（F 扩展）**，所有浮点运算直接使用 FPU 完成，一条指令即可完成除法，不存在软浮点多指令序列问题。那为什么加临界区会消除跳变？
深入查阅 FreeRTOS 在 RISC‑V 上的移植层，发现默认的上下文切换代码 **仅保存整数寄存器（x1~x31）**，而 **浮点寄存器（f0~f31）并未保存**。当感知任务执行 Otsu 浮点运算时被更高优先级的控制任务抢占，抢占回来后 FPU 寄存器内容已被破坏，导致后续浮点计算（特别是除法、累加）产生随机错误，从而输出极端阈值。临界区阻止了任务切换，意外“修复”了问题，但代价是牺牲实时性。

- 当只保护直方图累加循环时，FPU 仍在被使用（接下来的计算平均除法尚未执行），抢占可能发生在任何一条 FPU 指令之后；
- 当保护整个 Otsu 计算时，该任务在此期间不会被切换出去，FPU 寄存器得以完整保留；
- 互换优先级后 Perception 优先级降低，被抢占概率下降，但并未根除，只是掩盖了竞态窗口。

**关键发现**：CH32V307 的 FreeRTOS 移植未使能 FPU 上下文保存，是本次竞态的根本原因。
### 根治与反思
修复方案很干脆：[开启FreeRTOS的FPU支持](https://club.gizwits.com/thread-169672-1-1.html)。
#### 1. 启用 FPU 寄存器上下文保存
```C
/* if you want to use FPU, please define ARCH_FPU and enable float point and ABI of gcc */

#define ARCH_FPU 1
```
#### 2. 配置 FPU
要告诉编译器，目标芯片支持硬件浮点指令，并且使用单精度浮点ABI。
1.  在MounRiver Studio中，右键点击你的工程，选择 **Properties**。
2.  在左侧菜单栏，依次展开并进入： **C/C++ Build** -> **Settings**。
3.  在右侧的 **Target Processor** 选项卡中，进行如下设置：
    *   **Floating point**：选择 `Single precision extension （RVF）`。这表示允许编译器生成单精度浮点扩展指令。
    *   **Floating point ABI**：选择 `Single precision （-mabi=ilp32f）`。这表示传递浮点参数时使用浮点寄存器，这是利用硬件FPU效率最高的方式。
#### 3. 优化单精度常量
启用单精度常量优化是让编译器将不带后缀的浮点常量（如 `3.14`）当作单精度（`float`）而不是默认的双精度（`double`）处理。这样可以避免编译器调用低效的软浮点库函数来处理双精度常量，从而充分利用FPU进行快速计算。

**操作步骤：**
1.  在上述 **Settings** 界面中，切换到 **Optimization** 选项卡。
2.  勾选 **Single precision constants** 选项。


### 经验总结
**系统级调试思维**：从表象（数据竞争）深入到系统底层（RTOS 上下文切换、FPU 硬件特性），认识到问题可能跨越软件与硬件的边界。
**硬件手册与移植代码同等重要**：CH32V307 的 RISC‑V 内核支持 F 扩展，但移植层若不启用，FPU 就成了“定时炸弹”。查阅内核手册和 RTOS 移植指南缺一不可。
**临界区只能验证，不能依赖**：盲目扩大临界区会掩盖问题、破坏实时性，必须通过严谨的定位并找到根本原因。
**浮点运算在实时系统中的风险**：在多任务环境中，必须确保 FPU 上下文完整，否则看似正确的代码会因抢占而产生偶发性错误。

这个经历让我深刻体会到，**调试不仅仅是修改代码，更是对系统整体行为（任务调度、中断、硬件特性）的深度理解**，也为后续设计高可靠性嵌入式系统积累了宝贵经验。
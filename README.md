# RISC-V 气垫船控制系统

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![MCU: CH32V307](https://img.shields.io/badge/MCU-CH32V307-RISC--V-green)](https://www.wch.cn/products/CH32V307.html)
[![RTOS: FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-blue)](https://www.freertos.org/)

基于 RISC-V 架构和 FreeRTOS 实时操作系统的全向气垫船运动控制平台，集成了先进的无感电机驱动、高精度串级 PID 控制算法和四级人机交互菜单系统，实现了气垫船的精准姿态控制和稳定运动性能。

**第十九届全国大学生智能汽车竞赛 - 气垫船项目**

---

## 🚀 核心特性

### 🧠 智能控制系统
- **RISC-V 核心**：采用沁恒 CH32V307 高性能微控制器，144MHz 主频，支持硬件 FPU
- **实时操作系统**：基于 FreeRTOS 构建多任务实时架构，任务响应时间 < 1ms
- **混合调度策略**：优先级抢占 + 时间片轮转混合调度，确保关键任务实时性
- **任务划分**：传感器采集、电机控制、人机交互、业务逻辑四核心任务分离

### 🚀 高性能动力系统
- **无刷电机驱动**：
  - 无传感器方波驱动，基于反电动势过零检测算法
- **有刷电机驱动**：
  - 4路独立有刷电机驱动，支持 PWM 精确控制
  - 过流保护、热保护机制
- **运动控制算法**：
  - 串级 PID 控制器（内环角速度 + 外环速度）
  - 抗干扰自适应调节，速度控制精度 ±2%
  - 运动解耦控制，支持全向移动

### 🖥️ 人性化交互系统
- **四级菜单架构**：基于结构体数组的查询系统，耦合度低，扩展性强
- **实时监控显示**：ST7789V2 IPS 显示屏，实时显示系统状态参数
- **便捷参数调整**：EC11 旋转编码器，支持 PID 参数、控制模式在线调整
- **状态指示**：电压、温度、姿态角等多参数同屏显示

---

## 📁 项目结构
```
Hovercraft_Control_System/
├── README.md
├── Tools/          # 测试工具和辅助软件
├── Document/       # 技术文档和芯片手册
├── Firmware/       # 嵌入式固件和驱动程序
└── Hardware/       # 电路原理图和PCB设计
```

---

## 🛠️ 硬件平台

### 核心控制器
- **主控芯片**：WCH CH32V307VCT6 RISC-V MCU
- **调试接口**：WCH-LinkE 调试器

### 动力系统
- **推进电机**：BLDC 无刷电机 × 2
- **转向电机**：直流有刷电机 × 4
- **驱动模块**：定制无刷驱动板 + 有刷驱动板

### 传感系统
- **姿态传感器**：IMU20602 六轴陀螺仪加速度计
- **视觉传感器**：MT9V034 全局快门摄像头
- **速度反馈**：增量式编码器 × 1

### 人机交互
- **显示设备**：2.0 寸 ST7789V2 IPS 显示屏
- **输入设备**：EC11 旋转编码器 + 按键
- **状态指示**：RGB LED 指示灯

---

## 🔧 快速开始

### 环境搭建
1. **安装开发环境**：MountRiver Studio (MRS)
2. **配置工具链**：RISC-V GCC 编译工具链
3. **准备调试器**：WCH-LinkE 调试编程器

### 编译烧录
```bash
# 克隆项目
git clone https://github.com/your-username/Hovercraft_Control_System.git

# 导入工程到 MRS
# 配置编译选项
# 连接硬件并烧录程序
```

### 系统调试
1. **参数整定**：使用配套上位机工具调整 PID 参数
2. **数据监控**：通过串口实时监控系统运行状态
3. **性能测试**：分模块测试各功能单元性能指标

---

## 🤝 参与贡献

我们欢迎各种形式的贡献，包括但不限于：

- 🐛 **问题反馈**：提交 Issue 报告发现的 Bug
- 💡 **功能建议**：提出新功能或改进建议
- 🔧 **代码贡献**：提交 Pull Request 改进代码
- 📚 **文档完善**：帮助完善项目文档和使用教程
- 🔬 **技术研究**：参与算法优化和性能提升

### 贡献流程
1. Fork 本仓库到你的账户
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

---

## 🙏 致谢

- **全国大学生智能汽车竞赛组委会** - 提供宝贵的竞赛平台和学习机会
- **南京沁恒微电子股份有限公司** - 提供 CH32V307 RISC-V 芯片技术支持和开发工具
- **成都逐飞科技有限公司** - 提供完善的 CH32V307 软件库和硬件设计参考

### 特别感谢
感谢所有为项目做出贡献的老师和同学们，你们的辛勤付出和智慧结晶成就了这个项目！

---

## 📞 联系我们

如有问题或建议，欢迎通过以下方式联系：

- 📧 **邮箱**：joenikon04@gmail.com
- 💬 **Issues**：[项目 Issues 页面](https://github.com/JoinNico/Hovercraft_Control_System/issues)
- 🏠 **主页**：[项目主页](https://github.com/JoinNico/Hovercraft_Control_System)

---
*最后更新：2025年11月*
```
# 单相逆变器

## 开发环境

*   硬件平台：<https://oshwhub.com/hahahawoshishabi/ke-long> （嘉立创他人开源）
*   软件平台：Keil uVision5，STM32CubeMX，Matlab(2020b)

## 使用教程

### simulink仿真模型

1.  使用 Matlab2020b 打开 Single_Phase_Inverter\Simulations\Matlab\single_phase_inverter.slx  
    ![simulink_01](Documents\Media\simulink_01.png)  
2.  控制部分如图，程序设计根据控制部分编写  
    ![simulink_02](Documents\Media\simulink_02.png)  
3.  仿真结果如图逆变器 1、2 输出电流，逆变器 1 在 0.02s 时启动，逆变器 2 在 0.06s 时启动与逆变器 1 并联  
    ![simulink_03](Documents\Media\simulink_03.png)  

### 程序代码使用

1.  需要安装 Keil 和 STM32CubeMX 对应 STM32F3xx 系列芯片的支持包，主控芯片为 STM32F334R8T6
2.  使用 STM32CubeMX 打开 Single_Phase_Inverter\Codes\project\project.ioc，点击右上角 GENERATE CODE 生成工程  
3.  使用 Keil 打开 Single_Phase_Inverter\Codes\project\MDK-ARM\project.uvprojx  
4.  添加相应头文件，编译无误即可  
    ![keil_01](Documents\Media\keil_01.png)  
5.  下载烧录程序之前需根据实际情况更改 main.c 中相应参数  
6.  并联运行时可通过输入电流判断并联是否成功，如果电流大小正常及即并联成功，若输入电流过大则代表两个逆变器之间有环流，并联失败  
    ![action_01](Documents\Media\action_01.png)




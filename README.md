# WHU_balance_infantry

#### 介绍
基于交✌平步理论分析，港大代码的平衡步兵测试代码

#### 软件架构

程序编写采用vscdo结合keil进行高效编写，采用MATLAB拟合LQR VMC拟合矩阵

#### 使用说明

1. **05LQR拟合文件夹**：传入自己平步的各种物理参量，设置好Q R 矩阵个各变量权重，拟合数据使用方法：

    ```c
    float Fit_Matrix[40][6];
    float L_length ,R_length;
    uint8_t i,j,index;
    
    for (i=0;i<4;i++)
    {
        for (j=;j<10;j++)
        {
            index=i*10+j;
            LQR[i][j] = Fit_Matrix[index][0] + Fit_Matrix[index][1]*L_length + Fit_Matrix[index][2]*R_length + Fit_Matrix[index][3]*L_length*L_length + Fit_Matrix[index][4]*L_length*R_length + Fit_Matrix[index][5]*R_length*R_length
        }
    }
    ```

    

2. **04jacbon_calculation文件夹**：运行matlab工程中的主文件，会生成相应的拟合矩阵在左侧工作栏，同时可以在主文件最下方输入腿长腿角进行测试，反馈雅克比矩阵和髋关节扭矩大小

3. 01Balance_chassis_task文件夹：用Keil打开chassis_task.uvprojx文件，编译，下载，调试

#### 调试建议（借鉴CADN上 星夜雨夜的调试建议...）

![image-20250921185536221](C:/Users/ASUS/AppData/Roaming/Typora/typora-user-images/image-20250921185536221.png)

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)

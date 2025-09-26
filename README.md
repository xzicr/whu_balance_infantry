# WHU_balance_infantry

#### 介绍
基于交✌平步理论分析，港大代码的平衡步兵测试代码

#### 软件架构

程序编写采用vscode结合keil进行高效编写，采用MATLAB拟合LQR VMC拟合矩阵

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

#### LQR原理讲解

![](C:/Users/ASUS/AppData/Roaming/Typora/typora-user-images/image-20250925104638323.png)

![image-20250925105807430](C:/Users/ASUS/AppData/Roaming/Typora/typora-user-images/image-20250925105807430.png)

![image-20250925105045532](C:/Users/ASUS/AppData/Roaming/Typora/typora-user-images/image-20250925105045532.png)







#### 参考文献

[轮腿机器人代码调试补充](https://blog.csdn.net/Kevin3389179304/article/details/148379498?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522c9e3eb48c656ad1cc51178d32528747e%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=c9e3eb48c656ad1cc51178d32528747e&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-148379498-null-null.142^v102^pc_search_result_base5&utm_term=%E5%B9%B3%E8%A1%A1%E6%AD%A5%E5%85%B5%E8%B0%83%E8%AF%95%E6%80%9D%E8%B7%AF&spm=1018.2226.3001.4187)




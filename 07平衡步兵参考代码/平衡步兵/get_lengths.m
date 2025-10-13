function K = get_lengths(length)

%% 符号变量定义
% seita_wl : 左驱动轮转角                seita_wr ： 右驱动轮转角 
% seita_ll ：左腿倾斜角                  seita_lr ：右腿倾斜角
% seita_b  ： 机体倾斜角                    fai   ： 偏航角
% S ：自然坐标系下机器人水平方向移动距离      Sb    ： 自然坐标系下机体质心水平方向移动距离
% Sll Slr ： 自然坐标系下腿部质心水平方向移动距离
% hll hlr ： 自然坐标系下腿部质心竖直方向移动距离
% Tlwl Tlwr ： 驱动轮转矩（腿-轮）
% Tbll Tblr ： 腿部转矩（机体-腿）
% fl fr ：地面对驱动轮摩擦力
% Fwsl Fwsr ： 驱动轮对腿水平方向作用力
% Fwhl Fwhr ： 驱动轮对腿竖直方向作用力
% Fbsl Fbsr ： 腿对机体水平方向作用力
% Fbhl Fbhr ： 腿对机体竖直方向作用力
% hb ： 自然坐标系下机体质心竖直方向移动距离

syms seita_wl seita_wr seita_ll seita_lr seita_b fai S Sb hb Sll Slr hll hlr...
     Tlwl Tlwr Tbll Tblr fl fr Fwsl Fwsr Fwhl Fwhr Fbsl Fbsr Fbhl Fbhr...
     dS dfai dseita_ll dseita_lr dseita_b d2S d2fai d2seita_ll d2seita_lr d2seita_b...
     dSb dseita_wl dseita_wr d2Sb d2seita_wl d2seita_wr

%% 系统参数
Rw = 0.077;      Rl = 0.240;           %% 驱动轮半径  驱动轮轮距/2
ll = length;     lr = length;          %% 腿长（左右）
lwl = length/2;   lwr = length/2;      %% 驱动轮到腿部质心距离（左右）  
lbl = length/2;   lbr = length/2;      %% 机体质心到腿部质心距离（左右） 
lc = -0.01;                            %% 机体质心到腿部关节距离 
mw = 0.2;   ml = 1.3;   mb = 11.65;    %% 驱动轮质量  腿部质量  机体质量
Iw = mw*Rw^2;  Ill = ml*(ll^2+0.05^2)/12.0;  Ilr = ml*(lr^2+0.05^2)/12.0; %% 驱动轮转动惯量（0.244592）  腿部转动惯量（左右）
a = 0.48;                              %% 机器人边长
Ib = mb*(0.3^2+0.12^2)/12.0;  Iz = 2*0.5*(mw*2+ml*2+mb)*Rw*Rw; %% 机体转动惯量  机器人z轴转动惯量*2
g = 9.8;                               %% 重力加速度

%% 系统动力学方程
eq1 = (Iw*ll/Rw + mw*Rw*ll + ml*Rw*lbl)*d2seita_wl + (ml*lwl*lbl-Ill)*d2seita_ll + ...
      (ml*lwl + 0.5*mb*ll)*g*seita_ll + Tbll - Tlwl*(1+ll/Rw) == 0;
eq2 = (Iw*lr/Rw + mw*Rw*lr + ml*Rw*lbr)*d2seita_wr + (ml*lwr*lbr-Ilr)*d2seita_lr + ...
      (ml*lwr + 0.5*mb*lr)*g*seita_lr + Tblr - Tlwr*(1+lr/Rw) == 0;
eq3 = -(mw*Rw^2 + Iw + ml*Rw^2 + 0.5*mb*Rw^2)*d2seita_wl - ...
      (mw*Rw^2 + Iw + ml*Rw^2 + 0.5*mb*Rw^2)*d2seita_wr - ...
      (ml*Rw*lwl+0.5*mb*Rw*ll)*d2seita_ll - (ml*Rw*lwr + 0.5*mb*Rw*lr)*d2seita_lr + ...
      Tlwl + Tlwr == 0;
eq4 = (mw*Rw*lc + Iw*lc/Rw + ml*Rw*lc)*d2seita_wl + ...
      (mw*Rw*lc + Iw*lc/Rw + ml*Rw*lc)*d2seita_wr + ...
      ml*lwl*lc*d2seita_ll + ml*lwr*lc*d2seita_lr - Ib*d2seita_b + ...
      mb*g*lc*seita_b - (Tlwl + Tlwr)*lc/Rw - (Tbll + Tblr) == 0;
eq5 = (0.5*Iz*Rw/Rl + Iw*Rl/Rw)*d2seita_wl - ...
      (0.5*Iz*Rw/Rl + Iw*Rl/Rw)*d2seita_wr + ...
      0.5*Iz*ll/Rl*d2seita_ll - 0.5*Iz*lr/Rl*d2seita_lr - ...
      Tlwl*Rl/Rw + Tlwr*Rl/Rw == 0;

%% 求解加速度项
[dd_wl, dd_wr, dd_ll, dd_lr, dd_b] = solve([eq1, eq2, eq3, eq4, eq5], ...
    [d2seita_wl, d2seita_wr, d2seita_ll, d2seita_lr, d2seita_b]);

%% 运动学关系
d2S = 0.5*Rw*(dd_wl + dd_wr);
d2fai = 0.5*Rw/Rl*(-dd_wl + dd_wr) - 0.5*ll/Rl*cos(seita_ll)*dd_ll + ...
        0.5*lr/Rl*cos(seita_lr)*dd_lr + 0.5*ll/Rl*sin(seita_ll)*dseita_ll^2 - ...
        0.5*lr/Rl*sin(seita_lr)*dseita_lr^2;

%% 状态空间方程
x = [S, dS, fai, dfai, seita_ll, dseita_ll, seita_lr, dseita_lr, seita_b, dseita_b];
u = [Tlwl; Tlwr; Tbll; Tblr];
fx = [dS; d2S; dfai; d2fai; dseita_ll; dd_ll; dseita_lr; dd_lr; dseita_b; dd_b];

%% 计算雅可比矩阵
A = jacobian(fx, x);
B = jacobian(fx, u);

%% 在平衡点进行线性化
% 创建符号变量到数值0的映射
x_syms = [S, dS, fai, dfai, seita_ll, dseita_ll, seita_lr, dseita_lr, seita_b, dseita_b];
x_zeros = num2cell(zeros(size(x_syms)));

% 代入平衡点（所有状态变量为0）
A_lin = subs(A, x_syms, x_zeros);
B_lin = subs(B, x_syms, x_zeros);

% 转换为数值矩阵
A = double(A_lin);
B = double(B_lin);

%% LQR设计
% [S, dS, fai, dfai, seita_ll, dseita_ll, seita_lr, dseita_lr, seita_b, dseita_b]        
Q = diag([25, 15, 100, 10, 50, 10, 50, 10, 5000, 1]);
R = diag([2.0, 2.0, 0.25, 0.25]);

K = lqr(A, B, Q, R);

end
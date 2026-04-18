% main_leg_calculation.m
% 五杆机构雅可比矩阵计算主程序

clear all; close all; clc;

%% 步骤1：设置五杆机构参数
disp('=== 五杆机构参数设置 ===');
parameters.l1 = 0.15;    % 右腿第一段长度 [m]
parameters.l2 = 0.27;    % 右腿第二段长度 [m]  
parameters.l3 = 0.27;    % 左腿第一段长度 [m]
parameters.l4 = 0.15;    % 左腿第二段长度 [m]
parameters.l5 = 0.15;    % 髋关节间距 [m]

fprintf('机构参数:\n');
fprintf('右腿: l1=%.3fm, l2=%.3fm\n', parameters.l1, parameters.l2);
fprintf('左腿: l3=%.3fm, l4=%.3fm\n', parameters.l3, parameters.l4);
fprintf('髋间距: l5=%.3fm\n', parameters.l5);

% 保存参数
save('leg_parameters.mat', 'parameters');
disp('机构参数已保存到 leg_parameters.mat');

%% 步骤2：生成雅可比矩阵拟合参数（第一次运行需要）
disp('=== 生成雅可比矩阵拟合参数 ===');
generate_jacobian_fit();
disp('拟合参数生成完成！');

%% 步骤3：测试计算示例
disp('=== 测试计算示例 ===');
l1 =parameters.l1;    % 右腿第一段长度 [m]
l2 =parameters.l2;    % 右腿第二段长度 [m]  
l3 =parameters.l3;    % 左腿第一段长度 [m]
l4 =parameters.l4;    % 左腿第二段长度 [m]
l5 =parameters.l5;    % 髋关节间距 [m]
% 示例1：正常位置
test_l = 0.15;          % [m] 腿长
test_theta = 0*180/pi;        % [°] 角度（垂直向下为0°）
test_theta_rad = test_theta * pi/180;
%[phi1, phi2] = solve_kinematics_accurate(test_l, test_theta_rad);
[J1 ,J2 ,J3, J4] = calculate_jacobian_elements(test_l, test_theta_rad);
J_inv=[J1 ,J2 ;J3, J4];
J=inv(J_inv);
J_transpose=J';
J1 = J_transpose(1,1);
J2 = J_transpose(1,2);
J3 = J_transpose(2,1);
J4 = J_transpose(2,2);
fprintf('示例1 - l=%.3fm, θ=%.1f°:\n', test_l, test_theta);
disp('雅可比转置矩阵 J = ');
disp([J1 ,J2 ,J3, J4]);

% 示例2：另一个位置
test_l2 = 0.15;
test_theta2 = 0.156*180/pi;
test_theta_rad2 = test_theta2 * pi/180;
[phi1, phi2] = solve_kinematics_accurate(test_l2, test_theta_rad2);
[J1 ,J2, J3, J4] = calculate_jacobian_elements(test_l2, test_theta_rad2);
J_inv=[J1 ,J2 ;J3, J4];
J=inv(J_inv);
J_transpose=J';
J1 = J_transpose(1,1);
J2 = J_transpose(1,2);
J3 = J_transpose(2,1);
J4 = J_transpose(2,2);
fprintf('\n示例2 - l=%.3fm, θ=%.1f°:\n', test_l2, test_theta2);
disp('雅可比转置矩阵 J = ');
disp([J1 ,J2 ,J3, J4]);


%% 步骤4：计算关节力矩示例

disp('============');
disp('============');
disp('============');
disp('============');
disp('=== 关节力矩计算示例1 ===');

% 虚拟腿受力 [Fl; Fp]
F_virtual = [40;0];   % [N]

% 计算关节力矩
tau = J_transpose * F_virtual;

fprintf('虚拟腿受力 F = [%.1f; %.1f] N\n', F_virtual(1), F_virtual(2));
fprintf('关节力矩 tau = [%.3f; %.3f] N·m\n', tau(1), tau(2));



disp('=== 关节力矩计算示例2 ===');

% 虚拟腿受力 [Fl; Fp]
F_virtual = [0;30];   % [N]

% 计算关节力矩
tau = J_transpose * F_virtual;

fprintf('虚拟腿受力 F = [%.1f; %.1f] N\n', F_virtual(1), F_virtual(2));
fprintf('关节力矩 tau = [%.3f; %.3f] N·m\n', tau(1), tau(2));

disp('=== 关节力矩计算示例3 ===');

% 虚拟腿受力 [Fl; Fp]
F_virtual = [400;30];   % [N]

% 计算关节力矩
tau = J_transpose * F_virtual;

fprintf('虚拟腿受力 F = [%.1f; %.1f] N\n', F_virtual(1), F_virtual(2));
fprintf('关节力矩 tau = [%.3f; %.3f] N·m\n', tau(1), tau(2));


disp('=== 程序完成 ===');
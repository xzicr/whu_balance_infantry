% solve_kinematics.m
% 五杆机构运动学逆解 - 正确求解髋关节角度
% 这是以左边腿部作为参照面计算出来的电机角度

function [phi3, phi4] = solve_kinematics_accurate(l, theta)
    % 加载机构参数
    load('leg_parameters.mat', 'parameters');
    l1 = parameters.l1; l2 = parameters.l2;
    l3 = parameters.l3; l4 = parameters.l4;
    l5 = parameters.l5;
    
    % 计算足端坐标（X向右，Y向下）
    x_e = l * sin(theta);
    y_e = l * cos(theta);
    
    % 髋关节位置
    x_hip_right = l5/2;   % 右髋关节（三号电机）
    x_hip_left = -l5/2;   % 左髋关节（四号电机）
    y_hip = 0;
    
    % 求解三号电机角度（右腿髋关节角度）
    dx_r = x_e - x_hip_right;
    dy_r = y_e - y_hip;
    L_r = sqrt(dx_r^2 + dy_r^2);
    
    % 检查可达性
    if L_r > (l1 + l2) || L_r < abs(l1 - l2)
        error('右腿位置不可达: l=%.3fm, θ=%.1f°', l, theta*180/pi);
    end
    
    % 正确的髋关节角度计算：连杆l1与垂直方向的夹角
    % 使用余弦定理：cos(phi3) = (l1² + L_r² - l2²) / (2*l1*L_r)
    cos_phi3 = (l1^2 + L_r^2 - l2^2) / (2 * l1 * L_r);
    phi3 = acos(cos_phi3);
    
    % 基础角度：足端相对于右髋的方向
    base_angle_r = atan2(dy_r, dx_r);
    
    % 最终的1号电机角度
    phi3 = pi- base_angle_r + phi3;  % elbow-down配置
    
    
    % 求解2号电机角度（左腿髋关节角度）
    dx_l = x_e - x_hip_left;
    dy_l = y_e - y_hip;
    L_l = sqrt(dx_l^2 + dy_l^2);
    
    if L_l > (l3 + l4) || L_l < abs(l3 - l4)
        error('左腿位置不可达: l=%.3fm, θ=%.1f°', l, theta*180/pi);
    end
    
    % 正确的髋关节角度计算
    cos_phi4 = (l4^2 + L_l^2 - l3^2) / (2 * l4 * L_l);
    phi4 = acos(cos_phi4);
    
    % 基础角度：足端相对于左髋的方向
    base_angle_l = atan2(dy_l, dx_l);
    
    % 最终的四号电机角度
    phi4 = base_angle_l + phi4;  % elbow-down配置
    
    
    % 转换为度数
    phi3_deg = phi3 * 180/pi;
    phi4_deg = phi4 * 180/pi;
    
    fprintf('虚拟腿: l=%.3fm, θ=%.1f°\n', l, theta*180/pi);
    fprintf('足端位置: x=%.3fm, y=%.3fm\n', x_e, y_e);
    fprintf('1号电机角度: %.6f rad = %.3f°\n', phi3, phi3_deg);
    fprintf('2号电机角度: %.6f rad = %.3f°\n', phi4, phi4_deg);
    fprintf('右髋到足端距离: %.3fm\n', L_r);
    fprintf('左髋到足端距离: %.3fm\n', L_l);
end
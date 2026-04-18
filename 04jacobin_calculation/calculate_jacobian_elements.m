function [J11, J12, J21, J22] = calculate_jacobian_elements(l_val, theta_val)
    % 计算雅可比矩阵元素：使用相位展开消除 atan2 跳变
    % 输入: l_val - 虚拟腿长度, theta_val - 虚拟腿摆角(rad)
    % 输出: J11,J12,J21,J22 - 雅可比转置矩阵元素

    persistent last_phi3 last_phi4 first_call
    if isempty(first_call)
        first_call = true;   % 第一次调用时不展开，直接记录
        last_phi3 = 0;
        last_phi4 = 0;
    end

    delta = 1e-3;   % 数值微分步长

    % 内嵌函数：对角度进行相位展开
    function [phi3_u, phi4_u] = unwrap_angles(phi3, phi4)
        if first_call
            phi3_u = phi3;
            phi4_u = phi4;
            first_call = false;
        else
            phi3_u = phi3 + 2*pi * round((last_phi3 - phi3) / (2*pi));
            phi4_u = phi4 + 2*pi * round((last_phi4 - phi4) / (2*pi));
        end
        last_phi3 = phi3_u;
        last_phi4 = phi4_u;
    end

    % ---- 计算对 l 的偏导数 ----
    [phi3_l_plus, phi4_l_plus] = solve_kinematics_accurate(l_val + delta, theta_val);
    [phi3_l_plus, phi4_l_plus] = unwrap_angles(phi3_l_plus, phi4_l_plus);

    [phi3_l_minus, phi4_l_minus] = solve_kinematics_accurate(l_val - delta, theta_val);
    [phi3_l_minus, phi4_l_minus] = unwrap_angles(phi3_l_minus, phi4_l_minus);

    % ---- 计算对 theta 的偏导数 ----
    [phi3_theta_plus, phi4_theta_plus] = solve_kinematics_accurate(l_val, theta_val + delta);
    [phi3_theta_plus, phi4_theta_plus] = unwrap_angles(phi3_theta_plus, phi4_theta_plus);

    [phi3_theta_minus, phi4_theta_minus] = solve_kinematics_accurate(l_val, theta_val - delta);
    [phi3_theta_minus, phi4_theta_minus] = unwrap_angles(phi3_theta_minus, phi4_theta_minus);

    % ---- 中心差分求导 ----
    J11 = (phi3_l_plus - phi3_l_minus) / (2*delta);
    J21 = (phi4_l_plus - phi4_l_minus) / (2*delta);
    J12 = (phi3_theta_plus - phi3_theta_minus) / (2*delta);
    J22 = (phi4_theta_plus - phi4_theta_minus) / (2*delta);
end
function [J11, J12, J21, J22] = calculate_jacobian_elements(l_val, theta_val)
    % 计算雅可比矩阵元素：φ3和φ4对l和θ的偏导数
    % 使用数值微分方法（中心差分）
    
    % 设置微小变化量
    delta = 1e-3;
    
    % 计算对l的偏导数 (∂φ/∂l)
    [phi3_l_plus, phi4_l_plus] = solve_kinematics_accurate(l_val + delta, theta_val);
    [phi3_l_minus, phi4_l_minus] = solve_kinematics_accurate(l_val - delta, theta_val);
    
    J11 = (phi3_l_plus - phi3_l_minus) / (2 * delta);  % ∂φ3/∂l
    J21 = (phi4_l_plus - phi4_l_minus) / (2 * delta);  % ∂φ4/∂l
    
    % 计算对θ的偏导数 (∂φ/∂θ)
    [phi3_theta_plus, phi4_theta_plus] = solve_kinematics_accurate(l_val, theta_val + delta);
    [phi3_theta_minus, phi4_theta_minus] = solve_kinematics_accurate(l_val, theta_val - delta);
    
    J12 = (phi3_theta_plus - phi3_theta_minus) / (2 * delta);  % ∂φ3/∂θ
    J22 = (phi4_theta_plus - phi4_theta_minus) / (2 * delta);  % ∂φ4/∂θ
end
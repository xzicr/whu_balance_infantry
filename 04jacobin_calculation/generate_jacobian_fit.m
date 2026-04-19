% generate_jacobian_fit.m
% 生成雅可比矩阵的拟合参数（使用髋关节内角）

function generate_jacobian_fit()
    % 加载机构参数
    load('leg_parameters.mat', 'parameters');
    l1 = parameters.l1; l2 = parameters.l2;
    l3 = parameters.l3; l4 = parameters.l4;
    l5 = parameters.l5;
    
    % 定义采样范围
    l_samples = linspace(0.10, 0.40, 150);
    theta_samples = linspace(-60, 60, 150) * pi/180;
    
    [L_grid, Theta_grid] = meshgrid(l_samples, theta_samples);
    
    % 预分配内存
    J11 = zeros(size(L_grid));
    J12 = zeros(size(L_grid));
    J21 = zeros(size(L_grid));
    J22 = zeros(size(L_grid));
    
    fprintf('正在计算%d个点的雅可比矩阵...\n', numel(L_grid));
    
    % 计算每个点的雅可比矩阵
    for i = 1:numel(L_grid)
        l_val = L_grid(i);
        theta_val = Theta_grid(i);
        
        % 显示进度
        if mod(i, 1000) == 0
            fprintf('进度: %.1f%%\n', i/numel(L_grid)*100);
        end
        
        try
            % 求解髋关节内角
            %[phi1, phi2] = solve_kinematics_accurate(l_val, theta_val);
            
%           % 计算中间变量（使用髋关节内角）
%           x_e = l_val * sin(theta_val);
%           y_e = l_val * cos(theta_val);
%           
%           % 计算连杆端点位置（使用髋关节内角）
%           x_1 = x_hip_right + l1 * sin(phi1);  % 注意：使用sin因为phi1是内角
%           y_1 = l1 * cos(phi1);
%           
%           x_2 = x_hip_left + l3 * sin(phi2);
%           y_2 = l3 * cos(phi2);
%           
%           % 计算雅可比矩阵元素（修正公式）
%           % 这里需要根据您的具体雅可比矩阵公式调整
%           denom1 = l1 * sin(phi1);
%           denom2 = l3 * sin(phi2);
%           
%           if abs(denom1) < 1e-6, denom1 = 1e-6; end
%           if abs(denom2) < 1e-6, denom2 = 1e-6; end
%           
%           J11(i) = (x_e + x_1) / denom1;
%           J12(i) = (y_e - y_1) / denom1;
%           J21(i) = (x_e - x_2) / denom2;
%           J22(i) = (y_e - y_2) / denom2;
            [J11_val,J12_val,J21_val,J22_val] =calculate_jacobian_elements(l_val, theta_val);
            %求得逆矩阵
            J_inv = [J11_val,J12_val;
                     J21_val,J22_val];
            %真正雅克比矩阵
            J =inv(J_inv);
            %J的转置
            J_transpose =J';

            J11(i) = J_transpose(1,1);
            J12(i) = J_transpose(1,2);
            J21(i) = J_transpose(2,1);
            J22(i) = J_transpose(2,2);
        catch
            % 不可达位置设为NaN
            J11(i) = NaN; J12(i) = NaN;
            J21(i) = NaN; J22(i) = NaN;
        end
    end
    
    % 移除无效点
    valid_idx = ~isnan(J11);
    L_valid = L_grid(valid_idx);
    Theta_valid = Theta_grid(valid_idx);
    
    fprintf('有效数据点: %d\n', sum(valid_idx));
    fprintf('正在进行多项式拟合...\n');
    
    % 确保有足够数据点
    if sum(valid_idx) < 10
        fprintf('数据点不足，使用线性拟合...\n');
        fit_type = 'poly11';
    else
        fit_type = 'poly22';
    end
    
    % 使用多项式拟合
    fit_J11 = fit([L_valid(:), Theta_valid(:)], J11(valid_idx), fit_type);
    fit_J12 = fit([L_valid(:), Theta_valid(:)], J12(valid_idx), fit_type);
    fit_J21 = fit([L_valid(:), Theta_valid(:)], J21(valid_idx), fit_type);
    fit_J22 = fit([L_valid(:), Theta_valid(:)], J22(valid_idx), fit_type);
    
    % 保存拟合参数
    save('jacobian_fit_params.mat', 'fit_J11', 'fit_J12', 'fit_J21', 'fit_J22');
    
    fprintf('雅可比矩阵拟合参数已保存\n');
end
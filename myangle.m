function angle = myangle(x1, y1, x2, y2)
% myangle - 计算两个向量的夹角
% 输入: (x1,y1) 和 (x2,y2) 两个向量
% 输出: 夹角（弧度）

% 计算点积
dot_product = x1 * x2 + y1 * y2;

% 计算模长
mag1 = sqrt(x1^2 + y1^2);
mag2 = sqrt(x2^2 + y2^2);

% 避免除零
if mag1 < 1e-10 || mag2 < 1e-10
    angle = 0;
    return;
end

% 计算夹角
cos_angle = dot_product / (mag1 * mag2);
cos_angle = max(-1, min(1, cos_angle));  % 限制在[-1, 1]范围内
angle = acos(cos_angle);
end

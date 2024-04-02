% 定义仿真参数
num_samples = 1000;  % 仿真的样本数量
lambda = 0.5;        % 泊松分布的参数（虚警率）

% 生成泊松分布的虚警信号
num_false_alarms = poissrnd(lambda, 1, num_samples);

% 可视化虚警信号
figure;
hist(num_false_alarms, 0:max(num_false_alarms));
xlabel('虚警次数');
ylabel('频次');
title('泊松分布的虚警信号');



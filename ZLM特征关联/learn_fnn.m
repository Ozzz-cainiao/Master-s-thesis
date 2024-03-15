% 准备数据
% 特征数据
feature3 = {5, {320, 455, 560, 730, 890}, 420, 2, 2};

% 模糊化处理
num_lines = feature3{1};
spectral_freq = cell2mat(feature3{2});
num_blades = feature3{3};
harmonic_num = feature3{5};

% 对每个特征进行高斯模糊化处理
num_lines_fuzzy = gaussmf(num_lines, [1, mean(num_lines)]);
spectral_freq_fuzzy = gaussmf(spectral_freq, [1, mean(spectral_freq)]);
num_blades_fuzzy = gaussmf(num_blades, [1, mean(num_blades)]);
harmonic_num_fuzzy = gaussmf(harmonic_num, [1, mean(harmonic_num)]);

% 组合成一个输入向量
input_vector = [num_lines_fuzzy, spectral_freq_fuzzy, num_blades_fuzzy, harmonic_num_fuzzy];

disp(input_vector);



inputs = [1, 2, 3, 4, 5, 6]; % 输入数据，这里假设有6个特征
outputs = [1; 2; 3; 4; 5; 6]; % 输出数据，这里假设输出是一个列向量

% 创建模糊系统
fis = newfis('fuzzysystem');
fis = addvar(fis, 'input', 'Input1', [min(inputs), max(inputs)]);
fis = addvar(fis, 'output', 'Output1', [min(outputs), max(outputs)]);
fis = addmf(fis, 'input', 1, 'mf1', 'gaussmf', [std(inputs), mean(inputs)]);
fis = addmf(fis, 'output', 1, 'mf1', 'gaussmf', [std(outputs), mean(outputs)]);

% 生成模糊化的训练数据
fuzzy_inputs = evalfis(inputs, fis);

% 创建并训练神经网络
net = newff(fuzzy_inputs, outputs, [10]); % 创建一个有10个神经元的隐藏层的神经网络
net = train(net, fuzzy_inputs, outputs);

% 评估网络
outputs_pred = sim(net, fuzzy_inputs); % 使用训练好的网络进行预测

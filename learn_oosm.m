clc;
clear all;
close all;

%% 
q = 0.5;
R = diag([1, 0.1]);
dt = 1;
v = 10;
ekf = trackingEKF(@oneDmotion, @oneDmeas, ...
    'StateTransitionJacobianFcn', @oneDmotionJac, ...
    'MeasurementJacobianFcn', @oneDmeasJac, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', q, ...
    'State', [0;10], ... % x=0, v=10
    'StateCovariance', R,...
    'MeasurementNoise', R);
t = [1, 2, 3, 4, 3.5]; % 生成一系列时间步长t
x = v * t; % 设置对应位置x
allStates = [x; repmat(v, 1, numel(t))]; % 矩阵拼接，生成每个时间步长对应的位置和速度信息。
allMeasurements = oneDmeasWithNoise(allStates, R); % 根据输入的状态信息 allStates 和测量噪声协方差矩阵 R，生成带有噪声的测量值。

neglectEKF = clone(ekf); % Clone the EKF to preserve its initial state.
% 使用循环处理测量值，分布进行预测和校正操作
for i = 1:4
    predict(neglectEKF, dt); % Predict
    correct(neglectEKF, allMeasurements(:,i)); % Correct the filter
end

disp(neglectEKF.StateCovariance);
disp(trace(neglectEKF.StateCovariance));


reprocessingEKF = clone(ekf); % Clone the EKF to preserve its initial state
indices = [1 2 3 5 4]; % Reorder the measurements
for i = 1:numel(indices)
    if i <= 3 % Before t=3
        dt = 1;
    else % For 3 -> 3.5 and 3.5 -> 4
        dt = 0.5;
    end
    predict(reprocessingEKF, dt);
    correct(reprocessingEKF, allMeasurements(:,indices(i)));
end
disp(reprocessingEKF.StateCovariance);
disp(trace(reprocessingEKF.StateCovariance));

retroEKF = clone(ekf); % Clone the EKF to preserve its initial state
dt = 1;
for i = 1:4
    predict(retroEKF, dt); % Predict
    correct(retroEKF, allMeasurements(:,i)); % Correct the filter
end
retrodict(retroEKF,-0.5); % Retrodict from t=4 to t=3.5
retroCorrect(retroEKF, allMeasurements(:,5)); % The measurement at t=3.5
disp(retroEKF.StateCovariance);
disp(trace(retroEKF.StateCovariance));

for lag = 1:4
    timestamps = [0, 1, 2, 3, 4, 4.5-lag];
    allStates = [v*timestamps, repmat(v, 1, numel(timestamps))];
    allMeasurements = oneDmeasWithNoise(allStates, R);
    oneLagStruct(lag) = runOneLagValue(ekf, allMeasurements, timestamps); %#ok<SAGROW> 
end
displayTable(oneLagStruct)


function state = oneDmotion(state, ~, dt)
state = [1 dt;0 1]*state;
end

function [dfdx,dfdv] = oneDmotionJac(~, ~, dt)
dfdx = [1 dt;0 1];
dfdv = chol([dt^3/3 dt^2/2; dt^2/2 dt],'lower');
end

function z = oneDmeas(state)
z = state;
end

function H = oneDmeasJac(state)
H = eye(size(state,1));
end

function z = oneDmeasWithNoise(state,R)
z = state + R * randn(size(R,1), size(state,2));
end

function [x,P] = runNeglect(ekf, allMeasurements, timestamps)
neglectEKF = clone(ekf); % Clone the EKF to preserve its initial state
dt = diff(timestamps);
for i = 1:numel(dt)-1
    predict(neglectEKF, dt(i)); % Predict
    correct(neglectEKF, allMeasurements(:,i)); % Correct the filter
end
x = neglectEKF.State;
P = neglectEKF.StateCovariance;
end

function [x, P] = runReprocessing(ekf, allMeasurements, timestamps)
reprocessingEKF = clone(ekf); % Clone the EKF to preserve its initial state
[timestamps, indices] = sort(timestamps); % Reorder the timestamps
allMeasurements = allMeasurements(:, indices(2:end)-1); % Reorder the measurements
dt = diff(timestamps);
for i = 1:numel(dt)
    predict(reprocessingEKF, dt(i));
    correct(reprocessingEKF, allMeasurements(:,i));
end
x = reprocessingEKF.State;
P = reprocessingEKF.StateCovariance;
end

function [x, P] = runRetrodiction(ekf, allMeasurements, timestamps)
retrodictionEKF = clone(ekf); % Clone the EKF to preserve its initial state
dt = diff(timestamps);
for i = 1:numel(dt)-1
    predict(retrodictionEKF, dt(i));
    correct(retrodictionEKF, allMeasurements(:,i));
end
retrodict(retrodictionEKF, dt(end));
retroCorrect(retrodictionEKF, allMeasurements(:,end));
x = retrodictionEKF.State;
P = retrodictionEKF.StateCovariance;
end

function oneLagStruct = runOneLagValue(ekf, allMeasurements, timestamps)
oneLagStruct = struct('Lag',ceil(timestamps(end-1)-timestamps(end)),...
    'Neglect',zeros(2,2),...
    'Reprocessing',zeros(2,2),...
    'Retrodiction',zeros(2,2));

% Neglect the OOSM
[~, P] = runNeglect(ekf, allMeasurements, timestamps);
oneLagStruct.Neglect = P;

% Reprocess all the measurements according to time
[~, P] = runReprocessing(ekf, allMeasurements, timestamps);
oneLagStruct.Reprocessing = P;
% 
% Use retrodiction 
[~, P] = runRetrodiction(ekf, allMeasurements, timestamps);
oneLagStruct.Retrodiction = P;
end

function displayTable(t)
varNames = fieldnames(t);
fprintf('<strong>%6s  </strong>', 'Lag');
fprintf('<strong>%13s         </strong>', string(varNames(2:4)));
fprintf('\n════════════════════════════════════════════════════════════════════════\n');
for i = 1:numel(t)
    fprintf('    %d', t(i).Lag);
    for j = 2:numel(varNames)
        fprintf('    %c %1.4f %1.4f %c ', 9121, t(i).(varNames{j})(1,1:2), 9124);
    end
    fprintf('\n    ');
    for j = 2:numel(varNames)
        fprintf('     %c %1.4f %1.4f %c', 9123, t(i).(varNames{j})(2,1:2), 9126);
    end
    fprintf('\n');
    fprintf('────────────────────────────────────────────────────────────────────────\n');
end
end
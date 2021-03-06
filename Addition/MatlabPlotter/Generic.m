clear all
clc 
close all

%% 
% fn_path = '/home/apptronik/Repository/PnC/Addition/MatlabPlotter/functions';
% addpath(fn_path)
% data_path = '/home/apptronik/Repository/PnC/ExperimentDataCheck';

fn_path = '/Users/junhyeokahn/Repository/PnC/Addition/MatlabPlotter/functions';
addpath(fn_path)
data_path = '/Users/junhyeokahn/Repository/PnC/ExperimentDataCheck';


%%
targetJointIdx = [1,2,3,4,5];
targetJointIdx = 1:10;
numJoint = 10;
numTarget = length(targetJointIdx);
fig = fn_open_figures(numTarget);

Time = fn_read_file(data_path, 'Time', 1);
JPosDes = fn_read_file(data_path, 'JPosDes', numJoint);
JVelDes = fn_read_file(data_path, 'JVelDes', numJoint);
VirtualJPos = fn_read_file(data_path, 'VirtualJPos', numJoint);
VirtualJVel = fn_read_file(data_path, 'VirtualJVel', numJoint);
JTrqDes = fn_read_file(data_path, 'JTrqDes', numJoint);
JPosAct = fn_read_file(data_path, 'JPosAct', numJoint);
JVelAct = fn_read_file(data_path, 'JVelAct', numJoint);
JTrqAct = fn_read_file(data_path, 'JTrqAct', numJoint);
BusVoltage = fn_read_file(data_path, 'BusVoltage', numJoint);
BusCurrent = fn_read_file(data_path, 'BusCurrent', numJoint);
Temperature = fn_read_file(data_path, 'Temperature', numJoint);
MotorCurrent = fn_read_file(data_path, 'motorCurrent', numJoint);

startIdx = 1;
endIdx = length(Time)-15;
%% Plot

for i = 1 : numTarget
    figure(fig(i))
    subplot(4,2,1)
    hold on
    plot(Time(startIdx:endIdx), JPosDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), VirtualJPos(targetJointIdx(i), startIdx:endIdx),'g', 'linewidth', 2);
    plot(Time(startIdx:endIdx), JPosAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 1);
    hold off
    grid on
    title('jpos')
    subplot(4,2,2)
    plot(Time(startIdx:endIdx), BusVoltage(targetJointIdx(i), startIdx:endIdx).*BusCurrent(targetJointIdx(i), startIdx:endIdx), 'linewidth', 3);
    grid on
    title('electronic power')
    subplot(4,2,3)
    hold on
    plot(Time(startIdx:endIdx), JVelDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), VirtualJVel(targetJointIdx(i), startIdx:endIdx),'g', 'linewidth', 2);
    plot(Time(startIdx:endIdx), JVelAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 1);
    title('joint vel')
    grid on
    hold off
    subplot(4,2,4)
    plot(Time(startIdx:endIdx), JTrqAct(targetJointIdx(i), startIdx:endIdx).*JVelAct(targetJointIdx(i), startIdx:endIdx), 'linewidth', 3);
    title('mechanical power')
    grid on
    subplot(4,2,5)
    hold on
    plot(Time(startIdx:endIdx), JTrqDes(targetJointIdx(i), startIdx:endIdx),'r', 'linewidth', 3);
    plot(Time(startIdx:endIdx), JTrqAct(targetJointIdx(i), startIdx:endIdx),'b', 'linewidth', 1);
    title('jtrq')
    grid on
    hold off
    subplot(4,2,6)
    plot(Time(startIdx:endIdx), (JTrqAct(targetJointIdx(i), startIdx:endIdx).*JVelAct(targetJointIdx(i), startIdx:endIdx))./(BusVoltage(targetJointIdx(i), startIdx:endIdx).*BusCurrent(targetJointIdx(i), startIdx:endIdx)), 'linewidth', 3);
    title('efficiency')
    grid on
    subplot(4,2,7)
    plot(Time(startIdx:endIdx), Temperature(targetJointIdx(i), startIdx:endIdx), 'linewidth', 3);
    title('temperature')
    grid on
    subplot(4,2,8)
    plot(Time(startIdx:endIdx), MotorCurrent(targetJointIdx(i), startIdx:endIdx), 'linewidth', 3);
    title('motor current')
    grid on
end
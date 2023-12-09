%**************************************************************************
% 文件名: E:\坚果云同步文件夹\毕设——非合作多目标定位\FinalCode\AOA\main.m
% 版本: v1.0
% 作者: ZLM
% 联系方式: Liminzhang7@outlook.com
% 日期: 2023-12-09
% 描述: 这是调用精度分析的主程序
% 输入:  
% 输出:  
%**************************************************************************
clc
clear
close all
%% 调用函数
% function []=AccuracyAnalysis(xy,Xmin,Xmax,Ymin,Ymax,Step)
% function [] = Analysis2(arrx, arry, Xmin, Xmax, Ymin, Ymax, Step, errornor)

arrx=[500,1500];
arry=[1000,1000];
% arrx=[1700,300,1700];
% arry=[300,500,1000];
% arrx=[0,2000,2000,0];
% arry=[0,0,2000,2000];
% errornor = [0, 0, (1 / 180 * pi)^2]; %
errornor = [5^2, 5^2, 0]; %

errornor1=(0.2 / 180 * pi)^2;%度换成弧度

Analysis2(arrx,arry,0,2000,0,2000,10,errornor);
% AccuracyAnalysis(list,0,2000,0,2000,10,errornor);
% Analysis(arrx,arry,0,2000,0,2000,10,errornor);
% five(arrx,arry,0,2000,0,2000,10,errornor1);
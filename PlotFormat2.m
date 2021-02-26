% This function simply formats output plots

function [] = PlotFormat2()
Axis = gca;
Axis.TickLabelInterpreter = 'latex';
Axis.LineWidth = 1;
Axis.YTickLabel = {};
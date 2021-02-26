% This function simply formats output plots

function [] = PlotFormat()
Axis = gca;
Axis.TickLabelInterpreter = 'latex';
Axis.LineWidth = 1;
close all
clear all

dirPath = "NavigationError/";
disp("Path: " + dirPath)
run(dirPath+"params.m")
%%
disp("Pre process")
PreProcess_Simulation

%%
disp("h2rb_aapf_method")
ExeSimulation
%%
clear utility
DateString = string(datetime('now','Format','MMddHHmmss'));
save(dirPath+DateString+".mat")
%%
disp("result plotter")
resultPlotter
close all
clear all

derectory = "NavigationError/";
disp("Path: " + derectory)
run(derectory+"params.m")
%%
disp("Pre process")
PreProcess_Simulation

%%
disp("h2rb_aapf_method")
ExeSimulation
%%
clear utility
DateString = string(datetime('now','Format','MMddHHmmss'));
save(derectory+DateString+".mat")

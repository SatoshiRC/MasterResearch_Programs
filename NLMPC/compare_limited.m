%% load simulation result
load("RVF.mat")

rvf_maneuverTime = maneuverTimeMat;
rvf_controlEffort = controlEffortMat;
clear maneuverTimeMat
clear controlEffortMat

load("MPC_RES.mat")

mpc_maneuverTime = maneuverTimeMat;
mpc_controlEffort = controlEffortMat;

diff_maneuverTime = rvf_maneuverTime - mpc_maneuverTime;
diff_controlEffort = rvf_controlEffort - mpc_controlEffort;

%%
f=figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 15 8.5], "Color", "w");
plot(0:0.1:3.7, rvf_controlEffort(11, 38:75), 'o:', 'DisplayName', 'Proposed Method', "Color",[0 200 80]/255, "MarkerSize",5);
hold on
plot(0:0.1:3.7, mpc_controlEffort(11, 38:75), 'x:', 'DisplayName', 'NLMPC', "Color",[0 0 0]/255, "MarkerSize",5);
ylim([0 16])
xlabel("$\omega_{B/H,y}(0)$ [${}^\circ$/s]", "Interpreter", "latex")
ylabel("Control Effort [m/s]", "Interpreter", "latex")
legend("Location", "northwest", "Interpreter", "latex");
% legend("Location", "northoutside", "NumColumns",2);
fontsize(12,"points")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';


f=figure("Name", "control effort", "unit", "centimeter", "position", [0 0 15 8.5], "Color", "w");
plot(0:0.1:3.7, rvf_maneuverTime(12, 38:75), 'o:', 'DisplayName', 'Proposed Method', "Color",[0 200 80]/255, "MarkerSize",5);
hold on
plot(0:0.1:3.7, mpc_maneuverTime(12, 38:75), 'x:', 'DisplayName', 'NLMPC', "Color",[0 0 0]/255, "MarkerSize",5);
ylim([0 600])
legend("Location", "northwest","Interpreter","latex");
% legend("Location", "northoutside", "NumColumns",2);
xlabel("$\omega_{B/H,y}(0)$ [${}^\circ$/s]", "Interpreter", "latex")
ylabel("Maneuver Time [s]", "Interpreter", "latex")
fontsize(12,"points")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

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

f=figure("Name","control effort","Units","centimeters",Position=[10 10 10 7]);
s = surf(-3.7:0.1:3.7,-1:0.1:1,diff_controlEffort,LineStyle="none");
% s.LineWidth = 0.3;
% s.LineStyle = "none";
% s.FaceColor = "none";
% s.Marker = "o";
% s.MarkerFaceColor = "flat";
% s.MarkerEdgeColor = "none";
s.FaceColor = "interp";
daspect([37,10,100])
view(2)
c = colorbar;
c.Label.String = "Control Effort [m/s]";
c.Label.Interpreter = "latex";
c.TickLabelInterpreter = "latex";
xlim([-3.7 3.7]);
ylim([-1 1]);
fontsize(12,"points")
xlabel("$\omega_{B/H, y}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
ylabel("$\omega_{B/H, z}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';


f = figure("Name","Maneuver Time","Units","centimeters",Position=[0 10 10 7]);
s = surf(-3.7:0.1:3.7,-1:0.1:1,diff_maneuverTime,LineStyle="none");

%yticklabels(["-1","0","1"]) 
s.LineWidth = 0.37;
s.FaceColor = "interp";
daspect([37,10,1000])
view(2)
c = colorbar;
c.Label.String = "Maneuver Time [s]";
c.Label.Interpreter = "latex";
c.TickLabelInterpreter = "latex";
% clim([0 1000])
xlim([-3.7 3.7]);
ylim([-1 1]);
fontsize(12,"points")
xlabel("$\omega_{B/H, y}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
ylabel("$\omega_{B/H, z}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

%% boolean
diff_maneuverTime = diff_maneuverTime./abs(diff_maneuverTime);
diff_controlEffort = diff_controlEffort./abs(diff_controlEffort);

positiveIndex = find(diff_maneuverTime' > 0);
negativeIndex = find(diff_maneuverTime' < 0);
x = repmat(-3.7:0.1:3.7,1,1,21);
x = x(1,:,:);
y = repmat(-1:0.1:1,75,1);
f = figure("Name","Maneuver Time","Units","centimeters",Position=[0 10 10 7]);
% scatter(x(positiveIndex), y(positiveIndex),10,"MarkerFaceColor","blue","MarkerEdgeColor","Blue")
% hold on
scatter(x(negativeIndex), y(negativeIndex),10,"MarkerFaceColor","Blue","MarkerEdgeColor","Blue")
daspect([37,10,1000])
view(2)
xlim([-3.7 3.7]);
ylim([-1 1]);
fontsize(12,"points")
xlabel("$\omega_{B/H, y}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
ylabel("$\omega_{B/H, z}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

positiveIndex = find(diff_controlEffort' > 0);
negativeIndex = find(diff_controlEffort' < 0);
x = repmat(-3.7:0.1:3.7,1,1,21);
x = x(1,:,:);
y = repmat(-1:0.1:1,75,1);
f = figure("Name","Control Effort","Units","centimeters",Position=[0 10 10 7]);
% scatter(x(positiveIndex), y(positiveIndex),10,"MarkerFaceColor","blue","MarkerEdgeColor","Blue")
% hold on
scatter(x(negativeIndex), y(negativeIndex),10,"MarkerFaceColor","blue","MarkerEdgeColor","blue")
daspect([37,10,1000])
view(2)
xlim([-3.7 3.7]);
ylim([-1 1]);
fontsize(12,"points")
xlabel("$\omega_{B/H, y}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
ylabel("$\omega_{B/H, z}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
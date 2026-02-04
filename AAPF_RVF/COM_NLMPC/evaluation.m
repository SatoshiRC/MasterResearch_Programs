% dt

% vecnorm_input_log = vecnorm(u);
% controlEffort = sum(vecnorm_input_log,2)*dt;
% controlEffortMat = reshape(controlEffort(1,1,:), 75, 21)';

% for i=1:length(isArrived)
%     if isArrived(i)<=0
%         isArrived(i) = 10000;
%     end
% end
% maneuverTimeMat = reshape(isArrived, 75, 21)'*dt;

load("RVF.mat")

f=figure("Name","control effort","Units","centimeters",Position=[10 10 10 7]);
s = surf(-3.7:0.1:3.7,-1:0.1:1,controlEffortMat,LineStyle="none");
s.LineWidth = 0.3;
% s.LineStyle = "none";
% s.FaceColor = "none";
% s.Marker = "o";
% s.MarkerFaceColor = "flat";
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

%%
f=figure("Name","control effort","Units","centimeters",Position=[10 10 10 7]);
x = linspace(-3.7, 3.7, 75);
y = linspace(-1, 1, 21);
heatmap(x,y,controlEffortMat);
% s.LineWidth = 0.3;
% % s.LineStyle = "none";
% % s.FaceColor = "none";
% % s.Marker = "o";
% % s.MarkerFaceColor = "flat";
% s.FaceColor = "interp";
% view(2)
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
%%

f = figure("Name","Maneuver Time","Units","centimeters",Position=[0 10 10 7]);
s = surf(-3.7:0.1:3.7,-1:0.1:1,maneuverTimeMat,LineStyle="none");

%yticklabels(["-1","0","1"]) 
s.LineWidth = 0.37;
s.FaceColor = "interp";
daspect([37,10,1000])
view(2)
c = colorbar;
c.Label.String = "Maneuver Time [s]";
c.Label.Interpreter = "latex";
c.TickLabelInterpreter = "latex";
clim([0 1000])
xlim([-3.7 3.7]);
ylim([-1 1]);
fontsize(12,"points")
xlabel("$\omega_{B/H, y}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
ylabel("$\omega_{B/H, z}(0) [{}^\circ/\mathrm{s}]$","Interpreter","latex","FontSize",12)
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
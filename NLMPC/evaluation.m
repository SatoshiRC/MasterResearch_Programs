simulation_dt

vecnorm_input_log = vecnorm(input_log);
controlEffort = sum(vecnorm_input_log,2)*simulation_dt;
controlEffortMat = reshape(controlEffort(1,1,:), 75, 21)';

maneuverTimeMat = reshape(index_end, 75, 21)'*simulation_dt;

approachVelocityConstraintViolation = max(constraint_log(9,:,:),[],2);
approachVelocityConstraintViolationMat = reshape(approachVelocityConstraintViolation, 75, 21)';

w_z = -1:0.1:1;
w_y = -3.7:0.1:3.7;
%%
f = figure("Name","control effort","Units","centimeters",Position=[0 10 10 7]);
s = surf(w_y,w_z,controlEffortMat,LineStyle="none");
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

f = figure("Name","Maneuver Time","Units","centimeters",Position=[0 10 10 7]);
s = surf(w_y,w_z,maneuverTimeMat,LineStyle="none");
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

%%
figure("Name","Approach Velocity Constraint")
s = surf(w_y,w_z,approachVelocityConstraintViolationMat);
s.LineWidth = 0.3;
s.FaceColor = "interp";
daspect([37,10,1000])
view(2)
c = colorbar;
c.Label.String = "Approach Velocity [m/s]";
fontsize(12,"points")
xlabel("$\omega_{y,0} [{}^\circ/s]$","Interpreter","latex","FontSize",14)
ylabel("$\omega_{z,0} [{}^\circ/s]$","Interpreter","latex","FontSize",14)
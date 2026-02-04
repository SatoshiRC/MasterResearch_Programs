clear all

load("Case4/proposedMethod.mat");
j = 25;
quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
angularVelocityRow = targetAngularVelocity(:,:,j); 
angularVelocityRowRTN = rotateframe(quaternion_BODY2RTN_, angularVelocityRow) + [0 0 -omega];
rhoRow = rho(:,:,j);
pathRow = path(:,:,j);
veloRow = velo(:,:,j);
% velocity = rotateframe(conj(quaternion_BODY2RTN_),veloRow.').';
pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';
velocity = rotateframe(conj(quaternion_BODY2RTN_), (veloRow.' - cross(angularVelocityRowRTN, pathRow.')))';
% velocity = rotateframe(conj(quaternion_BODY2RTN_), veloRow.').' - cross(angularVelocityRow, pathXYZ.').';
proposedMethod = vecnorm(velocity);
lastIndexProposedMethod = isArrived(j);

load("Case4/conventionalMethod.mat");
j = 25;
quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
angularVelocityRow = targetAngularVelocity(:,:,j); 
angularVelocityRowRTN = rotateframe(quaternion_BODY2RTN_, angularVelocityRow) + [0 0 -omega];
rhoRow = rho(:,:,j);
pathRow = path(:,:,j);
veloRow = velo(:,:,j);
% velocity = rotateframe(conj(quaternion_BODY2RTN),veloRow.').';
pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';
velocity = rotateframe(conj(quaternion_BODY2RTN_), (veloRow.' - cross(angularVelocityRowRTN, pathRow.')))';
% velocity = rotateframe(conj(quaternion_BODY2RTN_), veloRow.').' - cross(angularVelocityRow, pathXYZ.').';
conventionalMethod = vecnorm(velocity);
lastIndexConventionalMethod = 10000;


%%
f=figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 12 9], "Color", "w");
box on
hold on
plot(time(1:lastIndexProposedMethod), proposedMethod(1:lastIndexProposedMethod), '-', 'DisplayName', 'AAPF + RVF', "Color",[0 200 80]/255, "MarkerSize",5);
plot(time(1:lastIndexConventionalMethod), conventionalMethod(1:lastIndexConventionalMethod), '-', 'DisplayName', 'AAPF', "Color",[0 0 0]/255, "MarkerSize",5);
% ylim([0 10])
xlabel('Time [s]',Interpreter='latex')
ylabel('Velocity [m/s]',Interpreter='latex')
legend("Position", [0.34 0.82 0.5 0.08], "NumColumns",2, Interpreter="latex");
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
fontsize(12,"points")
grid on

% figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 4 3], "Color", "w");
ax = axes("Position", [0.46 0.38 0.4 0.4]);
box on
hold on
plot(time(1:lastIndexProposedMethod), proposedMethod(1:lastIndexProposedMethod), '-', 'DisplayName', 'AAPF + RVF', "Color",[0 200 80]/255, "MarkerSize",5);
plot(time(1:lastIndexConventionalMethod), conventionalMethod(1:lastIndexConventionalMethod), '-', 'DisplayName', 'AAPF', "Color",[0 0 0]/255, "MarkerSize",5);
ylim([0 0.2])
xlim([100 500]);
xlabel('Time [s]',Interpreter='latex')
ylabel('Velocity [m/s]',Interpreter='latex')
% legend("Location", "northoutside", "NumColumns",2);
ax.FontSize = 12;
ax.XAxis.TickLabelInterpreter = 'latex';
ax.YAxis.TickLabelInterpreter = 'latex';
grid on
load("Case4/proposedMethod.mat");
totalDeltaVProposed = zeros([Ncase 1]);
for j=1:Ncase
    totalDeltaVProposed(j) = sum(vecnorm(u(:,:,j),2,1))*dt;
end
load("Case4/conventionalMethod.mat");
totalDeltaVConventional = zeros([Ncase 1]);
for j=1:Ncase
    totalDeltaVConventional(j) = sum(vecnorm(u(:,:,j),2,1))*dt;
end

f=figure("Name", "control effort", "unit", "centimeter", "position", [0 0 11 7], "Color", "w");
plot(initialAngularVero(:,2), totalDeltaVProposed, 'o:', 'DisplayName', 'AAPF + RVF', "Color",[0 200 80]/255, "MarkerSize",5);
hold on
plot(initialAngularVero(:,2), totalDeltaVConventional, 'x:', 'DisplayName', 'AAPF', "Color",[0 0 0]/255, "MarkerSize",5);
% ylim([0 10])
legend("Location", "northwest",Interpreter="latex");
% legend("Location", "northoutside", "NumColumns",2);
xlabel("$\omega_{B/H ,y}(0)$ [${}^\circ$/s]", "Interpreter", "latex")
ylabel("Control Effort [m/s]", "Interpreter", "latex")
fontsize(12,"points")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

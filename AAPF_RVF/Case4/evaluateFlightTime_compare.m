load("Case4/proposedMethod.mat");
proposedMethod = zeros([Ncase 1]);
for j=1:Ncase
    if isArrived(j) > 0
        proposedMethod(j) = isArrived(j)*dt;
    else
        proposedMethod(j) = 1000;
    end
end
load("Case4/conventionalMethod.mat");
conventionalMethod = zeros([Ncase 1]);
for j=1:Ncase
    if isArrived(j) > 0
        conventionalMethod(j) = isArrived(j)*dt;
    else
        conventionalMethod(j) = 1000;
    end
end

f=figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 11 7], "Color", "w");
plot(initialAngularVero(:,2), proposedMethod, 'o:', 'DisplayName', 'AAPF + RVF', "Color",[0 200 80]/255, "MarkerSize",5);
hold on
plot(initialAngularVero(:,2), conventionalMethod, 'x:', 'DisplayName', 'AAPF', "Color",[0 0 0]/255, "MarkerSize",5);
% ylim([0 10])
xlabel("$\omega_{B/H ,y}(0)$ [${}^\circ$/s]", "Interpreter", "latex")
ylabel("Maneuver Time [s]", "Interpreter", "latex")
legend("Location", "northwest", "Interpreter", "latex");
ylim([0 1000])
% legend("Location", "northoutside", "NumColumns",2);
fontsize(12,"points")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

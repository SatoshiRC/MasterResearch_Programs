maneuverTime = zeros([Ncase 1]);
for j=1:Ncase
    if isArrived(j) > 0
        maneuverTime(j) = isArrived(j)*dt;
    else
        maneuverTime(j) = 1000;
    end
end

f=figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 13 10], "Color", "w");
plot(1:1:7,maneuverTime(1:Ncase), 'o:','MarkerEdgeColor',[0 0 0], 'Color',[0 0 0], 'MarkerFaceColor',[0 0 0]);
% ylim([0 300])
xlabel("Index [-]", "Interpreter", "latex")
ylabel("Maneuver Time [s]", "Interpreter", "latex")
ylim([0, 2000]);
fontsize(12,"points")
% legend("Location", "northwest");
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
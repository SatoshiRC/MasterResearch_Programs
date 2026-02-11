maneuverTime = zeros([Ncase 1]);
for j=1:Ncase
    if isArrived(j) > 0
        maneuverTime(j) = isArrived(j)*dt;
    else
        maneuverTime(j) = 1000;
    end
end

figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 13 10], "Color", "w");
plot(initialAngularVelo(:,2),maneuverTime(1:Ncase), 'o:','MarkerEdgeColor',[0 0 1], 'Color',[0 0 0]);
% ylim([0 300])
xlabel("$\omega_{t,y}(0)$ [m]", "Interpreter", "latex")
ylabel("Maneuver Time [s]")
fontsize(12,"points")
% legend("Location", "northwest");
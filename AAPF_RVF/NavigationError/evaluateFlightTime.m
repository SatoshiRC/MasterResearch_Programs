% load("case_debug/proposedMethod.mat");
simCase = 100;
maneuverTimeMat = zeros([simCase Ncase/simCase]);
for j=1:Ncase
    if isArrived(j) > 0
        maneuverTimeMat(j) = isArrived(j)*dt;
    else
        maneuverTimeMat(j) = 1000;
    end
end

%%
load("Case4/proposedMethod.mat")
maneuverTimeRef = zeros([Ncase 1]);
for j=1:Ncase
    if isArrived(j) > 0
        maneuverTimeRef(j) = isArrived(j)*dt;
    else
        maneuverTimeRef(j) = 1000;
    end
end
%%
f=figure("Name", "time of flight", "unit", "centimeter", "position", [0 0 15 8.5], "Color", "w");
boxchart(maneuverTimeMat,"MarkerStyle",".","MarkerColor","black","LineWidth",1,"BoxWidth",0.6)
hold on
plot(maneuverTimeRef, "LineStyle",":", "Color","k","Marker","x")
box on
xlabel("$\omega_{B/H,y}(0)$ [${}^\circ$/s]", "Interpreter", "latex")
ylabel("Maneuver Time [s]", Interpreter="latex")
ylim([0 700]);
xticklabels(["0","","","","","0.5","","","","","1.0","","","","","1.5","","","","","2.0","","","","","2.5","","","","","3.0","","","","","3.5","",""]) 
f.CurrentAxes.XTickLabelRotation = 0;
fontsize(12,"points")
legend("Navigation Error", "Reference",Location="northwest",Interpreter = 'latex')
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
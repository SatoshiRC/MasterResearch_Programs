load("case_debug/proposedMethod.mat");
simCase = 100;
totalDeltaV = zeros([simCase Ncase/simCase]);
for j=1:Ncase
    totalDeltaV(j) = sum(vecnorm(u(:,:,j),2,1))*dt;
end

%%
load("Case4/proposedMethod.mat")
totalDeltaVProposed = zeros([Ncase 1]);
for j=1:Ncase
    totalDeltaVProposed(j) = sum(vecnorm(u(:,:,j),2,1))*dt;
end

%%
f=figure("Name", "delta-v", "unit", "centimeter", "position", [0 0 15 8.5], "Color", "w");
boxchart(totalDeltaV,"MarkerStyle",".","MarkerColor","black","LineWidth",1,"BoxWidth",0.6)
hold on
box on
plot(totalDeltaVProposed, "LineStyle",":", "Color","k","Marker","x")
xlabel("$\omega_{B/H,y}(0)$ [${}^\circ$/s]", "Interpreter", "latex")
ylabel("Control Effort [m/s]",Interpreter="latex")
xticklabels(["0","","","","","0.5","","","","","1.0","","","","","1.5","","","","","2.0","","","","","2.5","","","","","3.0","","","","","3.5","",""])
f.CurrentAxes.XTickLabelRotation = 0;
fontsize(12,"points")
legend("Navigation Error", "Reference",Location="northwest",Interpreter = 'latex')
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
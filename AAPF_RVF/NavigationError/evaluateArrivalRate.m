simCase = 100;
isArrivedMat = reshape(isArrived, simCase, [])'>0;

successRate = zeros(1, 38);
for i=1:38
    successRate = sum(isArrivedMat,2)/simCase;
end
%%
figure("Name","Success Rate","Units","centimeters","Position",[-15 0 9 5])
scatter(linspace(0,3.7,38),successRate,"*","MarkerEdgeColor","black")
% plot(successRate)
ylim([0, 1.1])
xlabel("$\omega_{t,y}(0)$ [deg/s]", "Interpreter", "latex")
ylabel("Arrival Rate [-]")
fontsize(12,"points")

totalDeltaV = zeros([Ncase 1]);
for j=1:Ncase
    totalDeltaV(j) = sum(vecnorm(u(:,:,j),2,1))*dt;
end

figure("Name", "control effort", "unit", "centimeter", "position", [0 0 13 10], "Color", "w");
plot(totalDeltaV, 'x:', 'DisplayName', 'AAPF', "Color",[0 0 0]/255, "MarkerSize",5);
% ylim([0 10])
% legend("Location", "northwest");
% legend("Location", "northoutside", "NumColumns",2);
xlabel("Index")
ylabel("DeltaV [m/s]")
fontsize(12,"points")
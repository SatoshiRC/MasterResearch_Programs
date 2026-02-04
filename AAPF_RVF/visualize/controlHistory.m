uRow = u(:,:,j);

figure('Name',"control values",'unit','centimeter','Position', [0 0 8 4],'Color','white')
hold on
plot(time(1:lastIndex),uRow(1,1:lastIndex));
plot(time(1:lastIndex),uRow(2,1:lastIndex));
plot(time(1:lastIndex),uRow(3,1:lastIndex));
xlabel('Time [s]')
ylabel('u [m/s^2]')
% xlim([0 450])
fontsize(8,"points")
leg = legend("$u_x$","$u_y$","$u_z$");
leg.FontSize = 8;
leg.Interpreter = "latex";
grid on
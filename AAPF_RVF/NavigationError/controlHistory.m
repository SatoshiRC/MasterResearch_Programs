j=1;
uRow = u(:,:,j);
lastIndex = isArrived(j);

f=figure('Name',"control values",'unit','centimeter','Position', [0 0 15 6],'Color','white');
plot(time(1:lastIndex),uRow(1,1:lastIndex));
hold on
plot(time(1:lastIndex),uRow(2,1:lastIndex));
plot(time(1:lastIndex),uRow(3,1:lastIndex));
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('Control [$\mathrm{m/s}^2$]', 'Interpreter', 'latex')
xlim([0 250])
ylim([-0.06 0.06])
fontsize(8,"points")
leg = legend("$u_x$","$u_y$","$u_z$","NumColumns",3, "Location","northoutside");
leg.Interpreter = "latex";
fontsize(12,"points")
grid on
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
uRow = u(:,:,j);

f=figure('Name',"control values",'unit','centimeter','Position', [0 0 8 4],'Color','white');
plot(time(1:lastIndex),uRow(1,1:lastIndex));
hold on
plot(time(1:lastIndex),uRow(3,1:lastIndex));
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('Control [$\mathrm{m/s}^2$]', 'Interpreter', 'latex')
xlim([0 500])
ylim([-0.06 0.06])
fontsize(8,"points")
leg = legend("$u_x$","$u_z$","NumColumns",2,"location","northwest");
leg.Interpreter = "latex";
fontsize(12,"points")
grid on
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
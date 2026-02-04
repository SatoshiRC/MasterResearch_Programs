pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';

f = figure('Name',"position",'unit','centimeter','Position', [0 0 8 4],'Color','white');
plot(time(1:lastIndex),pathXYZ(1,1:lastIndex));
hold on
plot(time(1:lastIndex),pathXYZ(3,1:lastIndex));
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('Position [m]', 'Interpreter', 'latex')
xlim([0 500])
leg = legend("$r_{ct, x}$","$r_{ct, z}$", "NumColumns", 2, "Location", "Southeast");
leg.Interpreter = "latex";
fontsize(12,"points")
grid on
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

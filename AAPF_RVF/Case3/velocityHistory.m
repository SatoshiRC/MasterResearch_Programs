pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';
velocity = rotateframe(conj(quaternion_BODY2RTN_), veloRow.').' - cross(angularVelocityRow, pathXYZ.').';

f=figure('Name',"velocity",'unit','centimeter','Position', [0 0 8 4],'Color','white');
plot(time(2:lastIndex),velocity(1,2:lastIndex));
hold on
plot(time(2:lastIndex),velocity(3,2:lastIndex));
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('Velocity [m/s]', 'Interpreter', 'latex')
xlim([0 500])
leg = legend("$\dot{r}_{ct, x}$","$\dot{r}_{ct, z}$","Location","southeast", "NumColumns", 2);
leg.Interpreter = "latex";
fontsize(12,"points")
grid on
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';
velocity = rotateframe(conj(quaternion_BODY2RTN_), veloRow.').' - cross(angularVelocityRow, pathXYZ.').';

figure('Name',"velocity",'unit','centimeter','Position', [0 0 8 4],'Color','white')
hold on
plot(time(2:lastIndex),velocity(1,2:lastIndex));
plot(time(2:lastIndex),velocity(2,2:lastIndex));
plot(time(2:lastIndex),velocity(3,2:lastIndex));
xlabel('Time [s]')
ylabel('Velocity [m/s]')
% xlim([0 450])
leg = legend("$\dot{r}_{ct, x}$","$\dot{r}_{ct, y}$","$\dot{r}_{ct, z}$","Location","northeast");
leg.FontSize = 8;
leg.Interpreter = "latex";
fontsize(8,"points")
grid on
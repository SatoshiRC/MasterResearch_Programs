pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';

figure('Name',"position",'unit','centimeter','Position', [0 0 8 4],'Color','white')
hold on
plot(time(1:lastIndex),pathXYZ(1,1:lastIndex));
plot(time(1:lastIndex),pathXYZ(2,1:lastIndex));
plot(time(1:lastIndex),pathXYZ(3,1:lastIndex));
xlabel('Time [s]')
ylabel('Position [m]')
% xlim([0 450])
leg = legend("$r_{ct, x}$","$r_{ct, y}$","$r_{ct, z}$");
leg.FontSize = 8;
leg.Interpreter = "latex";
fontsize(8,"points")
grid on

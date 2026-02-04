pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';

figure('Name',"position",'unit','centimeter','Position', [0 0 15 6],'Color','white')
plot(time(1:lastIndex),pathXYZ(1,1:lastIndex));
hold on
plot(time(1:lastIndex),pathXYZ(3,1:lastIndex));
xlabel('Time $[\mathrm{s}]$',"Interpreter","latex")
ylabel('Position $[\mathrm{m}]$',"Interpreter","latex")
% xlim([0 450])
ylim([-30 10])
leg = legend("$r_{ct, x}$","$r_{ct, z}$","Location","northoutside","NumColumns",3);
leg.FontSize = 8;
leg.Interpreter = "latex";
fontsize(12,"points")
grid on

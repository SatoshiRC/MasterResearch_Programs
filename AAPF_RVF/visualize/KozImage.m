pathXYZ = rotateframe(conj(quaternion_BODY2RTN),pathRow.');
[boundary]= target.koz([0 0 1], [1 0 0]);
figure();
ax = axes; % 新しいAxesを作成
set(ax, 'Color', 'k'); % Axes
hold on
xsize = 6.5;
ysize = xsize / 800 * 500;
image([-xsize xsize],[-ysize ysize],imread("../materials/project_fig_005.png"))
plot(boundary(:,1),boundary(:,2),Color="red");
plot(pathXYZ(1:isArrived(j),3),pathXYZ(1:isArrived(j),1),Color="green");

xlim([-30 30])
ylim([-30 30])
xlabel("x(m)")
ylabel("y(m)")
daspect([1 1 1])
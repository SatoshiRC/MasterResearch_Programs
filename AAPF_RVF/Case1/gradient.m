i = 1;


frame = figure('Units','centimeters','Position',[20 10 10 10]);

x = linspace(-30, 30, 11);
z = linspace(-30, 30, 11);
[X, Z] = meshgrid(x,z);
nabla_phi_ = zeros([size(X) 3]);
nabla_phi = zeros([ceil((length(time))/100) size(X) 3]);

originalVec = [0 0 1]*2; ... BODY frame

%visualization
attractivePoseRT = rotateframe(quaternion_BODY2RTN(i),attractivePose.').';

rho_ = rhoRow(:,i);
R = [rho_(1) rho_(2) rho_(3);
    0 rho_(4) rho_(5);
    0 0 rho_(6)];
attractiveShapingMatrix = R'*R;

positionXY = rotateframe(conj(quaternion_BODY2RTN(i)),pathRow(:,i).').';
for m=1:length(x)
    nabla_phi_j = zeros([1 length(x) 3]);
    x_j = x(m);
    position = zeros([1 3]);
    for k=1:length(z)
        position(1) = x_j;
        position(2) = 0;
        position(3) = z(k);

        r_ct = position' - attractivePose;
        r_cb_ = target.r_cb(position);
        
        if r_cb_ == 0
            nabla_phi_j(1,k,:) = -attractiveShapingMatrix*r_ct;
        else
            nabla_phi_j(1,k,:) = -attractiveShapingMatrix*r_ct;
        end
    end
    nabla_phi_(m,:,:)=nabla_phi_j;
end

pafVec = [0 0 6.5];
[boundary_RTN]= target.koz([0 1 0], pafVec);

hold off
g = quiver(Z,X,nabla_phi_(:,:,1),nabla_phi_(:,:,3),1.5,'k');
hold on
b = plot(boundary_RTN(:,1),boundary_RTN(:,3),Color="red");
% plot(path(1,1:i+1),path(2,1:i+1),Color="black",LineWidth=2)
% scatter(attractivePoseRT(1), attractivePoseRT(2),LineWidth=2,MarkerEdgeColor="black",Marker="x",SizeData=100);
% scatter(pathRow(1,i),pathRow(2,i),LineWidth=2,MarkerEdgeColor="black",Marker=".",SizeData=100);
legend([g, b], ["$-\nabla\phi_\mathrm{a}$" "KOZ"], "Interpreter", 'latex')
xlabel("x [m]","Interpreter","latex")
ylabel("z [m]","Interpreter","latex")
xlim([-30 30])
ylim([-30 30])
daspect([1 1 1])
frame.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
frame.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
fontsize(12,"points")
%%

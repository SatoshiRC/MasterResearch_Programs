arrivedIndex = find(isArrived > 0);
notArrivedIndex = find(isArrived == 0);
errorIndex = find(isArrived < 0);
%radius = 2*iniatialUpperAngularVero(1)*sqrt(length(arrivedIndex)/(pi*Ncase));


%%
% figure('Position', [100 100 500 550],'Color','white')
% hold on

% scatterA = scatter(angularVeloRandom(arrivedIndex,1),angularVeloRandom(arrivedIndex,2),"red",'DisplayName','Arrived');
% scatterB = scatter(angularVeloRandom(notArrivedIndex,1),angularVeloRandom(notArrivedIndex,2),"blue",'DisplayName','Not Arrived');
% scatterC = scatter(angularVeloRandom(errorIndex,1),angularVeloRandom(errorIndex,2),"black",'DisplayName','Enter KOZ');
% %circle = viscircles([0 0],radius,'Color','g');
% daspect([1 1 1])
% legend([scatterA, scatterB, scatterC,circle],{'Arrived','Not Arrived','Entered KOZ','Acceptable Area'},'Location','northoutside','Orientation','horizontal','NumColumns',2);
% legend([scatterA, scatterB, scatterC],{'Arrived','Not Arrived','Entered KOZ','Acceptable Area'},'Location','northoutside','Orientation','horizontal','NumColumns',2);

% xlabel('\omega_{bx}(deg/s)')
% ylabel('\omega_{by} (deg/s)')
%%
tic
index = linspace(1,Ncase,Ncase);
minRct = zeros([Ncase 1]);
parfor j=1:Ncase
    pathRow = path(:,:,j).';
    quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);

    r_ct = zeros(size(pathRow));
    r_ctNorm = zeros([1 length(pathRow(:,1))]);
    attractivePose_ = attractivePose;
    positionXY = rotateframe(conj(quaternion_BODY2RTN_(:)),pathRow(:,:)).';
    r_ct = positionXY - attractivePose_;
    r_ctNorm = vecnorm(r_ct);
    minRct(j) = min(r_ctNorm);
end
toc

figure('Color','white')
hold on
scatterA = scatter(index(arrivedIndex),minRct(arrivedIndex),"red");
scatterB = scatter(index(notArrivedIndex),minRct(notArrivedIndex),"blue");
scatterC = scatter(index(errorIndex),minRct(errorIndex),"black");
% title('Distance to the target point')
legend([scatterA, scatterB, scatterC],{'Arrived','Not Arrived','Entered KOZ'},'Location','southoutside','Orientation','horizontal');
xlabel('Index')
ylabel('min(r_{cf}) (m)')

run("visualize/evaluateFlightTime.m")
run("visualize/evaluateControlEffort.m")
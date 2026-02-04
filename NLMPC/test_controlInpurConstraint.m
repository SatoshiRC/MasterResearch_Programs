i = 1;

constraint_log_ = constraint_log(:,:,i);
input_log_ = input_log(:,:,i);
index_end_ = index_end(i);

mpc_dt = 5;
simEx = mpc_dt / simulation_dt;

thrust1 = zeros(1,steps);
thrust2 = vecnorm(input_log_);

for k=1:index_end_
    u_k = input_log_(:,k);

    thrust1(k) = sqrt(u_k'*u_k);
end

%%
figure('Name', 'thrust constraint')
hold on
plot(time(1:index_end_), thrust1(1, 1:index_end_));
plot(time(1:index_end_), thrust2(1, 1:index_end_));
plot(time(1:simEx:index_end_), constraint_log_(9, 1:ceil(index_end_/simEx)),'LineStyle', '--');
legend("Simulate", "vecnorm ","MPC result")
xlabel("time [s]")
ylabel("approach velocity [m/s]")
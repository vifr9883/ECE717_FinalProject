function IAE = compute_IAE(r_hat, r_traj, Ts)
% IAE - Computes the Integrated Absolute Error of the trajectory
% r_hat [3xk]
% r_traj [3xk]
err_norm = vecnorm(r_traj-r_hat);
IAE = sum(err_norm*Ts);
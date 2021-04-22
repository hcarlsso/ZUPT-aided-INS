function jac = numeric_jacobian(f, x)
% Calculate Jacobian of function f at given x
epsilon = 1e-6; 
epsilon_inv = 1/epsilon;
nx = length(x); % Dimension of the input x;
f0 = feval(f, x); % caclulate f0, when no perturbation happens
nf = length(f0);
jac = zeros(nf,nx);
% Do perturbation
for i = 1:nx
    x_ = x;
    x_(i) =  x(i) + epsilon;
    jac(:, i) = (feval(f, x_) - f0) .* epsilon_inv;
end
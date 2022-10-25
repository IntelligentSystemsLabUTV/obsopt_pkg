function [sys_An, num_An, den_An, N, N_An] = annihilator(sys)
%ANNIHILATOR Dynamic annihilator.
%
%   [SYS_AN, W_AN] = ANNIHILATOR computes the dynamic annihilator of the
%   SISO or MIMO linear system SYS:
%
%     W(s) * W_An(s) = 0 -> N(s) * N_An(s) = 0, for each s.
%
%   [SYS_AN, W_AN, N, N_AN] = ANNIHILATOR also returns the cell array 
%   with the coefficients of the polynomials to the numerator of the
%   related systems:
%
%     N(s) = N_0 + N_1 * s + ... + N_n * s^n
%     N_An(s) = N_{An,0} + N_{An,1} * s + ... + N_{An,n} * s^n
%
% Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
% October 2, 2022

% Get the system dimensions
n = size(sys.A, 1);
m = size(sys.B, 2);
p = size(sys.C, 1);
% Compute a left comprime factorization
[N, ~, ~] = lcf(sys);
% Compose the matrix that defines the relationships that 
% the annihilator must satisfy
M = zeros(p * (2^n + 1), m * (n+1));
for k = 1 : n+1
  for h = 1 : n+1
    i1 = (k-1) * p + (h-1) * p + 1;
    i2 = (k-1) * p + h * p;
    j1 = (h-1) * m + 1;
    j2 = h * m;
    M(i1:i2, j1:j2) = N{k};
  end
end
% Compute the null of the matrix
Mperp = null(M);
q = size(Mperp, 2);
if (q == 0)
  error('No annhiliator')
end
% Compose the matrices of coefficients of the annihilator numerator
N_An = cellmat(1, n+1, m, q);
for k = 1 : n+1
  j1 = (k-1) * m + 1;
  j2 = k * m;
  N_An{k} = Mperp(j1:j2, :);
end
% Compose the annihilator numerator
num_An = cellmat(m, q, 1, n+1);
for j = 1 : m
  for i = 1 : q
    for k = 1 : n+1
      num_An{j, i}(n-k+2) = N_An{k}(j, i);
    end
  end
end
% Compose the annihilator denomiantor
sys_eigen = eig(sys.A);
min_eigen = min(real(sys_eigen(real(sys_eigen) < 0)));
scale = 1;
poles = linspace(10 * min_eigen, 100 * min_eigen, n+1)/scale;
den_An = poly(poles);
den_An = den_An ./ den_An(end);
% Compute the annihilator transfer function
W_An = tf(num_An, den_An);
% Compute the annihilator state-space model
sys_An = ss(W_An);

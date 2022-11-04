function [num_An, N, N_An] = annihilator(sys, eta)
%ANNIHILATOR Dynamic annihilator.
%
%   [NUM_AN] = ANNIHILATOR(SYS, ETA) computes the dynamic annihilator of the
%   SISO or MIMO linear system SYS:
%
%     NUM(s) * NUM_An(s) = 0, for each s.
%
%   [NUM_AN, N, N_AN] = ANNIHILATOR(SYS, ETA) also returns the cell array 
%   with the coefficients of the polynomials to the numerator of the
%   related systems:
%
%     N(s) = N_0 + N_1 * s + ... + N_n * s^n
%     N_An(s) = N_An_0 + N_An_1 * s + ... + N_An_eta * s^eta
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
M = zeros(p * (eta+n), m * eta);
for k = 1 : n
  for h = 1 : eta
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
N_An = cellmat(1, eta+1, m, q);
for k = 1 : eta
  j1 = (k-1) * m + 1;
  j2 = k * m;
  N_An{k} = Mperp(j1:j2, :);
end
% Compose the annihilator numerator
num_An = cellmat(m, q, 1, eta+1);
for j = 1 : m
  for i = 1 : q
    for k = 1 : eta+1
      num_An{j, i}(eta-k+2) = N_An{k}(j, i);
    end
  end
end

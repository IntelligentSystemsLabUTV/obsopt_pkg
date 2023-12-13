function [N, d, W] = lcf(sys)
%LCF Left coprime factorization.
%
%   LCF computes a left coprime factorization of the
%   SISO or MIMO linear system SYS:
%
%     W(s) = d^(-1)(s) * N(s),
%
%   where d(s) and N(s) are coprime polynomial matrices.
%
%   [N, d] = LCF(SYS) returns the cell array with the coefficients 
%   of the polynomials to the numerator and the vector with the 
%   coefficients of the polynomial to the denominator:
%
%     N(s) = N_0 + N_1 * s + ... + N_n * s^n.
%     d(s) = d_0 + d_1 * s + ... + d_n * s^n.
%
%   [N, d, W] = LCF(SYS) also returns the corresponding transfer function.
%
% Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
% October 2, 2022

% Store the system matrices locally
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;
% Get the system dimensions
n = size(A, 1);
m = size(B, 2);
p = size(C, 1);
% Initializes the matrices of coefficients of adj(sI-A)
%   ->  adj(sI-A) = E_{n-1} * s^{n-1} + ... + E_1 * s + E_0 
E = cellmat(1, n+1, n, n);
E{n+1} = zeros(n, n);
% Initializes the vector of coefficients of det(sI-A)
%   ->  det(sI-A) = s^n + d_{n-1} * s^{n-1} + ... + d_1 * s + d_0 
d = zeros(1, n+1);
d(n+1) = 1;
% Iteratively calculate the coefficients using
% the Souriau-Leverrier-Faddeev algorithm
for k = 1 : n
  E{n-k+1} = d(n-k+2) * eye(n) + A * E{n-k+2};
  d(n-k+1) = -1/k * trace(A * E{n-k+1});
end
% Check the correctness of the algorithm
err = max(abs(d(1) * eye(n) + A * E{1}), [], 'all');
if err > 1e-3
  warning('Algorithm error: %s', char(err))
end
% Compute the matrices of coefficients of N(s)
%   ->  N(s) = N_0 + N_1 * s + ... + N_n * s^n
N = cellmat(1, n+1, p, m);
for k = 1 : n+1
  N{k} = C * E{k} * B + d(k) * D;
end
% Compose the transfer function numerator
num = cellmat(p, m, 1, n+1);
for i = 1 : p
  for j = 1 : m
    for k = 1 : n+1
      num{i, j}(n-k+2) = N{k}(i, j);
    end
  end
end
% Compose the transfer function denominator
den = fliplr(d);
% Compute the transfer function
W = tf(num, den);

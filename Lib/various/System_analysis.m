%% script for the theoretical test

% stuff
close all
clear 
clc

% down sampling
N = 2;
plot = 1;
filter = 1;

% sym vars
syms y [1 N] real
syms y_dot [1 N] real
syms x0 real
syms theta real
syms T real
syms V real
syms k real

% model
theta_true = 1;
x0_true = 1;
T_true = 0.1;
state = [x0, theta];

% flow
phi = x0.*exp(theta*k*T);
phi_dot = k*x0.*theta*exp(theta*k*T);
phi_fun = symfun(phi,[x0, theta, k, T]);
phi_fun_dot = symfun(phi_dot,[x0, theta, k, T]);

% check pivots around y
y_star = zeros(1,N);
for i=1:N
    y_star(i) = double(phi_fun(x0_true, theta_true, i-1,T_true));
    y_star_dot(i) = double(phi_fun_dot(x0_true, theta_true, i-1,T_true));
end

% define function
V = sym(0);
for i=1:N
   eval(['tmp = (y',num2str(i),' - phi_fun(x0,theta,',num2str(i-1),',T))^2;']);
   V = V + tmp;
   
   if filter
       eval(['tmp_dot = (y_dot',num2str(i),' - phi_fun_dot(x0,theta,',num2str(i-1),',T))^2;']);
       V = V + tmp_dot;
   end
end
if filter
    V_fun = symfun(V,[x0,theta,T,y,y_dot]);
else
    V_fun = symfun(V,[x0,theta,T,y]);
end

% compute gradient
V_grad = gradient_sym(V,state);
V_grad = simplify(V_grad);

% hessian sym
V_hess = gradient_sym(V_grad,state);
V_hess = simplify(V_hess);
if filter
    V_hess_fun = symfun(V_hess,[x0,theta,T,y,y_dot]);
else
    V_hess_fun = symfun(V_hess,[x0,theta,T,y]);
end


% function evaluation
Ts = 0.05;
x0_range = [0.8 1.2];
theta_range = [0.8 1.2];
x0_grid = x0_range(1)*x0_true:Ts:x0_range(2)*x0_true;
theta_grid = theta_range(1)*theta_true:Ts:theta_range(2)*theta_true;
[X,THETA] = meshgrid(x0_grid,theta_grid);
F = {};
F_tot = cell(size(V_hess,1),1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot V
tmp_str = 'V_val = double(V_fun(X,THETA,T_true';
for i=1:N
   tmp_str = [tmp_str,',y_star(',num2str(i),')'];  
end
if filter
   for i=1:N
       tmp_str = [tmp_str,',y_star_dot(',num2str(i),')'];  
    end 
end
tmp_str = [tmp_str, '));'];
eval(tmp_str);
if plot
    % plot
    figure;
    surf(X,THETA,V_val);

    % stuff
    xlabel('x_0');
    ylabel('\theta');
    zlabel('V');
    title('V');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot V_hess eig
tmp_str = 'V_hess_val = V_hess_fun(X,THETA,T_true';
for i=1:N
   tmp_str = [tmp_str,',y_star(',num2str(i),')']; 
end
if filter
   for i=1:N
       tmp_str = [tmp_str,',y_star_dot(',num2str(i),')'];  
    end 
end
tmp_str = [tmp_str, ');'];
eval(tmp_str);
for i=1:length(x0_grid)
   for j=1:length(theta_grid)
       tmp_V = double([ V_hess_val{1,1}(i,j), V_hess_val{1,2}(i,j); ...
                 V_hess_val{2,1}(i,j), V_hess_val{2,2}(i,j)]);
       V_hess_eig(i,j,:) = eig(tmp_V); 
   end
end

if plot
    for i=1:size(V_hess,1)
        figure()
        EIG = reshape(V_hess_eig(:,:,i),[length(x0_grid),length(theta_grid)]);
        surf(X,THETA,EIG);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % check positive definite - pivots
% IsSymmetric = ~any(any(V_hess-V_hess'));
% orl = cell(size(V_hess,1),1);
% for i=1:size(V_hess,1)
%     orl{i} = factor(subs(det(V_hess(1:i,1:i)),'T',1));
% end
% 
% 
% orl_sub = cell(size(V_hess,1),1);
% for i=1:size(V_hess,1)
%     tmp = subs(orl{i}(:),T,1);
%     tmp = subs(tmp,y,y_star);
%     orl_sub{i} = transpose(vpa(tmp,2));
% end
% 
% for i=1:size(V_hess,1)
% 
%     % number fo subplots depending on the Nterm
%     n_subplot = length(orl_sub{i});
% 
%     F_tot{i} = ones(length(x0_grid),length(theta_grid))';
%     for k=1:n_subplot
%              
%     % get function
%     f = symfun(orl_sub{i}(k),[x0, theta]);
%     F{i,k} = double(f(X,THETA));
%     F_tot{i} = F_tot{i}.*F{i,k};
%  
%     end
%     
%     if plot
% 
%         % plot
%         figure(i);
%         surf(X,THETA,F_tot{i});
% 
%         % stuff
%         xlabel('x_0');
%         ylabel('\theta');
%         zlabel('pivot');
%         title(['pivot ',num2str(i)]);
%     end
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% data
Ts = 0.01;
t=0:Ts:1;
y = sin(10*t);
yf = zeros(size(y));
Niter = length(t);

%% define filter
if 1
% derivative
eps1 = 0.0001;
G = tf([1e-4 0],[eps1 1]);
% G = zpk(0,-100,1);
SS = ss(G);
D = c2d(SS,Ts);
filter{1}.TF = D;
filter{1}.G = G;
else
% passabanda
G = zpk([-0.01],[-0.1 -1],1);
G0 = dcgain(G);
G = G/G0;
SS = ss(G);
D = c2d(SS,Ts);
filter{1}.TF = D;
filter{1}.G = G;
end


%% simulation
% reshape buffer
dim_filter = size(filter{1}.TF.B,1);
dim_input = size(filter{1}.TF.B,2);

for pos=1:Niter
    if (pos > dim_filter)     
        % initial condition
        x0 = zeros(dim_filter,1);                            
        for i=1:dim_filter
            x0(i) = yf(pos-dim_filter-1+i);                                
        end

        % input
        u = zeros(dim_input,dim_filter+1);
        for i=1:dim_filter+1
           u(:,i) = y(pos-dim_filter+i-1); 
        end

        yf(pos) = discreteSS(x0,u,filter{1}.TF.A, filter{1}.TF.B, filter{1}.TF.C, filter{1}.TF.D);                                
    else
        yf(pos) = yf(pos);
    end
end

%% plot
figure(1)
grid on
hold on

plot(t,y,'b--');
plot(t,yf,'r.');

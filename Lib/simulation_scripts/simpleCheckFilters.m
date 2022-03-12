clc
obj=obs;
A = obj.setup.filterTF{1}.TF.A;
B = obj.setup.filterTF{1}.TF.B;
C = obj.setup.filterTF{1}.TF.C;
D = obj.setup.filterTF{1}.TF.D;
FILTER.A = obj.setup.filterTF{1}.TF.A;
FILTER.B = obj.setup.filterTF{1}.TF.B;
FILTER.C = obj.setup.filterTF{1}.TF.C;
FILTER.D = obj.setup.filterTF{1}.TF.D;
TF = obj.setup.filterTF{1}.TF;
Ts = 0.1;
Ntot = 1000;
time = 0:Ts:(Ntot-1)*Ts;
omega_n = 0.2;
y = sin(time*omega_n);
X0 = 0;
yf = time*0;
yf(1)= 0;

tic
for k=2:Ntot,
    X0 = A*X0+B*y(k-1);
    yf(k) = C*X0+D*y(k);
end
tempo1 = toc



% figure(1)
% plot(time,yf,'k')
% hold on

X0=0;
tic
for k=2:Ntot,
    tspan = 0:obj.setup.DTs:dim_filter*obj.setup.DTs;
    tmp = lsim(TF,[y(k-1) y(k)],tspan,X0);
    tmp(end);
    yf(k) = tmp(end);
end
tempo3 = toc
% plot(time,yf,'b')


X0=0;
tic
for k=2:Ntot,
%     yf(k) = discreteSS(X0,[y(k-1) y(k)],obj.setup.filterTF{1}.TF.A, obj.setup.filterTF{1}.TF.B, obj.setup.filterTF{1}.TF.C, obj.setup.filterTF{1}.TF.D);
    yf(k) = discreteSS(X0,[y(k-1) y(k)],FILTER.A, FILTER.B, FILTER.C, FILTER.D);
end
tempo4 = toc
% plot(time,yf,'r')

tic
for k=2:Ntot,
    X0 = FILTER.A*X0+FILTER.B*y(k-1);
    yf(k) = FILTER.C*X0+FILTER.D*y(k);
end
tempo2 = toc




tppoints(1,:) = zeros(1,3);
Npoints = 10;
poles = -logspace(-3,3,Npoints);

for i=1:Npoints-1 
    POLES(i,:) = linspace(poles(i),poles(i+1),3);
    PSI(i,:) = poly(POLES(i,:));
    PSI(i,:) = PSI(i,:)./PSI(i,end);    
end


%%
function out = model_analysis(obs,option)
    
    if strcmp(option,'fromobs')
        % true model
        A = [obs.setup.params.A1 obs.setup.params.A2; obs.setup.params.A3 obs.setup.params.A4];
        B = [obs.setup.params.B1; obs.setup.params.B2];
        C=[obs.setup.params.C1 obs.setup.params.C2];
        D = 0;
        out.test = ss(A,B,C,D);

        % estimated model
        A_est = [obs.init.params.A1, obs.init.params.A2; obs.init.params.A3 obs.init.params.A4];
        B_est = [obs.init.params.B1; obs.init.params.B2];
        C_est = [obs.init.params.C1, obs.init.params.C2];
        D_est = D;
        out.test_est = ss(A_est,B_est,C_est,D_est);

        % stabilised model
        out.K = [obs.init.params.K1 obs.init.params.K2];
        out.test_stable = ss(A_est+B_est*out.K,B_est,C_est,D_est);

        % plots
        figure()
        rlocus(out.test)
        hold on
        rlocus(out.test_est)
        rlocus(out.test_stable)
    elseif strcmp(option,'routh')               
        
        % tf symbol
        syms s
        assume(s,'real')
        
        %%% Plant
        out.omega = 2*pi*10;
        out.rho = 0.5;
        out.test = tf([out.omega^2],[1 2*out.omega*out.rho out.omega^2]);
                
        [A,B,C,D] = tf2ss(out.test.num{1},out.test.den{1});
        out.test_ss = ss(A,B,C,D);
        
        out.test_ss_obsv = obsv(out.test_ss);
        out.test_ss_ctrb = ctrb(out.test_ss);
        
        Ap = out.test_ss.A;
        Bp = out.test_ss.B;
        Cp = out.test_ss.C;
        Dp = out.test_ss.D;
        out.P = ss(Ap,Bp,Cp,Dp);
        
        % poly
        out.Gp = simplify(vpa(Cp*pinv((s*eye(size(Ap))-Ap))*Bp+Dp));
        [out.Nump, out.Denp] = numden(out.Gp);
        
        % controller
        syms a0 a1 b0 b1 d0
        assume(a0,'real')
        assume(a1,'real')
        assume(b0,'real')
        assume(b1,'real')
        assume(d0,'real')
        out.a0 = a0;
        out.a1 = a1;
        out.b0 = b0;
        out.b1 = b1;
        out.d0 = d0;
        
        Ac = [0 1; a0 a1];
        Bc = [b0; b1];
        Cc = [1 0];
        Dc = d0;
        out.Ac = Ac;
        out.Bc = Bc;
        out.Cc = Cc;
        out.Dc = Dc;
        out.Gc = simplify(vpa(Cc*pinv((s*eye(size(Ac))-Ac))*Bc+Dc));
        [out.Numc, out.Denc] = numden(out.Gc);
        
        % total model
        out.A = [Ap-Bp*Cp*Dc, Bp*Cc; -Bc*Cp, Ac];
        out.B = [Bp*Dc; Bc];
        out.C = [Cp, [0 0]; -Dc*Cp, Cc];
        out.D = [0; Dc];
        out.poli = simplify(out.Nump*out.Numc + out.Denp*out.Denc);
        out.poli_coeffs = coeffs(out.poli,s);
        
        % create example
        Ac_es = double(subs(out.Ac,[a0 a1],[10 20]));
        Bc_es = double(subs(out.Bc,[b0 b1],[30 60]));
        Cc_es = Cc;
        Dc_es = double(subs(out.Dc,d0,50));
        Gc_es_ss = ss(Ac_es,Bc_es,Cc_es,Dc_es);
        Gc_es_tf = subs(out.Gc,[a0 a1 b0 b1 d0],[10 20 30 60 50]);
        [Gc_es_num,Gc_es_den] = numden(Gc_es_tf);
        Gc_es_num = double(coeffs(Gc_es_num,s));
        Gc_es_den = double(coeffs(Gc_es_den,s));
        Gc_es_tf = tf(Gc_es_num,Gc_es_den);
                        
        % stability
        out.RE = simplify(myRouth(out.poli_coeffs));
        
    elseif strcmp(option,'rlocus')
        
        %%% Plant
        Ap = [obs.init.params.A1 obs.init.params.A2; obs.init.params.A3 obs.init.params.A4];
        Bp = [obs.init.params.B1; obs.init.params.B2]; 
        Cp = [obs.init.params.C1 obs.init.params.C2];
        Dp = 0;       
        out.test_ss = ss(Ap,Bp,Cp,Dp);
        
        out.test_ss_obsv = obsv(out.test_ss);
        out.test_ss_ctrb = ctrb(out.test_ss);        
        
        % estimated plant
        Ap = [0 1; obs.init.params.a0est obs.init.params.a1est];
        Bp = [obs.init.params.b0est; obs.init.params.b1est]; 
        Cp = [obs.init.params.c0est obs.init.params.c1est];
        Dp = obs.init.params.d0est;
        out.test_ss_est = ss(Ap,Bp,Cp,Dp);
        
        %%% controller
        Ac = [0 1; obs.init.params.a0 obs.init.params.a1];
        Bc = [obs.init.params.b0; obs.init.params.b1]; 
        Cc = [obs.init.params.c0 obs.init.params.c1];
        Dc = obs.init.params.d0;
        out.ctrl_ss = ss(Ac,Bc,Cc,Dc);
                
        %%% FB system
        Af = [(Ap - Bp*Cp*Dc), (Bp*Cc); (-Bc*Cp), (Ac)];
        Bf = [(Bp*Dc); Bc];
        Cf = [(Cp), zeros(1,2); (-Dc*Cp), (Cc)];
        Df = [0; (Dc)];
        out.fb_ss = ss(Af,Bf,Cf,Df);        
        
        %%% plots
        figure(1)
        rlocus(out.test_ss);
        hold on
        rlocus(out.test_ss_est)
        figure(2)
        bode(out.test_ss);
        hold on
        bode(out.test_ss_est)
        figure(3)
        rlocus(out.ctrl_ss)       
        figure(4)
        bode(out.test_ss,out.ctrl_ss)
        figure(5)
        bode(out.fb_ss);

        
    else
        
        error('Wrong option');
    end

end
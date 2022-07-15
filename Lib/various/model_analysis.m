%%
function out = model_analysis(obs,fromobs)
    
    if fromobs
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
    else
        
        % tf symbol
        syms s
        assume(s,'real')
        
        % plant
        Ap = [1 -1; 1 1];
        Bp = [1; 2];
        Cp = [2 1];
        Dp = 0;
        out.Ap = Ap;
        out.Bp = Bp;
        out.Cp = Cp;
        out.Dp = Dp;
        out.P = ss(Ap,Bp,Cp,Dp);
        % poly
        out.Gp = simplify(vpa(Cp*pinv((s*eye(size(Ap))-Ap))*Bp+Dp));
        [out.Np, out.Dp] = numden(out.Gp);
        
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
        [out.Nc, out.Dc] = numden(out.Gc);
        
        % total model
        out.A = [Ap-Bp*Cp*Dc, Bp*Cc; -Bc*Cp, Ac];
        out.B = [Bp*Dc; Bc];
        out.C = [Cp, [0 0]; -Dc*Cp, Cc];
        out.D = [0; Dc];
        out.poli = simplify(out.Np*out.Nc + out.Dp*out.Nc);
        out.poli_coeffs = coeffs(out.poli,s);
        
        % stability
        out.RE = simplify(myRouth(out.poli_coeffs));
                
    end

end
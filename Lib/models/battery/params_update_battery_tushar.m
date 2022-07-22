%% PARAMS_UPDATE_BATTERY
% file: params_update_battery.m
% author: Federico Oliva
% date: 27/05/2022
% description: this function updates the estimated parameters on a Van der 
% Pol oscillator
% INPUT:
% params: structure with all the necessary parameters 
% x: state vector
% OUTPUT:
% params_out: updated structure with the new model parameters
function params_out = params_update_battery_tushar(params,x,t)
    % assign params
    params_out = params;    
    
%     ocv = [3.5042   3.5573    3.6009    3.6393    3.6742    3.7121    3.7937    3.8753    3.9700    4.0764    4.1924];
%     soc = [0   10  20  30  40  50  60  70  80  90  100];
%     R0 = [0.0114    0.0110    0.0113    0.0112    0.0110    0.0107    0.0107    0.0107    0.0109    0.0113    0.0116];
%     R1 = [0.0103    0.0067    0.0051    0.0043    0.0039    0.0034    0.0033    0.0033    0.0033    0.0033    0.0028];
%     C1 = [0.2288    0.6122    1.8460    2.0975    1.5254    1.0440    1.3903    1.6694    1.5784    1.2165    0.9118];

%     I = params.input(t,x,params);
%     Vt = params.measure(x,params,t);
%     Voc_temp = Vt + x(2) + (I* params.R0);
%     soc_temp = spline(ocv,soc,params_out.Voc);
    if t>45
        disp(x)
    end
    Voc_temp = spline(params.input_soc, params.input_OCV, x(1));
    R0_temp = spline(params.input_soc, params.input_R0, x(1));
    R1_temp = spline(params.input_soc, params.input_R1, x(1));
    C1_temp = spline(params.input_soc, params.input_C1, x(1));

    params_out.Voc = Voc_temp;
    params_out.R0 = R0_temp;
    params_out.R1 = R1_temp;
    params_out.C1 = C1_temp;
end
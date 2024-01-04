%% PseudoDer
% file: PseudoDer.m
% author: Daniele Carnevale
% date: 10/02/2021
% description: this function computes the time derivative of a signal over 
%              a buffer by approximating the incremental difference formula
% INPUT: 
%           Ts: sampling time 
%           y: new data sample
%           wlen: length of the two subwindows where the median/mean is 
%           evaluated
%           bufsize: samples buffer length 
%           median: flag to select median computation over buffer or sample average
%           reset: flag to discard the data buffer
% OUTPUT: 
%           dy: derivative value
%           der_buffer: updated signal buffer
%           counter: buffer filling status counter
%  
%           how the algorithm works:
%           (------bufsize------) 
%           (-wlen-)-----(-wlen-) 
%           [--V1--]-----[--V2--]
%           dy = (V2 - V1)/(Ts * wlen)

function [dy, der_buffer, counter]  = PseudoDer(Ts,y,wlen,bufsize,signalsize,median,reset,obs,der_buffer, counter)   
  
     % Reset buffer if required 
     if(isempty(der_buffer) || (reset)) 
         for traj = 1:obs.init.params.Ntraj
            der_buffer(traj).val = zeros(signalsize,bufsize); 
         end
        
        counter = 0; 
     end 
  
     % Update buffer by shifting old samples and store new one 
     for k = 1:(bufsize-1) 
        der_buffer(obs.init.traj).val(:,k) = der_buffer(obs.init.traj).val(:,k+1); 
     end 

     % update 
     counter = counter+1; 
     der_buffer(obs.init.traj).val(:,bufsize) = y; 
  
     % Compute derivative over buffer (if enough data has been collected) 
     if (counter >= bufsize) 
        temp1 = 0; 
        temp2 = 0; 
      
        if (median) 
            % Current and future values are computed using medians 
            temp1 = median(der_buffer(obs.init.traj).val(:,1:wlen),2); 
            temp2 = median(der_buffer(obs.init.traj).val(:,(bufsize-wlen+1):bufsize),2); 
        else 

            % Current and future values are averaged over the subwindows 
            for k=1:wlen 
                temp1 = temp1+der_buffer(obs.init.traj).val(:,k); 
            end 
            temp1 = temp1/wlen; 
      
            for k=(bufsize-wlen+1):bufsize 
                temp2 = temp2 + der_buffer(obs.init.traj).val(:,k); 
            end 
            temp2 = temp2/wlen; 
        end 
      
        % compute derivative
        dy = (temp2-temp1)/(Ts*(bufsize-wlen)); 
     else 

        % no derivative if not enough signal data
        dy = zeros(signalsize,1); 
        
     end 
 end
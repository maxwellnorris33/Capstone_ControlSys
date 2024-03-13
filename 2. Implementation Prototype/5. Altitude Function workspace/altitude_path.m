function [altRef, lastAltTime] = altitude_path(crnt_alt,trgt_alt, climb_rt, h0, clock, lastAltTime)
    %---------- Variables ------------------
    %crnt_alt = current altitude in m
    % trgt_alt = targeted new altitude in m
    % vertical climb rate in m/s
    % h0 = starting altitude
    % clock = current time
    % lastAlt = last altitude durring last iteration of this function in m
    % lastAltTime = last time of last altitude during last iteration

    %------------- Code -------------------
    %if currAlt is above or below desired alt, initiate climb/descent
    %if currAlt is within 5% of desired alt, initiate alt lock

    tolerance = 0.1*abs(trgt_alt-h0);
    
   
    if crnt_alt < (trgt_alt - tolerance)
        rampSignal = climb_rt+crnt_alt;
        altRef = rampSignal;
        lastAltTime = clock;
        disp('up')
        disp(altRef)
    elseif crnt_alt > (trgt_alt + tolerance)
        rampSignal = -climb_rt+crnt_alt;
        altRef = rampSignal;
        lastAltTime = clock;
        disp(rampSignal)
    else
        altRef = trgt_alt;
        lastAltTime = clock;
        disp('else')
        disp(altRef)
    end
end



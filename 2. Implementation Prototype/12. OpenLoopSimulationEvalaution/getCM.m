function CM = getCM(a_interest,r_interest)

    
    zerodef0 = [0.0, -0.0020, 0.0];
    aileffect10 = [0.0745, -0.0026, 0.0046];
    ruddeffect10 = [0.0074, -0.0033, 0.0161];
    cmx_a = [];
    cmx_r = [];
    cmx_tot = [];
    cmy_a = [];
    cmy_r = [];
    cmy_tot = [];
    cmz_a = [];
    cmz_r = [];
    cmz_tot = [];
    for i=1:length(a_interest)
    
    cmx_a(i) = interp1([0 10],[zerodef0(1), aileffect10(1)],a_interest(i));
    cmx_r(i) = interp1([0 10],[zerodef0(1), ruddeffect10(1)],r_interest(i));
    cmx_tot(i)  = sum([cmx_a(i), cmx_r(i)]);
    
    cmy_a(i) = interp1([0 10],[zerodef0(2), aileffect10(2)],a_interest(i));
    cmy_r(i) = interp1([0 10],[zerodef0(2), ruddeffect10(2)],r_interest(i));
    cmy_tot(i)  = sum([cmy_a(i), cmy_r(i)]);
    
    cmz_a(i) = interp1([0 10],[zerodef0(3), aileffect10(3)],a_interest(i));
    cmz_r(i) = interp1([0 10],[zerodef0(3), ruddeffect10(3)],r_interest(i));
    cmz_tot(i)  = sum([cmz_a(i), cmz_r(i)]);
    
    end
    CM = [abs(cmx_tot); cmy_tot; cmz_tot];

end



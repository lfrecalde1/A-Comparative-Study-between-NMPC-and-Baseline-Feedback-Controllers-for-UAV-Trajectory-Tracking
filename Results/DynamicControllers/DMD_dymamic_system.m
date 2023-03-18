function [v_real_sig] = DMD_dynamic_system(A,B,v_real, v_ref,ts)

vp = A*v_real+B*v_ref;
v_real_sig = v_real + vp*ts;

end
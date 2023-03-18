function [Ae,Be,P,G]=DMD_Online(m,v_estimate1,vc,v_real,P,G,k)

U = [v_estimate1(:,k+2-m:k+1);
     vc(:,k+1-m:k)];
V = v_real(:,k+2-m:k+1);
C = eye(m);
C(1,1) = -1;
factor = inv(inv(C)+(U'*P*U));
P = (P - P*U*factor*U'*P);
G = G +(V-G*U)*factor*U'*P;
Ae = G(:,1:4);
Be = G(:,5:end);


end

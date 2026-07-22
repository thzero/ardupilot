function q = rocket_quat_mul(a, b)
% ROCKET_QUAT_MUL  Hamilton product a*b. Equivalent to quatmultiply(a,b).
q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end

classdef SE_2_3
    methods (Static)
        function Tout = compose(T1, T2)
            [R1, p1, v1] = LieGroups.SE_2_3.extractSE23(T1);
            [R2, p2, v2] = LieGroups.SE_2_3.extractSE23(T2);
            Tout = LieGroups.SE_2_3.constructSE23(R1*R2, R1*p2 + p1, R1*v2 + v1);
        end
        
        function [Tout, J_minv_m] = inverse(T)
            [R, p, v] = LieGroups.SE_2_3.extractSE23(T);
            
            Rinv = R';
            pinv = -R'*p;
            vinv = -R'*v;
            Tout = LieGroups.SE_2_3.constructSE23(Rinv, pinv, vinv);
            J_minv_m = -LieGroups.SE_2_3.AdjointMatrix(T);
        end
        
        function Tout = identity()
            Tout = eye(5);
        end
        
        function v = vee(g)
            [omega_cross, vlin, alin] = LieGroups.SE_2_3.extractse23(g);
            omega = LieGroups.SO3.unskew(omega_cross);
            v = [vlin; omega; alin];
        end
        
        function g = hat(v)
            vlin = v(1:3);
            omega = v(4:6);
            alin = v(7:9);
            omega_cross = LieGroups.SO3.skew(omega);
            g = LieGroups.SE_2_3.constructse23(omega_cross, vlin, alin);
        end
        
        function T = exp(g)
            v = LieGroups.SE_2_3.vee(g);
            T = LieGroups.SE_2_3.exphat(v);
        end
        
        function [vOut, J_vout_m, J_vout_v] = act(T, vIn)
            [R, p, ~] = LieGroups.SE_2_3.extractSE23(T);                        
            vOut = p + R*vIn;
                        
            J_vout_m = [R -R*LieGroups.SO3.hat(vIn) zeros(3, 3)];
            J_vout_v = R;
        end
        
        function [T, J_mout_m, J_mout_t] = exphat(vIn)
            vlin = vIn(1:3);
            omega = vIn(4:6);
            alin = vIn(7:9);
            
            R = LieGroups.SO3.exphat(omega);
            JlSO3 = LieGroups.SO3.leftJacobian(omega);
            
            p = JlSO3*vlin;
            v = JlSO3*alin;
            T = LieGroups.SE_2_3.constructSE23(R, p, v);
            
            Tinv = LieGroups.SE_2_3.inverse(T);
            J_mout_m = LieGroups.SE_2_3.AdjointMatrix(Tinv);
            J_mout_t = LieGroups.SE_2_3.rightJacobian(vIn);
        end
        
        function g = log(T)
            v = LieGroups.SE_2_3.logvee(T);
            g = LieGroups.SE_2_3.hat(v);
        end
        
        function v = logvee(T)
            [R, p, v] = LieGroups.SE_2_3.extractSE23(T);
            omega = LieGroups.SO3.logvee(R);
            JlinvSO3 = LieGroups.SO3.leftJacobianInverse(omega);
            rho = JlinvSO3*p;
            alpha = JlinvSO3*v;
            
            v = [rho; omega; alpha];
        end
        
        function AdT = AdjointMatrix(T)
            [R, p, v] = LieGroups.SE_2_3.extractSE23(T);
            p_cross = LieGroups.SO3.skew(p);
            v_cross = LieGroups.SO3.skew(v);
            AdT = [       R    p_cross*R    zeros(3);
                    zeros(3)           R    zeros(3);
                    zeros(3)   v_cross*R          R];
        end
        
        function adT = crossProductMatrix(v)
            vlin = v(1:3);
            omega = v(4:6);
            alin = v(7:9);
            
            vlin_cross = LieGroups.SO3.skew(vlin);
            omega_cross = LieGroups.SO3.skew(omega);
            alin_cross = LieGroups.SO3.skew(alin);
            
            adT = [omega_cross      vlin_cross     zeros(3);
                zeros(3)    omega_cross     zeros(3);
                zeros(3)     alin_cross  omega_cross];
        end
        
        function Jr = rightJacobian(v)
            omega = v(4:6);
            JrSO3 = LieGroups.SO3.rightJacobian(omega);
            
            Qvr = LieGroups.SE3.leftJacobianLinearSubmatrix(-v(1:6));
            Qar = LieGroups.SE3.leftJacobianLinearSubmatrix(-[v(7:9); omega]);
            
            Jr = [JrSO3      Qvr   zeros(3);
                zeros(3)   JrSO3   zeros(3);
                zeros(3)     Qar     JrSO3];
        end
                
        function Jl = leftJacobian(v)
            omega = v(4:6);
            JlSO3 = LieGroups.SO3.leftJacobian(omega);
            
            Qv = LieGroups.SE3.leftJacobianLinearSubmatrix(v(1:6));
            Qa = LieGroups.SE3.leftJacobianLinearSubmatrix([v(7:9); omega]);
            
            Jl = [JlSO3       Qv   zeros(3);
                zeros(3)   JlSO3   zeros(3);
                zeros(3)      Qa     JlSO3];
        end
        
        function Jlinv = leftJacobianInverse(v)
            Jl = LieGroups.SE_2_3.leftJacobian(v);            
            Jlinv = inv(Jl);
        end
        
        function [R, p, v]  = extractSE23(T)
            R = T(1:3, 1:3);
            p = T(1:3, 4);
            v = T(1:3, 5);
        end
        
        function [omega_cross, vlin, alin] = extractse23(g)
            omega_cross = g(1:3, 1:3);
            vlin = g(1:3, 4);
            alin = g(1:3, 5);
        end
        
        function T = constructSE23(R, p, v)
            T = eye(5);
            T(1:3, 1:3) = R;
            T(1:3, 4)  = p;
            T(1:3, 5)  = v;
        end
        
        function g = constructse23(omega_cross, vlin, alin)
            g = zeros(5);
            g(1:3, 1:3) = omega_cross;
            g(1:3, 4) = vlin;
            g(1:3, 5) = alin;
        end
    end
end


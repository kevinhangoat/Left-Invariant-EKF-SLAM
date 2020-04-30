classdef InEKF < handle   
    properties
        g;                  % g term
        mu;                 % Pose Mean
        Sigma;              % Pose Sigma
        gfun;               % Motion model function
        mu_pred;             % Mean after prediction step
        Sigma_pred;          % Sigma after prediction step
        mu_cart;
        sigma_cart;
    end
    
    methods
        function obj = InEKF(sys, init)
            obj.g = [0; 0; 9.81];
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, wk, ak, dt)
            
            R = obj.mu(1:3, 1:3); % Rotation matrix 3x3
            v = obj.mu(1:3, 4);   % velocity vector 3x1
            p = obj.mu(1:3, 5);   % position vector 3x1
            
            
            H_prev = [...
                      R               v      p;
                      zeros(1,3)      1      0;
                      zeros(1,3)      0      1];
            
            R_pred = R * expm(obj.skew(wk * dt));
            v_pred = v + (R * ak' + obj.g) *dt;
            p_pred = p + v*dt + 1/2 * (R * ak' + obj.g) * dt^2;
            
            H_pred = [...
                      R_pred            v_pred      p_pred;
                      zeros(1,3)             1           0;
                      zeros(1,3)             0           1];
            
%             u_se5 = logm(H_prev \ H_pred);

            Adjoint = @(R, v, p) [R, [zeros(3, 6)]; ...
                                  obj.skew(v)*R, R, zeros(3, 3);
                                  obj.skew(p)*R, zeros(3,3), R];
            AdjX = Adjoint(R, v, p);
            
            %Log-Linear Left-Invariant Dynamics
            A_l = [-obj.skew(wk), zeros(3, 3), zeros(3,3);...
                   -obj.skew(ak), -obj.skew(wk), zeros(3,3);...
                   zeros(3,3),      eye(3),    -obj.skew(wk)];
            
%             obj.propagation(u_se5, AdjX, A_l, dt);
            obj.propagation(H_pred, A_l, dt);
        end
        
        function propagation(obj, H_pred, A_l, dt)
            % SE(5) propagation model; the input is u \in se(5) plus noise
            % propagate mean
%             obj.mu_pred = obj.mu * expm(u);
            obj.mu_pred = H_pred;
            
%             obj.mu_pred = obj.mu;
            
            % propagate covariance
            phi = expm(A_l * dt);
            obj.Sigma_pred = obj.Sigma + ...
                             phi * diag([1 1 1 1 1 1 1 1 1]) * phi';
        end
        
        function correction(obj, Y)
            b = [0; 0; 0; 0; 1];
                       
            H = [zeros(3), zeros(3), eye(3)];
            
            
            N = 10*eye(3);             
%             N = obj.mu_pred \ N * obj.mu_pred' \ eye(length(obj.mu_pred));
            Y = [Y'; 0; 1];
            nu = obj.mu_pred \ Y - b; 
            
            S = H * obj.Sigma_pred * H' + N;
            K = obj.Sigma_pred * H' * (S \ eye(size(S)));
            
            delta = K * nu(1:3);
            
            obj.mu = obj.mu_pred * expm(obj.skew_hat(delta));
            
            obj.Sigma = (eye(9) - K * H) * obj.Sigma_pred * (eye(9) - K * H)' + K * N * K';
        end
        
        
        function w_bar = skew(obj, w)
            w_bar = [0 -w(3) w(2) ; w(3) 0 -w(1) ; -w(2) w(1) 0]; 
        end 
        
        % skew from 9x1 to 5x5
        function w_bar_hat = skew_hat(obj, w)
            w_bar_hat = [obj.skew(w(1:3)), w(4:6), w(7:9);... 
                         zeros(2,5)]; 
        end 
    end
end

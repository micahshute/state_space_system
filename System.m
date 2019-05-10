classdef System
    %SYSTEM Initialize a System using either a tf or ss matrices. Allow
    %complex operations on the system with built-in methods
    
    properties
        a,b,c,d, x0
    end
    
    methods
        function obj = System(a,b,c,d)
            % You can input ss matrices. If you only input 2 arguments
            % System assumes you input a num, den pair for a TF. 
            % It initializes itself appropriately based off of those
            % assumptions.
            if nargin == 2
                [a,b,c,d] = tf2ss(a, b);
                obj.a = a;
                obj.b = b;
                obj.c = c;
                obj.d = d;
            end
            if nargin == 4
                obj.a = a;
                obj.b = b;
                obj.c = c;
                obj.d = d;
            end
            
        end
        
        function obj = set_num(obj, num)
            % Allows you to set a new TF numerator for the system
            % This returns a new System object.
           [old_num, den] = obj.tf;
           obj = System(num, den);
        end
        
        function obj = set_den(obj, den)
            % Allows you to set a new TF denominiator for the system
            % This returns a new System object.
           [num, old_den] = obj.tf;
           obj = System(num, den);
        end
        
        
        function obj = set_sys(obj,sys)
            % Return a new object from an input of 
            % a matlab return from either ss() or tf()
            [t_a,t_b,t_c,t_d] = ssdata(sys);
            obj = obj.update_ss(t_a,t_b,t_c,t_d);
        end
        
        function [ ad,bd,cd,dd ] = get_dcf(obj)
            % Return Diagonal Canonical Form matrices
            [ad,bd,cd,dd] = obj.get_ssxform(obj.tdcf);
        end
        
        function obj = to_dcf(obj)
            % Return a new System object that is the same as the current
            % object but is in Diagonal Canonical Form
           t = obj.tdcf;
           obj = obj.xform(t);
        end
        
        function outsys = sys(obj)
            % Return the output of matlab's ss() function
            % with the objects SS matrices as input
            outsys = ss(obj.a,obj.b,obj.c,obj.d);
        end
        
        function [num, den] = tf(obj)
            %return the TF of the system as 2 vectors reprisenting 
            % the num and den
            % Currently only works for SISO
           [num, den] = ss2tf(obj.a, obj.b, obj.c, obj.d); 
        end
        
        function numerator = num(obj)
            % return just the TF numerator of the system
           [numerator, den] = obj.tf;
        end
        
        function denominator = den(obj)
            % return just the TF denominator of the system
           [num, denominator] = obj.tf;
        end
        
        function [ at bt ct dt ] = get_ssxform( obj, t )
            % Given a SS transformation matrix t, return
            % the SS matrices of the transformed system
            ti = inv(t);
            at = ti * obj.a * t;
            bt = ti * obj.b;
            ct = obj.c * t;
            dt = obj.d;
        end
       
        function obj = xform(obj, t)
            % Return a new System transformed by the state transformation
            % matrix t
            [t_a,t_b,t_c,t_d] = obj.get_ssxform(t);
            obj.a = t_a;
            obj.b = t_b;
            obj.c = t_c;
            obj.d = t_d;
        end
       
        function vals = eigenvalues(obj)
            % Return the eigenvalues of the system
           vals = eig(obj.a); 
        end
        
        function [ t ] = tdcf(obj)
            % return the Diagonal Canonical Form Transformation matrix of
            % the system
            [vec, vals] = eig(obj.a);
            t = vec;
        end

        function p = controllability_matrix(obj)
            % Return the controllability matrix of the system
            p = ctrb(obj.a, obj.b);
        end
        
        
        function status = is_controllable(obj)
            % Return a 1/0 flag based on if the system is controllable or
            % not. Print a statement declaring the controlalbility
            sizea = size(obj.a,1);
            if(rank(obj.controllability_matrix) < sizea)
                display('System Not Controllable');
                status = false;
            else
                display('System Is Controllable');
                status = true;
            end
        end
        
        function display_ss(obj)
           % Display the SS matrices
           A = obj.a
           B = obj.b
           C = obj.c
           D = obj.d
        end
        
        function [ a,b,c,d ] = get_ccf( obj)
            % Return the SS matrices of the system in Controller Canonical
            % Form
            p = ctrb(obj.a,obj.b);
            i_pccf = obj.ipccf;
            t = p * i_pccf;
            [a,b,c,d] = obj.get_ssxform(t);

        end
        
        function t_ccf = tccf(obj)
            p = ctrb(obj.a,obj.b);
            i_pccf = obj.ipccf;
            t_ccf = p * i_pccf;
        end
        
        function  p  = ipccf(obj)
            % Return the inverse of the controller canonical form
            % controllability matrix
            den = obj.den;
            s = size(den,2) - 1;
            p = zeros(s,s);
            for i = 1:s
                for j = 1:s-i
                    p(i,j) = den(s + 2 - i - j);
                end
                    p(i, s-i + 1) = 1;
            end
        end

        
        function obj = to_ccf( obj)
            % Return a new System object which is equivalent to the current
            % system but in Controller Canonical Form
            t = obj.tccf;
            obj = obj.xform(t);
        end
        
        
        function obj = set_x0(obj, x0)
            % Set initial conditions for the System
            obj.x0 = x0;
        end
        
        function [yout, t, xout] = simulate(obj, u, t)
            % Simulate an output of the system given input and time
            % vecotrs. Return output, time and states
           [yout, t, xout] = lsim(obj.sys, u, t, obj.x0); 
        end
        
        function  w = cont_gram(obj, tf, t0)
        %CONT_GRAM Calculate the Controllability Gramian.
        %   Inputs:final time, and initial time(defaults to
        %   0)

            
            if(nargin <= 1)
               syms tf
            end

            if(nargin <= 2)
                t0 = 0;
            end


           syms t
           v = expm(obj.a * (t0 - t));
           w = int(v * obj.b * obj.b' * v', t, t0, tf);

        end
        
        
        function [ u ] = find_input(obj, out_i, out_f, tf, t0)
        %FIND_INPUT Calculate an input argument for a system given A,B, initial and
        %final desired outputs, and stop time. Start time is optional and defaults
        %to 0.
            if size(obj.a,1) ~= size(out_i,1) || size(out_i,1) ~= size(out_f,1)...
                    || size(out_i,2) ~= size(out_f,2) || size(out_i,2) ~= 1
                error('Output sizes must match the dimensions of a and must be one column')
            end
            if nargin < 3
                error('Not enough input args. Requres SS Matrices A and b, initial input, final input, final time, and initial time(defaults to 0)');
            end
            if nargin == 3
                t0 = 0;
            end
            syms t
            v = expm(obj.a * (t0 - t));
            w = obj.cont_gram(tf,t0);
            u = obj.b' * v' * inv(w) * (subs(v, t, tf)*out_f - out_i);

        end
        
        function k = get_control_gain(obj, des_eigs)
            % Get the CL gain for the system given desired eigenvalues
            k = place(obj.a, obj.b, des_eigs);
        end
        
        function obj = control(obj, des_eigs)
            % Return a system that DOES NOT have Steady State tracking but
            % has added Closed Loop gain 
            k = obj.get_control_gain(des_eigs);
            obj.a = obj.a - obj.b * k;
        end
        
        function [k, an, bn] = servocontroller_gain(obj, des_eigs)
            % Return the gain and A and B matrices of a servocontroller
            % design of the current system. The A and B values are the ones
            % used to calculate k using `place`
            aug_zeros = zeros(1,size(obj.a,1))';
            an = [obj.a, aug_zeros; -obj.c, 0];
            bn = [obj.b;0];
            k = place(an, bn, des_eigs);
        end
        
        function obj = servocontroller(obj, des_eigs)
            % Return a servocontrolled CL System calculated from the
            % current system
            [k, an, bn] = obj.servocontroller_gain(des_eigs);
            aa = an - bn * k;
            ba = zeros(1,size(obj.a,1) + 1)';
            ba(end) = 1;
            ca = [obj.c 0];
            obj.a = aa;
            obj.b = ba;
            obj.c = ca;
        end
        
        
        function iq = iqocf(obj)
           % Get the inverse of Qocf which is equal to iPccf 
           iq = obj.ipccf;
        end
        
        function q_ocf = qocf(obj)
            % Return Qocf
           q_ocf = obj.pccf;
        end
        
        function p_ccf = pccf(obj)
            % return Pccf
            p_ccf = inv(obj.ipccf);
        end
        
        function p_ret = p(obj)
            % return Controllability Matrix P
           p_ret = ctrb(obj.a, obj.b);
        end
        
        function q_ret = q(obj)
            %return Observability matrix Q
           q_ret = obsv(obj.a, obj.c);
        end
        
        function t_ocf = tocf(obj)
            % Return Tocf (transform from current SS sys to Oberver
            % Canonical Form)
            t_ocf = obj.q \ obj.qocf;
        end
        
        function [ao, bo, co, do] = get_ocf(obj)
            t = obj.tocf;
            [ao,bo,co,do] = obj.get_ssxform(t);
        end
        
        function obj = to_ocf(obj)
           t = obj.tocf;
           obj = obj.xform(t);
        end
        
        function status  = is_observable(obj)
           s = size(obj.a,1);
           if rank(obj.q) < s
               status = false;
               display('The system is not observable.');
           else
              status = true;
              display('The system is observable.');
           end
        end
        
        
        function w = obs_gram(obj, tf, t0)
            
            if(nargin <= 1)
               syms tf
            end

            if(nargin <= 2)
                t0 = 0;
            end


           syms t
           v = expm(obj.a * (t - t0));
           w = int(v' * obj.c' * obj.c * v, t, t0, tf);
        end
        
    end
    
end


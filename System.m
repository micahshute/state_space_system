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
        
        function [z,p,k] = zpk(obj)
            % Return zeros, poles, gain data for system
           [z,p,k] = zpkdata(obj.sys, 'v'); 
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
        
        
        function [ damping_ratio, damping_angle ] = damp_from_po(obj, po )
        %DAMP_FROM_PO Pass in max percent overshoot, get back minimum damping coeff
        % If two returns are requested, you also get the damping_angle (maximum
        % angle)
            damping_ratio = abs(log(po / 100)) / sqrt(pi^2 + log(po/100)^2);
            damping_angle = acos(damping_ratio);
            %damping_angle = vpa(damping_angle, 5);
        end

       
        function vals = eigenvalues(obj)
            % Return the eigenvalues of the system
           vals = eig(obj.a); 
        end
        
        function p = poles(obj)
           p = obj.eigenvalues; 
        end
        
        function z = zeros(obj)
           num = obj.num;
           z = roots(num);
        end
        
        function [ t ] = tdcf(obj)
            % return the Diagonal Canonical Form Transformation matrix of
            % the system
            [vec, vals] = eig(obj.a);
            t = vec;
        end

        
        
        function status = is_controllable(obj)
            % Return a 1/0 flag based on if the system is controllable or
            % not. Print a statement declaring the controlalbility
            sizea = size(obj.a,1);
            p = round(obj.p, 7);
            p_rank = min(rank(p), rank(obj.p));
            if(p_rank < sizea)
                display('System is NOT controllable');
                status = false;
            else
                display('System is controllable');
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
        
        function stepinfo(obj)
           stepinfo(obj.sys)
        end
        
        function [k, an, bn] = servocontroller_gain(obj, des_eigs)
            % Return the gain and A and B matrices of a servocontroller
            % design of the current system. The A and B values are the ones
            % used to calculate k using `place`
            aug_zeros = zeros(1,size(obj.a,1))';
            an = [obj.a, aug_zeros; -obj.c, 0];
            bn = [obj.b;0];
            k = obj.try_place(an, bn, des_eigs);
        end
        
        function gain = try_place(obj,a,b,eigs)
            try 
                gain = place(a, b, eigs);
            catch 
                try
                    % Try ackermann's instead. If this doesn't work (ie system > 1
                    % output), then slightly change pole location to eliminate
                    % duplicates via algorithm below.
                    gain = acker(a, b, eigs);
                catch

                    % For duplicate poles, place will not work. So, change them
                    % slightly to allow the algorithm to place the poles. 
                    dev = 0.00000000001;
                    sign = -1;
                    for i = 1:size(eigs)
                       eigs(i) = eigs(i) + sign * ceil(i / 2) * dev;
                       sign = sign * -1;
                    end
                    gain = place(a, b, eigs);
                end
            end
            
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
        
        function p = controllability_matrix(obj)
            % Return the controllability matrix of the system
            try
                p = ctrb(obj.a, obj.b);
            catch
                s = size(obj.a,1);
                p = [obj.b];
                for i = 2:s
                    p = [p ((obj.a^(i-1)) * obj.b)];
                end
            end
        end
        
        
        function p_ret = p(obj)
            % return Controllability Matrix P
           p_ret = obj.controllability_matrix;
        end
        
        function q = observability_matrix(obj)
            % return Observability matrix
             try 
                 
                 q = obsv(obj.a, obj.c);
                 
             catch
                 
                 s = size(obj.a,1);
                 q = [obj.c];
                 for i = 2:s
                    q = [q ;(obj.c * (obj.a^(i-1)))];
                 end
                 
             end
            
        end
        
        function q_ret = q(obj)
            %return Observability matrix Q
           q_ret = obj.observability_matrix;
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
           q = round(obj.q,7);
           s = size(obj.a,1);
           q_rank = min( rank(q), rank(obj.q));
           if q_rank < s
               status = false;
               display('System is NOT observable.');
           else
              status = true;
              display('System is observable.');
           end
        end
        
        
        function w = obs_gram(obj, tf, t0)
            % Return the Observability Gramian
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
        
        
        function [a,b,c,d] = get_dual(obj)
            % Get Dual representation of system (dx = a'x + c'u ; y = b'x +
            % d');
            a = obj.a';
            b= obj.c';
            c = obj.b';
            d = obj.d';
        end
        
        function obj = to_dual(obj)
           % Return a new System in the form of the dual of the original
           % system. Note that the TF will not change
           [a,b,c,d] = obj.get_dual;
           obj.a = a;
           obj.b = b;
           obj.c = c;
           obj.d = d;
        end
        
        function t = get_xform_to(obj, to_sys)
            t = obj.q \ to_sys.q;
        end
        
        function p = char_eqn(obj)
            % Return characteristic equation for A
           p = poly(obj.a);
        end
        
        function obj = reduce2(obj)
            % Returns a new System object reduced from its old form.
            % IE Cancels zeros and poles
            % Works only for SISO 
            [reducable, duplicates] = obj.is_reducable;
            duplicates_map = containers.Map('KeyType','char','ValueType','logical');
            for i = 1:size(duplicates,2)
                duplicates_map(num2str(duplicates(i))) = true;
            end
            if ~reducable
                return
            else
                poles = obj.poles;
                zeros = obj.zeros;
                new_poles = [];
                new_zeros = [];
                for i = 1:size(poles);
                    if ~duplicates_map.isKey(num2str(poles(i)))
                        new_poles = [new_poles poles(i)];
                    end
                end
                for i = 1:size(zeros);
                    if ~duplicates_map.isKey(num2str(zeros(i)))
                        new_zeros = [new_zeros zeros(i)];
                    end
                end
                num = poly(new_zeros);
                den = poly(new_poles);
                obj = System(num, den);
            end
        end
        
        function obj = reduce(obj)
            % Reduce using MATLAB's built in minreal
           msys = minreal(obj.sys);
           [a,b,c,d] = ssdata(msys);
           obj.a = a;
           obj.b = b;
           obj.c = c;
           obj.d = d;
        end
        
        
        function [flag, duplicates] = is_reducable(obj)
            % Check to see if there are cancellable zeros and poles
            flag = false;
            duplicates = [];
            poles = obj.poles;
            zeros = obj.zeros;
            pole_map = containers.Map('KeyType','char','ValueType','logical');
            for i = 1:size(poles);
               poles(i) = round(poles(i), 7);
               pole_map(num2str(poles(i))) = true;
            end
            for i = 1:size(zeros);
                zeros(i) = round(zeros(i), 7);
                if pole_map.isKey(num2str(zeros(i)))
                    flag = true;
                    duplicates = [duplicates zeros(i)];
                end
            end
        end
        
        
        function flag = strings_equal(obj, s1, s2)
            if(size(s1,2) ~= size(s2,2))
               flag = false;
               return;
            end
            
            flag = (sum(s1 == s2) == size(s1,2));
        end
        
        function tfs = ss2tf(obj)
            % Get TF directly from ss matrices
           syms s
           sa = size(obj.a,1);
           tfs = obj.c * inv(s*eye(sa) - obj.a) * obj.b + obj.d;
        end
        
        function tfs = to_tf(obj)
            % Get TF object back from SS 
           syms s
           sa = size(obj.a,1);
           tfeq = obj.c * inv(s*eye(sa) - obj.a) * obj.b + obj.d;
           str = char(tfeq);
           s = tf('s');
           eval(['tfs = ',str])
        end
        
        function l = full_observer_gain(obj, des_eigs)
            % Get the gain L for a full order Observer
            % l = place(obj.a', obj.c', des_eigs)';
            try 
                l = place(obj.a', obj.c', des_eigs)';
            catch 
                try
                    % Try ackermann's instead. If this doesn't work (ie system > 1
                    % output), then slightly change pole location to eliminate
                    % duplicates via algorithm below.
                    l = acker(obj.a', obj.c', des_eigs)';
                catch

                    % For duplicate poles, place will not work. So, change them
                    % slightly to allow the algorithm to place the poles. 
                    dev = 0.00000000001;
                    sign = -1;
                    for i = 1:size(des_eigs)
                       des_eigs(i) = des_eigs(i) + sign * ceil(i / 2) * dev;
                       sign = sign * -1;
                    end
                    l = place(obj.a', obj.c', des_eigs)';
                end
            end
        end
        
        
        function [a,b,l] = get_full_observer(obj, des_eigs)
            % To calculate the full-dimentional Linear State Observer:
            % Input desired eigenvalues, get back 3 matrices:
            % Outputs [A-LC, B, L] to form: 
            % xhatdot(t) = (A-LC) * xhat(t) + Bu(t) + L(y(t))
            % Where: xhat(t) = state estimates; u(t) = OL sys input;
            % y(t) = OL sys output
           l = obj.full_observer_gain(des_eigs);
           a = obj.a - l * obj.c;
           b = obj.b;
        end
        
        function [y, yhat, xsys, xhat] = simulate_full_observer(obj, des_eigs, x0)
            % Simulate system states with a sin input. 
            % Can change input and initial conditions manually in simulink
            % model
           
            if (nargin < 3) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 3 
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            l = obj.full_observer_gain(des_eigs);
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('full_order_observer_simulation', [], opts)
            figure;
            plot(dy.time, dy.signals.values)
            title('y(t) - y_{hat}(t)');
            xlabel('time {\it(sec)}');
            figure;
            plot(y.time, y.signals.values);
            hold on;
            plot(yhat.time, yhat.signals.values, 'r--');
            title('y(t) vs y_{hat}(t)');
            legend('y(t)', 'y_{hat}(t)');
            xlabel('time {\it(sec)}');
            number_states = xsys.signals.dimensions;
            for i = 1:number_states
                
               figure;
               plot(xsys.time, xsys.signals.values(:,i));
               hold on
               plot(xhat.time, xhat.signals.values(i,:), 'r--');
               title(strcat('x_',num2str(i),'(t) vs x_{hat_',num2str(i),'}(t)'));
               legend(strcat('x_',num2str(i),'(t)'), strcat('x_{hat_',num2str(i),'}(t)'));
               xlabel('time {\it(sec)}');
            end
            
        end
        
        function [y, yhat, xsys, xhat] = get_simulation_full_observer(obj, des_eigs, x0)
             if (nargin < 3) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 3 
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            l = obj.full_observer_gain(des_eigs);
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('full_order_observer_simulation', [], opts)
            
        end
        
        function [l,j,n,m,t] = get_untransformed_reduced_observer(obj, des_eigs)
            % Gives the information to get a the reduced order observer
            % estimate of xhat(t):
            % wdot(t) = Jw(t)+Ny(t)+Mu(t)
            % zhat2(t) = Ly(t) + w(t) ; zhat1(t) = y(t)
            % xhat(t) = Tzhat(t)
            [t,l,e,pt,qt,j,m,n] = obj.get_all_reduced_observer_info(des_egs)
        end
        
        function [t,l,p,q,e_m_lc, aq, ap] = get_reduced_observer(obj, des_eigs)
            % Gives the information to get xhat(t) from the following set of equations:
            % w(t) = (E-LC)[AQzhat2(t) + APy(t)+Bu(t)]
            % zhat2(t) = Ly(t) + w(t)
            % xhat(t) = Py(t) + Qzhat2(t)
            % WHERE:
            % [C;E] = inv(T) ; T = [P Q] (P and Q are NOT
            % controllability/invertability matrix). 
            % e_m_lc = (E-LC); aq = AQ; ap = AP; p = P; q = Q; l = L;
            % NOTE: This is the system used in the simulink model above
            [t,l,e,p,q,j,m,n] = obj.get_all_reduced_observer_info(des_eigs);
            ap = obj.a*p;
            e_m_lc = e - l*obj.c;
            aq = obj.a*q;
        end
        
        function [y, yhat, xsys, xhat] = get_simulation_reduced_observer(obj, des_eigs, x0)
            % Simulate system with a sin input, return simulation outputs
            % y, yhat, xsys, xhat
            % Remember that a reduced order observer requires one less
            % eigenvalue than a full-order obserer
            if (nargin < 3) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 3 
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            [t,l,pt,qt,e_m_lc, aq, ap] = obj.get_reduced_observer(des_eigs);
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('reduced_order_observer_simulation', [], opts)
            xhat.signals.values = xhat.signals.values(:,:)';
        end
        
        function [y, yhat, xsys, xhat] = simulate_reduced_observer(obj, des_eigs, x0)
            
            % Simulate system states with a sin input. 
            % Can change input and initial conditions manually in simulink
            % model
            % Remember that a reduced observer requires one less eigenvalue
            % than a full-order observer
            [t,l,pt,qt,e_m_lc, aq, ap] = obj.get_reduced_observer(des_eigs);
            if (nargin < 3) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 3 
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
     
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('reduced_order_observer_simulation', [], opts)
            figure;
            plot(y.time, y.signals.values);
            hold on;
            plot(yhat.time, yhat.signals.values, 'r--');
            title('y(t) vs y_{hat}(t)');
            legend('y(t)', 'y_{hat}(t)');
            xlabel('time {\it(sec)}');
            number_states = xsys.signals.dimensions;
            xhat.signals.values = xhat.signals.values(:,:)';
            for i = 1:number_states
               figure;
               plot(xsys.time, xsys.signals.values(:,i));
               hold on
               plot(xhat.time, xhat.signals.values(:,i), 'r--');
               title(strcat('x_',num2str(i),'(t) vs x_{hat_',num2str(i),'}(t)'));
               legend(strcat('x_',num2str(i),'(t)'), strcat('x_{hat_',num2str(i),'}(t)'));
               xlabel('time {\it(sec)}');
               
            end
        end
        
        function [t,l,e,pt,qt,j,m,n] = get_all_reduced_observer_info(obj, des_eigs)
            % Calculate all necessary matrices to create a reduced order
            % observer:
            % Gives enough information to use the following system:
            % w(t) = (E-LC)[AQzhat2(t) + APy(t)+Bu(t)]
            % zhat2(t) = Ly(t) + w(t)
            % xhat(t) = Py(t) + Qzhat2(t)
            % WHERE:
            % [C;E] = inv(T) ; T = [P Q] (P and Q are NOT
            % controllability/invertability matrix). 
            % NOTE: This is the system used in the simulink model above
            % OR: 
            % wdot(t) = Jw(t)+Ny(t)+Mu(t)
            % zhat2(t) = Ly(t) + w(t) ; zhat1(t) = y(t)
            % xhat(t) = Tzhat(t)
            % Remember that a reduced-order observer requires one less
            % eigenvaue than a full-order observer
            tinv = obj.c;
            psize = size(obj.c,1);
            nsize = size(obj.c,2);
            for i = (psize + 1):nsize
               vec = obj.get_linearlly_independent_vector(tinv);
               tinv = [tinv; vec];
            end
            
            t = inv(tinv);
            e = tinv((nsize-psize):end, :);
            pt = t(:, 1:psize);
            qt = t(:, psize + 1:end);
            a11 = obj.c * obj.a * pt;
            a12 = obj.c * obj.a * qt;
            a21 = e * obj.a * pt;
            a22 = e * obj.a * qt;
            %b1 = obj.b(1:psize,1);
            %b2 = obj.b((nsize-psize):end, 1);
            try 
                l = place(a22', a12', des_eigs)';
            catch 
                try
                    % Try ackermann's instead. If this doesn't work (ie system > 1
                    % output), then slightly change pole location to eliminate
                    % duplicates via algorithm below.
                    l = acker(a22', a12', des_eigs)';
                catch

                    % For duplicate poles, place will not work. So, change them
                    % slightly to allow the algorithm to place the poles. 
                    dev = 0.00000000001;
                    sign = -1;
                    for i = 1:size(des_eigs)
                       des_eigs(i) = des_eigs(i) + sign * ceil(i / 2) * dev;
                       sign = sign * -1;
                    end
                    l = place(a22', a12', des_eigs)';
                end
            end
            j = a22 - l*a12;
            % Test pole location
            % eig(j) --> checks
            n = a21 + j*l - l*a11;
            %m = b2 - l*b1;
            m = (e - l*obj.c)*obj.b;
        end
        
        function vec = get_linearlly_independent_vector(obj, mat)
            % Create a random linearlly indpenedent vector to add to a give
            % matrix to facilitate making the state transformation matrix 
            % T = [C;E] for the reduced observer system. 
            rows = size(mat, 1);
            cols = size(mat, 2);
            attempt = 1;
            while 1
               vec = obj.rand_vec(attempt, mat); 
               reduced = rref([mat; vec]);
               success = true;
               for row = 1:(rows + 1)
                  if sum(reduced(row, :)) == 0
                     attempt = attempt + 1;
                     success = false;
                     break;
                  end
               end
               if success
                   break;
               end
            end
        end
        
        function vec = rand_vec(obj, attempt, mat)
            rows = size(mat, 1);
            cols = size(mat, 2);
            if attempt <= cols
                % Attempt a single direction first (ie eye matrix if
                % possible)
                vec = zeros(1, cols);
                vec(attempt) = 1;
            else
                % Create a random vector
                vec = zeros(1, cols);
                for i = 1:cols
                    vec(i) = 100 * rand;
                end
            end
        end
        
        function [y,t] = step(obj)
            [y,t] = step(obj.sys);
        end
        
        function is_stabilizable(obj)
            
        end
        
        function is_detectable(obj)
            
        end
        
        function obj = to_full_observer_controller(obj, observer_eigs, control_eigs)
           % The return 
            l = obj.full_observer_gain(observer_eigs);
            k = obj.get_control_gain(control_eigs);
            na = [obj.a, -obj.b * k;...
                l* obj.c, obj.a - obj.b*k - l*obj.c];
            nb = [obj.b; obj.b];
            
            nc = [obj.c, zeros(size(obj.c))];
            nd = obj.d;
            obj = System(na,nb,nc,nd);
        end
        
        function [y, yhat, xsys, xhat, yol, xol] = get_simulation_full_observer_controller(obj, obs_eigs, cont_eigs, x0)
            
            if (nargin < 4) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 4
                    x0 = zeros(size(obj.c));
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            
            k = obj.get_control_gain(cont_eigs);
            l = obj.full_observer_gain(obs_eigs);
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('full_order_observer_controller_simulation', [], opts)
        end
        
        function simulate_full_observer_controller(obj, obs_eigs, cont_eigs, x0)
           
            if (nargin < 4) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 4
                    x0 = zeros(size(obj.c));
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            
            [y, yhat, xsys, xhat, yol, xol] = obj.get_simulation_full_observer_controller(obs_eigs, cont_eigs, x0);
            figure;
            dy = y.signals.values - yhat.signals.values;
            plot(y.time, dy)
            title('y(t) - y_{hat}(t)');
            xlabel('time {\it(sec)}');
            figure;
            plot(y.time, y.signals.values);
            hold on;
            plot(yhat.time, yhat.signals.values, 'r--');
            plot(yol.time, yol.signals.values, 'g--');
            title('y(t) vs y_{hat}(t)');
            legend('y(t)', 'y_{hat}(t)', 'y_{ol}(t)');
            xlabel('time {\it(sec)}');
            number_states = xsys.signals.dimensions;
            xsys.signals.values = xsys.signals.values(:,:)';
            xhat.signals.values = xhat.signals.values(:,:)';
            %xol.signals.values = xol.signals.values(:,:)';
            for i = 1:number_states
                
               figure;
               plot(xsys.time, xsys.signals.values(:,i));
               hold on
               plot(xhat.time, xhat.signals.values(:,i), 'r--');
               plot(xol.time, xol.signals.values(:,i), 'g--');
               title(strcat('x_',num2str(i),'(t) vs x_{hat_',num2str(i),'}(t)'));
               legend(strcat('x_',num2str(i),'(t)'), strcat('x_{hat_',num2str(i),'}(t)'),strcat('x_{ol_',num2str(i),'}(t)'));
               xlabel('time {\it(sec)}');
            end
        end
        
        
        function obj = to_reduced_observer_controller(obj, obs_eigs, cont_eigs)
           % Return new System for a reduced-order observer-baseed
           % compensator WITHOUT SS Tracking. The states are orered x(t),
           % w(t), where zhat2(t) = Ly(t) + w(t), and xhat(t) = Py(t) +
           % Qzhat2(t). P and Q are not the Controllability and
           % Observability matrices in this context, they are partitioned
           % portions of a transformation matrix used to transition xhat to
           % zhat (and back). Specific calculations are seen in other
           % Reduced Observer methods. So, you can get the states of the
           % resulting object from this method by using .simulate, and then
           % using L and y(t), and the w(t) states to get zhat2(t), and
           % then using P and Q (gotten from another method above) to find
           % xhat(t). Or, you can use the
           % get_simulated_reduced_observer_controller method below which
           % will give you xhat values. 
           [t,l,e,pt,qt,j,m,n] = obj.get_all_reduced_observer_info(obs_eigs);
           k = obj.get_control_gain(cont_eigs);
           aa = [obj.a - obj.b *k*(pt + qt*l)*obj.c,...
               -obj.b*k*qt;...
               -m*k*(pt+qt*l)*obj.c + n*obj.c,...
               j-m*k*qt];
           bb = [obj.b;m];
           cc = [obj.c, zeros(1, size(aa,2) - size(obj.c,2))];
           dd = 0;
           obj.a = aa;
           obj.b = bb;
           obj.c = cc;
           obj.d = dd;
           
        end
        
        function [y, yhat, xsys, xhat, yol, xol] = get_simulation_reduced_observer_controller(obj, obs_eigs, cont_eigs, x0)
            % Get results from a simulink simulation of a reduced order
            % observer-based controller. If you want to change initial
            % conditions of the observer or the input, alter the simulink
            % file referenced in this method.
            if (nargin < 4) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 4
                    x0 = zeros(size(obj.c));
                   %error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            
            [t,l,pt,qt,e_m_lc, aq, ap] = obj.get_reduced_observer(obs_eigs);
            k = obj.get_control_gain(cont_eigs);
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('reduced_order_observer_controller_simulation', [], opts)
            
            
        end
        
        
        function simulate_reduced_observer_controller(obj, obs_eigs, cont_eigs, x0)
           % Get data from the above method and plot it
           % Requries desired observer eigenvalues (n-1, because reduced
           % observer) and n controller eigenvalues.
            if (nargin < 4) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 4
                    x0 = zeros(size(obj.c));
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            
            [y, yhat, xsys, xhat, yol, xol] = obj.get_simulation_reduced_observer_controller(obs_eigs, cont_eigs, x0);
            figure;
            dy = y.signals.values - yhat.signals.values;
            plot(y.time, dy)
            title('y(t) - y_{hat}(t)');
            xlabel('time {\it(sec)}');
            figure;
            plot(y.time, y.signals.values);
            hold on;
            plot(yhat.time, yhat.signals.values, 'r--');
            plot(yol.time, yol.signals.values, 'g--');
            title('y(t) vs y_{hat}(t)');
            legend('y(t)', 'y_{hat}(t)', 'y_{ol}(t)');
            xlabel('time {\it(sec)}');
            number_states = xsys.signals.dimensions;
            xsys.signals.values = xsys.signals.values(:,:)';
            xhat.signals.values = xhat.signals.values(:,:)';
            %xol.signals.values = xol.signals.values(:,:)';
            for i = 1:number_states
                
               figure;
               plot(xsys.time, xsys.signals.values(:,i));
               hold on
               plot(xhat.time, xhat.signals.values(:,i), 'r--');
               plot(xol.time, xol.signals.values(:,i), 'g--');
               title(strcat('x_',num2str(i),'(t) vs x_{hat_',num2str(i),'}(t)'));
               legend(strcat('x_',num2str(i),'(t)'), strcat('x_{hat_',num2str(i),'}(t)'),strcat('x_{ol_',num2str(i),'}(t)'));
               xlabel('time {\it(sec)}');
            end
        end
        
        function obj = full_compensator(obj, ceigs, oeigs)
           % Input requires n+1 desired controller eigenvalues (+1 due to
           % servocontroller design), and n observer desired eigenvalues
           % (should be 10x controller eigs)
           % Transform system in to a CL full-order-observer-based
           % controller with servocontroller SS Tracking
           % Require n+1 controller eigs (an extra for servocontroller) and
           % n observer eigenvalues
           % State are ordered: x(t)(n states), zeta(t) (1 state), xhat(t)
           % (n states)
           k = obj.servocontroller_gain(ceigs);
           l = obj.full_observer_gain(oeigs);
           ki = -k(end);
           k = k(1:end-1);
           bki = obj.b*ki;
           mbk = -obj.b*k;
           obj.a = [obj.a, bki, mbk;...
                    -obj.c, zeros(size(obj.c,1), size(bki,2)), zeros(size(obj.c,1),size(mbk,2));...
                    l*obj.c, bki, obj.a + mbk - l*obj.c];
           obj.b = [zeros(size(obj.b)); 1; zeros(size(obj.b))];
           obj.c = [obj.c, 0, zeros(size(obj.c))];
           obj.d = 0;
            
        end
        
        
        function simulate_full_compensator(obj, ceigs, oeigs, x0)
            % This does NOT have a companion get_full_compensator
            % simulation method, because getting a new System via the
            % full_compensator method above and using .simulate on that new
            % System object will return all the information you need. State
            % order in that return will be x(t) (n states), zeta (one
            % state), and then xhat(t) (n states). This method exists
            % mostly for easy plotting, a simulink drawing to show the
            % actual implementation of the system, and for easy simulink
            % GUI to change around inputs or initial conditions settings.
            if (nargin < 4) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 4
                    x0 = zeros(size(obj.c));
                   error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
            
            k = obj.servocontroller_gain(ceigs);
            l = obj.full_observer_gain(oeigs);
            ki = -k(end);
            k = k(1:end-1);
            a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('full_compensator_with_ss_simulation', [], opts)
            figure;
            dy = y.signals.values - yhat.signals.values;
            plot(y.time, dy)
            title('y(t) - y_{hat}(t)');
            xlabel('time {\it(sec)}');
            figure;
            plot(y.time, y.signals.values);
            hold on;
            plot(yhat.time, yhat.signals.values, 'r--');
            plot(yol.time, yol.signals.values, 'g--');
            title('y(t) vs y_{hat}(t)');
            legend('y(t)', 'y_{hat}(t)', 'y_{ol}(t)');
            xlabel('time {\it(sec)}');
            number_states = xsys.signals.dimensions;
            xsys.signals.values = xsys.signals.values(:,:)';
            xhat.signals.values = xhat.signals.values(:,:)';
            %xol.signals.values = xol.signals.values(:,:)';
            for i = 1:number_states
                
               figure;
               plot(xsys.time, xsys.signals.values(:,i));
               hold on
               plot(xhat.time, xhat.signals.values(:,i), 'r--');
               plot(xol.time, xol.signals.values(:,i), 'g--');
               title(strcat('x_',num2str(i),'(t) vs x_{hat_',num2str(i),'}(t)'));
               legend(strcat('x_',num2str(i),'(t)'), strcat('x_{hat_',num2str(i),'}(t)'),strcat('x_{ol_',num2str(i),'}(t)'));
               xlabel('time {\it(sec)}');
            end
        end
        
        function obj = reduced_compensator(obj, ceigs, oeigs)
           % Input requires n+1 controller eigenvalues and n-1 observer
           % eiegenvalues (+1 ceigs bc of servocontroller design, -1 oeigs
           % because the observer is reduced)
           % This will return a new System Object to the user, which they
           % can then use to simulate an output. Also, the method defined
           % below wil simulate via a simulink simulation.
           % The states will be: x(t) (n states),zeta, w(t) (n-1 states). See description of w(t) above
           % in the to_reduced_observer_controller method description.
           [t,l,e,pt,qt,j,m,n] = obj.get_all_reduced_observer_info(oeigs);
           k = obj.servocontroller_gain(ceigs);
           % Separate controller gain from servocontroller gain
           ki = -k(end);
           k = k(1:end-1);
           aa = [obj.a - obj.b *k*(pt + qt*l)*obj.c, obj.b * ki, -obj.b*k*qt;...
               -obj.c, zeros(size(obj.c,1),size(obj.b,2)), zeros(size(obj.c,1), size(obj.b*k,2) - 1);...
               -m*k*(pt+qt*l)*obj.c + n*obj.c, m*ki, j-m*k*qt];
           bb = [zeros(size(obj.b)); 1; zeros(size(obj.b,1) - 1, size(obj.b,2))];
           cc = [obj.c, zeros(1, size(aa,2) - size(obj.c,2))];
           dd = 0;
           obj.a = aa;
           obj.b = bb;
           obj.c = cc;
           obj.d = dd;
        end
        
        
        function simulate_reduced_compensator(obj, ceigs, oeigs, x0)
           % Input controller eigenvalues (n+1 because there is servocontroller ss tracking), 
           % and observer eigenvalues (n-1 because the observer is reduced
           % order. There is no sister method for this which returns the
           % data for the simulation. You can use the method above to do a
           % manual simulation, which will have states x(t) (n states),
           % w(t) (n-1 states). The xhat(t) values can be harvested via the
           % relationships zhat2(t) = Ly(t) + w(t) ; 
           % xhat(t) = Py(t) + Qzhat2(t), where P and Q can be obtained from
           % the method .get_reduced_observer (NOTE P and Q are NOT the
           % controllability and observability matrices in this instance)
           
           if (nargin < 4) && (size(obj.x0,1) > 0)
                x0 = obj.x0;
            else
                if nargin < 4
                    x0 = zeros(size(obj.c));
                   %error('You must either use set_x0 to set initial conditions or add them as an argument')
                end
            end
           
           
           [t,l,pt,qt,e_m_lc, aq, ap] = obj.get_reduced_observer(oeigs);
           k = obj.servocontroller_gain(ceigs);
           % Separate controller gain from servocontroller gain
           ki = -k(end);
           k = k(1:end-1);
           a = obj.a;
            b = obj.b;
            css = eye(size(a));
            dss = [zeros(size(a,1), 1)];
            c = obj.c;
            opts = simset('SrcWorkspace', 'current');
            sim('reduced_compensator_simulation', [], opts)
            figure;
            dy = y.signals.values - yhat.signals.values;
            plot(y.time, dy)
            title('y(t) - y_{hat}(t)');
            xlabel('time {\it(sec)}');
            figure;
            plot(y.time, y.signals.values);
            hold on;
            plot(yhat.time, yhat.signals.values, 'r--');
            plot(yol.time, yol.signals.values, 'g--');
            title('y(t) vs y_{hat}(t)');
            legend('y(t)', 'y_{hat}(t)', 'y_{ol}(t)');
            xlabel('time {\it(sec)}');
            number_states = xsys.signals.dimensions;
            xsys.signals.values = xsys.signals.values(:,:)';
            xhat.signals.values = xhat.signals.values(:,:)';
            %xol.signals.values = xol.signals.values(:,:)';
            for i = 1:number_states
                
               figure;
               plot(xsys.time, xsys.signals.values(:,i));
               hold on
               plot(xhat.time, xhat.signals.values(:,i), 'r--');
               plot(xol.time, xol.signals.values(:,i), 'g--');
               title(strcat('x_',num2str(i),'(t) vs x_{hat_',num2str(i),'}(t)'));
               legend(strcat('x_',num2str(i),'(t)'), strcat('x_{hat_',num2str(i),'}(t)'),strcat('x_{ol_',num2str(i),'}(t)'));
               xlabel('time {\it(sec)}');
            end
            
        end
            
            
        %TODO: 
        % --> Refactor observer/controller gain functions to 
        % use a single fucntion which has the try/catch logic
        % --> Also refactor plotting methods to have a common method call
        % --> Standardize order of controller eigenvalues and observer
        % eigenvalues in methd marguments
    end
    
end


function [M,C,Tg,B] = manipEqns(n)
% Gives out the symbolic representations for the maniplator Eqn matrices

% Parameters can be manually set here
    param.m = ones(n,1);    % masses for each link
    param.L = ones(n,1);    % lengths for each link
    param.l = param.L/2;    % half length for each link
    param.J = ones(n,1);    % moment of Inertia for each link
    param.c = ones(n,1);    % viscous damping for each link

    m = param.m;
    L = param.L;
    l = param.l;
    J = param.J;
    c = param.c;

    syms g  

    % Symbolic states and inputs
    q = sym(zeros(n,1));
    dq = sym(zeros(n,1));
    d2q = sym(zeros(n,1));
    u = sym(zeros(n,1));


    for k=1:n
        q(k) = sym(sprintf('q%d', k));
        dq(k) = fulldiff(q(k));
        d2q(k) = fulldiff(dq(k),{q(k),dq(k)});
        u(k) = sym(sprintf('u%d', k));
    end

    T = 0;
    V = 0;
    D = 0;
    
    for i = 1:n
        j_i = .5*J(i)*dq(i)^2;
        
        if i == 1
            ke_xy = .5*m(i)*( (l(i)*cos(q(i))*dq(i))^2 + (-l(i)*sin(q(i))*dq(i))^2 );
            pe = m(i)*g*l(i)*cos(q(i));
            D = .5*(c(i)*dq(i)^2);
        else
            ke_xy = .5*m(i)* ( ( l(i)*cos(q(i))*dq(i) + sum(L(1:i-1).*cos(q(1:i-1)).*dq(1:i-1)))^2 + ...
                (-l(i)*sin(q(i))*dq(i)- sum(L(1:i-1).*sin(q(1:i-1)).*dq(1:i-1)))^2);
            pe = m(i)*g*( l(i)*cos(q(i))+sum(L(1:i-1).*cos(q(1:i-1))) );
            D = D + .5*(c(i)*(dq(i)-dq(i-1))^2);
        end
        
        T = T + j_i + ke_xy;
        V = V + pe;

    end

    Lgrn = simplify(T-V);
    vars = {};
    
    for j=1:n
        vars =[vars, {q(j), dq(j)}];
    end
    
    for i = 1:n
        eqn(i) = simplify(fulldiff(simplify(diff(Lgrn,dq(i))),vars) - diff(Lgrn,q(i)) + simplify(diff(D,dq(i))));
    end
    
    M = sym(zeros(n,n));
    C = sym(zeros(n,n));
    Tg = sym(zeros(n,1));
    B = eye(n);
    
    for j = 1:n
        for k=1:n
            
            [cd2_1,td2_1] = coeffs(eqn(j), d2q(k));
            [cd_1,td_1] = coeffs(eqn(j), dq(k));
            
            if length(td2_1) ~= 1
                M(j,k) = sum(cd2_1(1:end-1).*(td2_1(1:end-1)./d2q(k)));
            else
                M(j,k)=0;
            end
            
            if length(td_1) ~= 1
                C(j,k) = sum(cd_1(1:end-1).*(td_1(1:end-1)./dq(k)));
            else
                C(j,k) = 0;
            end

        end
        
        [cg,tg] = coeffs(eqn(j),g);
        Tg(j,1) = cg(1:end-1).*(tg(1:end-1)./g);
    end
    
%     for i = 1:size(M)
    % M
    % C
    % Tg
    % B
end


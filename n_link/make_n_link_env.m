function env = make_n_link_env()
    n = 5;
    [M,C,Tg,B] = manipEqns(n);
    [M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
    env = n_link_arm_Env(M_eq,C_eq,Tg_eq,B);
end
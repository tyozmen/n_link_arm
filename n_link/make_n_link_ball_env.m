function env = make_n_link_ball_env()
  n = 3;
  [M,C,Tg,B] = manipEqns(n);
  [M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
  env = n_link_ball_Env(M_eq,C_eq,Tg_eq,B, ones(n,1), false);



  % n = 5;
  %  [M,C,Tg,B] = manipEqns(n);
  %  [M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
  %  env = n_link_arm_Env(M_eq,C_eq,Tg_eq,B);
end

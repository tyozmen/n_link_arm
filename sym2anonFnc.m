function [M_test,C_test,Tg_test] = sym2anonFnc(M,C,Tg)
% takes in symbolic Maniplator matrices and converts them into anonymous functions with variables (q1-qn,dq1-dqn)
n = length(M);
q_Tg = sort(symvar(Tg));
% q_M = sort(symvar(M))
q_C = sort(symvar(C));
vrs_q = [];
vrs_dq = [];
dummystr = [];
dummystr_C = [];
Tg_str = ['['];
M_str = ['['];
C_str = ['['];
for i = 1:length(q_Tg)
    if i == n
        vrs_q = [vrs_q sprintf('%s', q_Tg(i))];
        vrs_dq = [vrs_dq sprintf('%s', q_C(i))];
%         dummystr = [dummystr sprintf('dummy(%d)',i)];
        Tg_str = [Tg_str sprintf('%s]',Tg(i))];
        
    else
        vrs_q = [vrs_q sprintf('%s, ', q_Tg(i))];
        vrs_dq = [vrs_dq sprintf('%s, ', q_C(i))];
%         dummystr = [dummystr sprintf('dummy(%d),',i)];
        Tg_str = [Tg_str sprintf('%s;',Tg(i))];
    end    
    
    for j = 1:n
        
        if i == n && j == n
            M_str = [M_str sprintf('%s]',M(i,j))];
            C_str = [C_str sprintf('%s]',C(i,j))];
%             dummystr_C = [dummystr_C sprintf('dummy_full(%d)',(i-1)*n+j)];
        elseif j == n
            M_str = [M_str sprintf('%s;',M(i,j))];
            C_str = [C_str sprintf('%s;',C(i,j))];
%             dummystr_C = [dummystr_C sprintf('dummy_full(%d), ',(i-1)*n+j)];
        else
            M_str = [M_str sprintf('%s,',M(i,j))];
            C_str = [C_str sprintf('%s,',C(i,j))];
%             dummystr_C = [dummystr_C sprintf('dummy_full(%d), ',(i-1)*n+j)];
        end
    end
end
% dummystr_C
% Tg_str
% M_str
% C_str
% for i = 1:length(q_Tg)
%     if i == 1
%         vrs = [vrs sprintf(' q(%d) ', i)];
%     elseif i == n
%         vrs = [vrs sprintf('q(%d)', i)];
%     else
%         vrs = [vrs sprintf('q(%d) ', i)];
%     end
% end
% vrs
% for k=1:n
%     eval(sprintf('q%d = q(k);', k));
% %     eval(sprintf('dq%d = dq(k);', k));
% end
% subs(Tg)
% vrs_q;
eval(sprintf(['Tg_test = @(' vrs_q ') ' Tg_str ';']));
eval(sprintf(['M_test = @(' vrs_q ') ' M_str ';']));
eval(sprintf(['C_test = @(' vrs_q ',' vrs_dq ') ' C_str ';']));
end


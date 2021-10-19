function [M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg)
% takes in symbolic Maniplator matrices and converts them into anonymous functions with variables (q1-qn,dq1-dqn)
n = length(M);

Tg_str = ['['];
M_str = ['['];
C_str = ['['];

for i = 1:length(Tg)
    if i == n
        Tg_str = [Tg_str sprintf('%s]',Tg(i))];
    else
        Tg_str = [Tg_str sprintf('%s;',Tg(i))];
    end    
    
    for j = 1:n
        
        if i == n && j == n
            M_str = [M_str sprintf('%s]',M(i,j))];
            C_str = [C_str sprintf('%s]',C(i,j))];
        elseif j == n
            M_str = [M_str sprintf('%s;',M(i,j))];
            C_str = [C_str sprintf('%s;',C(i,j))];
        else
            M_str = [M_str sprintf('%s,',M(i,j))];
            C_str = [C_str sprintf('%s,',C(i,j))];
        end
    end   

end
% dummystr_C    M_str = strrep(M_str,old_q_str, new_q_str);

% Tg_str
% M_str
% C_str
% for i = 1:length(q_Tg)1:length(q_Tg)
%     if i == 1
%         vrs = [vrs sprintf(' q(%d) ', i)];
%     elseif i == n
%         vrs = [vrs sprintf('q(%d)', i)];
%     else
%         vrs = [vrs sprintf('q(%d) ', i)];
%     end1:length(q_Tg)
% end
% vrs
% for k=1:n
%     eval(sprintf('q%d = q(k);', k));
% %     eval(sprintf('dq%d = dq(k);', k));
% end
% subs(Tg)
% vrs_q;
for i = 1:n
    old_q_str = sprintf('q%i',i);
    new_q_str = sprintf('q(%i)',i);
    old_dq_str = sprintf('dq(%i)',i);
    new_dq_str = sprintf('q(%i)',i+n);
    
    M_str = strrep(M_str,old_q_str, new_q_str);
    Tg_str = strrep(Tg_str,old_q_str, new_q_str);
    C_str =  strrep(C_str,old_q_str, new_q_str);
    C_str =  strrep(C_str,old_dq_str, new_dq_str);
end

sprintf(['Tg_eq = @(q) ' Tg_str ';'])
sprintf(['M_eq =  @(q) ' M_str  ';'])
sprintf(['C_eq =  @(q) ' C_str  ';'])

eval(sprintf(['Tg_eq = @(q) ' Tg_str ';']));
eval(sprintf(['M_eq =  @(q) ' M_str  ';']));
eval(sprintf(['C_eq =  @(q) ' C_str  ';']));
end


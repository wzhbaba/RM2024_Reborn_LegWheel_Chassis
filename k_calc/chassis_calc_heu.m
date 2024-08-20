%LQR求解
clear;

syms theta dot_theta ddot_theta;
syms x dot_x ddot_x;
syms x_body dot_x_body ddot_x_body;
syms phi dot_phi ddot_phi;
syms T T_p;
syms R L L_M l m_w m_p M I_w I_p I_M g;
body_fusion = 1;    %机体速度
g = 9.8;            %重力加速度
R = 0.076;          %轮半径
m_w = 0.47; %轮质量
m_p = 1.874;        %摆杆质量
M = 6.975;           %机体质量
I_w = 0.001974; %轮转动惯量
I_M = 0.143;                 %机体转动惯量
l = 0.0353;                  %机体质心离转轴距离

Q_cost=diag([160 160 200 120 2000 50]);
R_cost=diag([1, 0.25]);

if body_fusion
    ddot_x = ddot_x_body - (L+L_M)*cos(theta)*ddot_theta + (L+L_M)*sin(theta)*dot_theta^2;
end
%对机体受力分析
N_M = M * (ddot_x + (L + L_M) * (-dot_theta^2*sin(theta) + ddot_theta*cos(theta)) - l*(-dot_phi^2*sin(phi) + ddot_phi*cos(phi)));
P_M = M*g + M*((L+L_M)*(-dot_theta^2*cos(theta) - ddot_theta*sin(theta)) + l*(-dot_phi^2*cos(phi) - ddot_phi*sin(phi)));
N = N_M + m_p*(ddot_x + L*(-dot_theta^2*sin(theta) + ddot_theta*cos(theta)));
P = P_M + m_p*g + m_p*L*(-dot_theta^2*cos(theta) - ddot_theta*sin(theta));

%方程求解
equ1 = ddot_x - (T - N*R)/(I_w/R + m_w*R);
equ2 = (P*L + P_M*L_M)*sin(theta) - (N*L+N_M*L_M)*cos(theta) - T + T_p - I_p*ddot_theta;
equ3 = T_p + N_M * l * cos(phi) + P_M * l * sin(phi) - I_M * ddot_phi;
if body_fusion
    [ddot_theta, ddot_x_body, ddot_phi] = solve([equ1, equ2, equ3], [ddot_theta, ddot_x_body, ddot_phi]);
    Ja = jacobian([dot_theta ddot_theta dot_x_body ddot_x_body dot_phi ddot_phi], [theta, dot_theta, x_body, dot_x_body, phi, dot_phi]);
    Jb = jacobian([dot_theta ddot_theta dot_x_body ddot_x_body dot_phi ddot_phi], [T, T_p]);
    
    A = simplify(vpa(subs(Ja, [theta, dot_theta, x_body, dot_x_body, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
    B = simplify(vpa(subs(Jb, [theta, dot_theta, x_body, dot_x_body, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
    
else
    [ddot_theta, ddot_x, ddot_phi] = solve([equ1, equ2, equ3], [ddot_theta, ddot_x, ddot_phi]);
    Ja = jacobian([dot_theta ddot_theta dot_x ddot_x dot_phi ddot_phi], [theta, dot_theta, x, dot_x, phi, dot_phi]);
    Jb = jacobian([dot_theta ddot_theta dot_x ddot_x dot_phi ddot_phi], [T, T_p]);
    
    A = simplify(vpa(subs(Ja, [theta, dot_theta, x, dot_x, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
    B = simplify(vpa(subs(Jb, [theta, dot_theta, x, dot_x, phi, dot_phi], [0, 0, 0, 0, 0, 0])));
end

%% LQR计算

leg_var = 0.096;
K=zeros(30,12);
leglen=zeros(30,1);
for i=1:30
    leg_var=leg_var+0.01; % 10mm线性化一次
    llm= 0.8424 * leg_var - 0.0272;
    ll = 0.1576 * leg_var + 0.0272;
    leglen(i)=leg_var;
    i_p = 0.0328 * leg_var + 0.0216;
    trans_A=double(subs(A,[L L_M I_p],[ll llm i_p]));
    trans_B=double(subs(B,[L L_M I_p],[ll llm i_p]));
    KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
    KK_t=KK.';
    K(i,:)=KK_t(:);
end

%% 系数拟合
K_cons=zeros(12,3);  %排列顺序是

for i=1:12
    res=fit(leglen,K(:,i),'poly2');
    K_cons(i,:)=[res.p1,res.p2,res.p3];
end
disp(K_cons)
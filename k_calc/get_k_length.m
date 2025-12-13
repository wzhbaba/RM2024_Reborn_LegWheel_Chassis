function K = get_k_length(leg_length)
   
    %theta : 摆杆与竖直方向夹角             R   ：驱动轮半径
    %x     : 驱动轮位移                    L   : 摆杆重心到驱动轮轴距离
    %phi   : 机体与水平夹角                LM  : 摆杆重心到其转轴距离
    %T     ：驱动轮输出力矩                 l   : 机体重心到其转轴距离
    %Tp    : 髋关节输出力矩                 mw  : 驱动轮转子质量
    %N     ：驱动轮对摆杆力的水平分量        mp  : 摆杆质量
    %P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量
    %Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量
    %Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量
    %Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量

    syms x(t) T R Iw mw M L LM theta(t) l phi(t) mp g Tp Ip IM
    syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

    R1=0.076;                         %驱动轮半径
    L1= 0.605814 * leg_length * leg_length + 0.962002 * leg_length - 0.062925;                   %摆杆重心到驱动轮轴距离
    LM1= - 0.605814 * leg_length * leg_length + 0.962002 * leg_length - 0.062925;                 %摆杆重心到其转轴距离
    l1=0.0058;                          %机体质心距离转轴距离
    
    mw1=1.35;                         %驱动轮质量
    mp1=0.987;                         %杆质量
    M1=5.577+2.234;                          %机体质量
    
    Iw1=0.01638248;                     %驱动轮转动惯量
    Ip1=0.311354 * leg_length * leg_length * leg_length -0.305621 * leg_length * leg_length + 0.076419 * leg_length + 0.017774; %摆杆转动惯量
    IM1=0.3513083215;       %机体绕质心转动惯量

    
    NM = M*diff(x + (L + LM )*sin(theta)-l*sin(phi),t,2);
    N = NM + mp*diff(x + L*sin(theta),t,2);
    PM = M*g + M*diff((L+LM)*cos(theta)+l*cos(phi),t,2);
    P = PM +mp*g+mp*diff(L*cos(theta),t,2);

    eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta,t,2) == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
    eqn3 = IM*diff(phi,t,2) == Tp +NM*l*cos(phi)+PM*l*sin(phi);
    
    eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);

    [f1,f2,f3] = solve(eqn10,eqn20,eqn30,f1,f2,f3);
   
    A=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[theta0,d_theta,x0,d_x,phi0,d_phi]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    A=subs(A,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.81]);
    A=double(A);
    B=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[T,Tp]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    B=subs(B,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.81]);
    B=double(B);

    %Q=diag([10, 5 ,50 ,100 ,400, 5]);%theta d_theta x d_x phi d_phi%10 1 100 600 4000 1
                                  %theta d_theta x d_x phi d_phi%        
     % R=[240 0;0 25];                %T Tp

    %R=diag([250, 25]); %这套速度权重要大于位移权重

    Q = diag([160 200 200 60 2000 80]);
    R = diag([1, 0.4]);

    % Q=diag([1000 1 1 2 20000 1]);
    % R=[1.5 0;0 0.25];

    K=lqr(A,B,Q,R);
    
end
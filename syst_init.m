% Initialization code for Simulink file - initialize the observer matrices

% 
%
% [1] - Tucci,Meng,Guerrero,Ferrari-Trecate (2016), 'A consensus-based 
%       secondary voltage control layer for stable current sharing and 
%       voltage balancing in DC microgrids', Technical Report. URL:
%       http://arxiv.org/abs/1603.03624.
clear

%% Simulation parameters

% Source Voltage
Vs = 80;

% Number of DGUs
N = 6;
% Number of states per DGU
n = 3;

% % tTot = 22;
% tTot = 16;
% % Time of connection of DGUs
% Tc = 1.5; 
% % Tplg = 1.5;
% Tplg = 8;
% Tunp = tTot;
% % Tunp = 15;
% Tcommfail = tTot-1;

Tc = 1.5;
T_zip_change = 6;
Tplg = 10;
T_zie_change = 17;
Tunp = 22;
tTot = 29;
Tcommfail = tTot-3;

Vref1 = 47;
Vref2 = 48;
Vref3 = 45;
Vref4 = 50;
Vref5 = 46;
Vref6 = 49;

VrefN = [Vref1;Vref2;Vref3;Vref4;Vref5;Vref6];

%% Define DGU parameters
% Electrical components - Local parameters
DGU(1).Rt = 2.0e-1;
DGU(1).Ct = 2.2e-3;
DGU(1).Lt = 1*1.8e-3;

DGU(2).Rt = 3.0e-1;
DGU(2).Ct = 1.9e-3;
DGU(2).Lt = 1*2.0e-3;

DGU(3).Rt = 1.0e-1;
DGU(3).Ct = 1.7e-3;
DGU(3).Lt = 1*2.2e-3;

DGU(4).Rt = 5.0e-1;
DGU(4).Ct = 2.5e-3;
DGU(4).Lt = 1*3.0e-3;

DGU(5).Rt = 4.0e-1;
DGU(5).Ct = 2.0e-3;
DGU(5).Lt = 1*1.3e-3;

DGU(6).Rt = 6.0e-1;
DGU(6).Ct = 3.0e-3;
DGU(6).Lt = 1*2.5e-3;

% Electrical interconnections 
DGU(1).Ni = [2,3,6];
DGU(2).Ni = [1,4];
DGU(3).Ni = [1,4];
DGU(4).Ni = [2,3,5];
DGU(5).Ni = [4,6];
DGU(6).Ni = [1,5];

% Electrical line parameters
% Line 1 : connects DGUs (1,2)
% Line 2 : connects DGUs (1,3)
% Line 3 : connects DGUs (1,6)
% Line 4 : connects DGUs (2,4)
% Line 5 : connects DGUs (3,4)
% Line 6 : connects DGUs (4,5)
% Line 7 : connects DGUs (5,6)

Rij_scale = 1e1;

% --1----------- 2 ------ 3 ------ 6 -----
DGU(1).Rij = [ 5.0e-2 ; 7.0e-2 ; 1.0e-1 ]*Rij_scale;
DGU(1).Lij = [ 2.1e-6 ; 1.8e-6 ; 2.5e-6 ];

% --2----------- 1 ------ 4 -----
DGU(2).Rij = [ 5.0e-2 ; 4.0e-2 ]*Rij_scale;
DGU(2).Lij = [ 2.1e-6 ; 2.3e-6 ];

% --3----------- 1 ------ 4 -----
DGU(3).Rij = [ 7.0e-2 ; 6.0e-2 ]*Rij_scale;
DGU(3).Lij = [ 1.8e-6 ; 1.0e-6 ];

% --4----------- 2 ------ 3 ------ 5 -----
DGU(4).Rij = [ 4.0e-2 ; 6.0e-2 ; 8.0e-2 ]*Rij_scale;
DGU(4).Lij = [ 2.3e-6 ; 1.0e-6 ; 1.8e-6 ];

% --5----------- 4 ------ 6 -----
DGU(5).Rij = [ 8.0e-2 ; 8.0e-2 ]*Rij_scale;
DGU(5).Lij = [ 1.8e-6 ; 3.0e-6 ];

% --6----------- 1 ------ 5 -----
DGU(6).Rij = [ 1.0e-1 ; 8.0e-2 ]*Rij_scale;
DGU(6).Lij = [ 2.5e-6 ; 3.0e-6 ];


% Primary controllers
DGU(1).K  = [-2.134,-0.163,13.553,-2.134];
DGU(2).K  = [-0.869,-0.050,48.285,-0.869];
DGU(3).K  = [-0.480,-0.108,30.673,-0.480];
DGU(4).K  = [-6.990,-0.175,102.960,-6.990];
DGU(5).K  = [-0.101,-0.010,16.393,-0.101];
DGU(6).K  = [-2.134,-0.163,13.553,-2.134];


% Secondary controller rated currents
DGU(1).Ir = 1;
DGU(2).Ir = 1.25;
DGU(3).Ir = 1.5;
DGU(4).Ir = 1.75;
DGU(5).Ir = 2;
DGU(6).Ir = 2.25;

% Load parameters 
Il_bar1 = 5/3;
Pl_bar1 = 100/3;%*1e1;
Yl_bar1 = 1/10/3;%*1e-2;

Il_bar2 = 6/3;
Pl_bar2 = 120/3;%*1e1;
Yl_bar2 = 1/9/3;%*1e-2;

Il_bar3 = 7/3;
Pl_bar3 = 105/3;%*1e1;
Yl_bar3 = 1/11/3;%*1e-2;

Il_bar4 = 6.5/3;
Pl_bar4 = 95/3;%*1e1;
Yl_bar4 = 1/11/3;%*1e-2;

Il_bar5 = 8/3;
Pl_bar5 = 110/3;%*1e1;
Yl_bar5 = 1/10/3;%*1e-2;

Il_bar6 = 7.5/3;
Pl_bar6 = 105/3;%*1e1;
Yl_bar6 = 1/10.5/3;%*1e-2;

Yl = diag([Yl_bar1,Yl_bar2,Yl_bar3,Yl_bar4,Yl_bar5,Yl_bar6]);
Il = [Il_bar1;Il_bar2;Il_bar3;Il_bar4;Il_bar5;Il_bar6];
Plstar = [Pl_bar1;Pl_bar2;Pl_bar3;Pl_bar4;Pl_bar5;Pl_bar6];

% Linear dynamics of DGUi
DGU = stateDyn(DGU,1);
DGU = stateDyn(DGU,2);
DGU = stateDyn(DGU,3);
DGU = stateDyn(DGU,4);
DGU = stateDyn(DGU,5);
DGU = stateDyn(DGU,6);

Aconn = zeros(n*N);
for i = 1:N
    for j = 1:N
        if size(find(DGU(i).Ni == j),2) ~= 0
            jj = find(DGU(i).Ni == j);
            Aconn(n*(i-1)+1:n*i,n*(j-1)+1:n*j) = DGU(i).Aij(:,n*(jj-1)+1:n*jj);
        elseif i == j 
            Aconn(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = DGU(i).A;
        end
    end
end

Adisc = zeros(n*N);
for i = 1:N
    tmpA = DGU(i).A;
    %       Set Aii(1,1) = 0;
    tmpA(1) = 0;
    Adisc((i-1)*n+1:i*n,(i-1)*n+1:i*n) = tmpA;
end

Aii = blkdiag(DGU.A);
Aij = Aconn-Aii;
B = blkdiag(DGU.B);
K = blkdiag(DGU.K);
M = blkdiag(DGU.M);
Ir = [DGU.Ir]';

%% BUILD LAPLACIAN MATRICES
 
D = diag(Ir.^-1);

%communication incidence matrix
% Bc = [1, 1, 1, 0, 0, 0, 0;
%      -1, 0, 0, 1, 0, 0, 0;
%       0,-1, 0, 0, 1, 0, 0;
%       0, 0, 0,-1,-1, 0, 1;
%       0, 0, 0, 0, 0, 1,-1;
%       0, 0,-1, 0, 0,-1, 0];
  
   
% Bc = [1, 1, 0, 0, 0;
%       0, 0, 1, 0, 0;
%      -1, 0, 0, 0, 0;
%       0, 0,-1, 0, 1;
%       0, 0, 0, 1,-1;
%       0,-1, 0,-1, 0];

Bc_first = [1, 1, 0, 0, 0;
            0, 0, 1, 0, 0;
           -1, 0,-1, 1, 1;
            0,-1, 0,-1, 0;
            0, 0, 0, 0,-1;
            0, 0, 0, 0, 0];
  
Bc_full = [1, 1, 0, 0, 0, 0;
           0, 0, 1, 0, 0, 0;
          -1, 0,-1, 1, 1, 1;
           0,-1, 0,-1, 0, 0;
           0, 0, 0, 0,-1, 0;
           0, 0, 0, 0, 0,-1];

Bc_unp = [1, 1, 0, 0, 0;
          0, 0, 1, 0, 0;
         -1, 0,-1, 1, 1;
          0,-1, 0,-1, 0;
          0, 0, 0, 0, 0;
          0, 0, 0, 0,-1];
  
Wcomm_first = eye(size(Bc_first,2)); 
Wcomm_full = eye(size(Bc_full,2)); 
Wcomm_unp = eye(size(Bc_unp,2)); 
L_first = Bc_first*Wcomm_first*(Bc_first');
L_full = Bc_full*Wcomm_full*(Bc_full');    %communication Laplacians 
L_unp = Bc_unp*Wcomm_unp*(Bc_unp');

%electrical graphs
% ---------------------1/R12-----------1/R24-----------1/R13---------------1/R34-----------1/R45
Wel_first = diag([1/DGU(1).Rij(1), 1/DGU(2).Rij(2), 1/DGU(1).Rij(2), 1/DGU(3).Rij(2), 1/DGU(4).Rij(3)]);

% ---------------------1/R12-----------1/R24-----------1/R13---------------1/R34-----------1/R45----------1/R16--------------1/R56
Wel = diag([1/DGU(1).Rij(1), 1/DGU(2).Rij(2), 1/DGU(1).Rij(2), 1/DGU(3).Rij(2), 1/DGU(4).Rij(3), 1/DGU(1).Rij(3), 1/DGU(5).Rij(2)]);

% ---------------------1/R12-----------1/R24-----------1/R13---------------1/R34-----------1/R16
Wel_unp = diag([1/DGU(1).Rij(1), 1/DGU(2).Rij(2), 1/DGU(1).Rij(2), 1/DGU(3).Rij(2), 1/DGU(1).Rij(3)]);

Bel_first = [1, 0, 1, 0, 0;
            -1, 1, 0, 0, 0;
             0, 0,-1, 1, 0;
             0,-1, 0,-1, 1;
             0, 0, 0, 0,-1];
   
Bel = [1, 0, 1, 0, 0, 1, 0;
      -1, 1, 0, 0, 0, 0, 0;
       0, 0,-1, 1, 0, 0, 0;
       0,-1, 0,-1, 1, 0, 0;
       0, 0, 0, 0,-1, 0,-1;
       0, 0, 0, 0, 0,-1, 1];

Bel_unp = [1, 0, 1, 0, 1;
          -1, 1, 0, 0, 0;
           0, 0,-1, 1, 0;
           0,-1, 0,-1, 0;
           0, 0, 0, 0,-1];

M_first = Bel_first*Wel_first*(Bel_first');    %first electrical Laplacian
M = Bel*Wel*(Bel');    %full electrical Laplacian
M_unp = Bel_unp*Wel_unp*(Bel_unp');    %unplugged electrical Laplacian

% analysis of equilibria - first mG

N1 = ones(N-1,1);
D_first = diag(Ir(1:N-1).^(-1));
invD = inv(D_first);
Lt = invD - inv(N1'*invD*N1)*invD*N1*N1'*invD;
Yl_first = Yl(1:5,1:5);

Lp = M_first + Lt*D_first*Yl_first;

Ltilde = [Lp; N1'*invD];

Ltildet = [Lt*D_first;zeros(1,N-1)];

Itilde = [-Lt*D_first*Il(1:N-1);N1'*invD*VrefN(1:N-1)];

invLtilde = pinv(Ltilde);
 
disp('FIRST:')

Vstar = invLtilde*Itilde

Pcri = 4*inv(diag(Vstar))*invLtilde*Ltildet*inv(diag(Vstar));

Delta = norm(Pcri*Plstar(1:N-1),Inf)

syms delta 
deltas = double(solve(4*delta*(1-delta)-Delta));
deltam = deltas(1)
deltap = deltas(2)

(1-deltam)*Vstar
(1+deltam)*Vstar
(1-deltap)*Vstar


% normWel = norm(Wel_first)
% normLe = norm(M_first)
% normLtilde = norm(Ltilde)
% normLtildeinv = norm(pinv(Ltilde))
% normVstar = norm(Vstar)
% normVstarinv = norm(inv(diag(Vstar)))



% % analysis of equilibria - ZIP-modified mG
% 
% N1 = ones(N-1,1);
% D_first = diag(Ir(1:N-1).^(-1));
% invD = inv(D_first);
% Lt = invD - inv(N1'*invD*N1)*invD*N1*N1'*invD;
% Yl_first = Yl(1:5,1:5);
% Yl_first(1,1) = 1.3*Yl_first(1,1);
% Yl_first(4,4) = 1.2*Yl_first(4,4);
% 
% Lp = M_first + Lt*D_first*Yl_first;
% 
% Ltilde = [Lp; N1'*invD];
% 
% Ltildet = [Lt*D_first;zeros(1,N-1)];
% 
% Itilde = [-Lt*D_first*Il(1:N-1);N1'*invD*VrefN(1:N-1)];
% 
% invLtilde = pinv(Ltilde);
%  
% disp('ZIP MODIFIED:')
% 
% Vstar = invLtilde*Itilde
% 
% Pcri = 4*inv(diag(Vstar))*invLtilde*Ltildet*inv(diag(Vstar));
% 
% Plstar_first = Plstar(1:N-1);
% Plstar_first(1) = 1.3*Plstar_first(1);
% Plstar_first(4) = 1.2*Plstar_first(4);
% 
% Delta = norm(Pcri*Plstar_first,Inf);
% 
% syms delta 
% deltas = double(solve(4*delta*(1-delta)-Delta));
% deltam = deltas(1)
% deltap = deltas(2)
% 
% (1-deltam)*Vstar
% (1+deltam)*Vstar
% (1-deltap)*Vstar


% % analysis of equilibria - full mG
% 
% N1 = ones(N,1);
% invD = inv(D);
% Lt = invD - inv(N1'*invD*N1)*invD*N1*N1'*invD;
% 
% Lp = M + Lt*D*Yl;
% 
% Ltilde = [Lp; N1'*invD];
% 
% Ltildet = [Lt*D;zeros(1,N)];
% 
% Itilde = [-Lt*D*Il;N1'*invD*VrefN];
% 
% invLtilde = pinv(Ltilde);
% 
% disp('FULL:') 
% 
% Vstar = invLtilde*Itilde
% 
% Pcri = 4*inv(diag(Vstar))*invLtilde*Ltildet*inv(diag(Vstar));
% 
% Delta = norm(Pcri*Plstar,Inf);
% 
% syms delta 
% deltas = double(solve(4*delta*(1-delta)-Delta));
% deltam = deltas(1)
% deltap = deltas(2)
% 
% (1-deltam)*Vstar
% (1+deltam)*Vstar
% (1-deltap)*Vstar


% % analysis of equilibria - unplugged mG
% 
% N1 = ones(N-1,1);
% D_unp = diag(Ir([1,2,3,4,6]).^(-1));
% invD = inv(D_unp);
% Lt = invD - inv(N1'*invD*N1)*invD*N1*N1'*invD;
% Yl_first = Yl([1,2,3,4,6],[1,2,3,4,6]);
% 
% Lp = M_unp + Lt*D_unp*Yl([1,2,3,4,6],[1,2,3,4,6]);
% 
% Ltilde = [Lp; N1'*invD];
% 
% Ltildet = [Lt*D_unp;zeros(1,N-1)];
% 
% Itilde = [-Lt*D_unp*Il([1,2,3,4,6]);N1'*invD*VrefN([1,2,3,4,6])];
% 
% invLtilde = pinv(Ltilde);
%  
% disp('UNPLUGGED:')
% 
% Vstar = invLtilde*Itilde
% 
% Pcri = 4*inv(diag(Vstar))*invLtilde*Ltildet*inv(diag(Vstar));
% 
% Delta = norm(Pcri*Plstar([1,2,3,4,6]),Inf);
% 
% syms delta 
% deltas = double(solve(4*delta*(1-delta)-Delta));
% deltam = deltas(1)
% deltap = deltas(2)
% 
% (1-deltam)*Vstar
% (1+deltam)*Vstar
% (1-deltap)*Vstar



 
 
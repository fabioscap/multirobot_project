% Implementation of Consensus strategy on the swarm of drones
clear all; close all; clc; addpath(genpath(pwd));


%Now we have to build the undirected graph structure that defines the
%neighbors of each agent. We encode these infomrations in a binary matrix
%where 1 in position ij indicates that agent i and j are connected, while 0
%means that they are not connected. For implementation reasons, when i=j we
%put 1 as value. So the diagonal is made of all ones.

% Graph = [1 1 0 0 1;
%          1 1 1 0 0;
%          0 1 1 1 0;
%          0 0 1 1 1;
%          1 0 0 1 1
%          ];

dim =2 ;

% the actual target pose (unknown to the agents)
% [ x; y];
p_t = zeros(dim, 1);

p_t = [10;20];

% the number of agents
n_agents = 5;


% the sampling time for the ARTVA signal
% agents receive samples at discrete intervals
tau = 0.5;

%Gains values
Kp = 1;
Kd = 1;

e = Environment2(dim, n_agents, p_t, tau);

for i=1:n_agents
    e.RLSStep_Consensus(i);
end

%Here we build the differential equation x_dot = M * x, which will be used
%into ode45 as f

%The x vector is built with this order:
%   -agents positions, (x_i,y_i) if 2D, (x_i, y_i, z_i) if 3D.
%   -first derivatives of positions (xd_i, yd_i, zd_i)
%   -agents estimatations of target location,i.e p_hat, that is either 2D
%    or 3D
% So the total number of elements is: N_agents*dim*3
% (Ex: 5 agents, 2D -->  5*2*3 = 30)
%
%The x_dot vector contains instead:
%   -first derivatives of positions
%   -second derivatives of positions 
%   -first derivatives of estimations of target locations
%
%For computing the derivatives of target locations, so pd_hat,the laplacian
% matrix L is used. By storing it in memory, we will call its rows to then
%multiply them to the vector that contains all p_hat.
%In formula:  
%                 pd_hat =            L         *        p_hat
% Dimensions:   N_ags x dim     N_ags * N_ags       (N_ags*dim)xdim
% Along the second dimension (dim) are stored the x,y and possibly z, of
% each agent.


%Degree matrix
D = 2*eye(n_agents);
%Adjancey matrix
A = e.graph -eye(n_agents);
%Laplacian Matrix
L = D - A;

n_entries = n_agents*e.dim;
vv = n_entries*3; %this shpul be 30. if dim = 2
p_hat = zeros(n_agents,dim);
pd_hat = zeros(n_agents,dim);

% %p_hat stores the estimates in a vector of dimension: n_agent x dim
% for i= 1:n_agents
%     p_hat(i,1) = e.p_hat(1,i);
%     p_hat(i,2) = e.p_hat(2,i);
% end
% 
% 
% for i= 1:n_agents
%     pd_hat(i,1) = L(i,:) * e.p_hat(1,:)';
%     pd_hat(i,2) = L(i,:) * e.p_hat(2,:)';
% end
% 
% %First n_entries of entries of x_dot are velocities of agents, second
% % n_entries are the accelerations of agents and the last n_entries are
% % derivatives of estimations of target, i.e p_dot_hat
% x_dot = zeros(vv,1);
% iterator = 0;
% for s=1:(vv/2) %this should be 30/2 = 15
% 
%     if s<(n_agents+1) % This fill the first 10 entries of x_dot
%         x_dot((s*2)-1) = e.agents(round(s/2)).x(1);
%         x_dot(s*2) = e.agents(round(s/2)).x(2);
%     elseif s<(2*n_agents +1) %This fill the second 10 entries of x_dot
%         %This is the x coordinate of the agent
%         x_dot((s*2)-1) = -Kp*e.agents(round((s-n_agents)/2)).x(1) ...
%             -Kd*e.agents(round((s-n_agents)/2)).x(3)...
%             + Kp*p_hat(round((s-n_agents)/2),1) ...
%             -Kd*pd_hat(round((s-n_agents)/2),1);
%         %This is the y coordinate of the agent
%         x_dot(s*2) = -Kp*e.agents(round((s-n_agents)/2)).x(2) ...
%             -Kd*e.agents(round((s-n_agents)/2)).x(4)...
%             + Kp*p_hat(round((s-n_agents)/2),2) ...
%             -Kd*pd_hat(round((s-n_agents)/2),2);
%         %TODO variable implementation for including the z, if needed
%     else    %This fill the last 10 entries 
%         x_dot((s*2)-1)  = pd_hat(round((s-2*n_agents)/2),1);
%         x_dot(s*2) =  pd_hat(round((s-2*n_agents)/2),2);
%     end
% end
% 

syms px_1 py_1 px_2 py_2 px_3 py_3 px_4 py_4 px_5 py_5 real
syms pdx_1 pdy_1 pdx_2 pdy_2 pdx_3 pdy_3 pdx_4 pdy_4 pdx_5 pdy_5 real
syms x1 y1 x2 y2 x3 y3 x4 y4 x5 y5 xd1 yd1 xd2 yd2 xd3 yd3 xd4 yd4 xd5 yd5 real
%uncomment if 3d
%syms pz_1 pz_2 pz_3 pz_4 pz_5 pdz_1 pdz_2 pdz_£ pdz_4 pdz_5 real
%syms z1 z2 z3 z4 z5  zd1 zd2 zd3 zd4 zd5 real

%We want to write the formula of x_dot = M*x
 x = [x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,...
     xd1,yd1,xd2,yd2,xd3,yd3,xd4,yd4,xd5,yd5,...
     px_1,py_1,px_2,py_2,px_3,py_3,px_4,py_4,px_5,py_5]';
 %Let's create a comfortable structure to call elements of L
 L_vector = zeros(2*(length(L)^2),1);
 appoggio = reshape(L,length(L)^2,1);
%  for k = 1:length(L)^2
%      L_vector((k*2)-1) = appoggio(k);
%      L_vector(k*2) =appoggio(k) ;
%  end
%  clear appoggio

L_net = [L(1,1), 0, L(1,2), 0, L(1,3), 0, L(1,4), 0, L(1,5), 0;
         0, L(1,1), 0, L(1,2), 0, L(1,3), 0, L(1,4), 0, L(1,5);
         L(2,1), 0, L(2,2), 0, L(2,3), 0, L(2,4), 0, L(2,5), 0;
         0, L(2,1), 0, L(2,2), 0, L(2,3), 0, L(2,4), 0, L(2,5);
         L(3,1), 0, L(3,2), 0, L(3,3), 0, L(3,4), 0, L(3,5), 0;
         0, L(3,1), 0, L(3,2), 0, L(3,3), 0, L(3,4), 0, L(3,5);
         L(4,1), 0, L(4,2), 0, L(4,3), 0, L(4,4), 0, L(4,5), 0;
         0, L(4,1), 0, L(4,2), 0, L(4,3), 0, L(4,4), 0, L(4,5);
         L(5,1), 0, L(5,2), 0, L(5,3), 0, L(5,4), 0, L(5,5), 0;
         0, L(5,1), 0, L(5,2), 0, L(5,3), 0, L(5,4), 0, L(5,5)
         ];

 M = zeros(30,30);
 %Now let's build x_dot
 for s = 1:n_entries
     M(s,s+n_entries) = 1;
 end

for s = (n_entries+1):2*n_entries
    M(s,s-n_entries) = -Kp;
    M(s,s) = -Kd;
    M(s,s+n_entries) = Kp;

end


M(n_entries+1:2*n_entries, 2*n_entries+1:end) = ...
    M(n_entries+1:2*n_entries, 2*n_entries+1:end) +L_net;

M(2*n_entries+1:end, 2*n_entries+1:end) =...
    M(2*n_entries+1:end, 2*n_entries+1:end) +L_net;

%Build x_0 for ode45
x_0 = zeros(length(x),1);
for s=1:(vv/2) %this should be 30/2 = 15

    if s<(n_agents+1) % This fill the first 10 entries of x_dot
        x_0((s*2)-1) = e.agents(round(s/2)).x(1);
        x_0(s*2) = e.agents(round(s/2)).x(2);
    elseif s<(2*n_agents +1) %This fill the second 10 entries of x_dot
        x_0((s*2)-1) = e.agents(round((s-n_agents)/2)).x(3);
        x_0(s*2) = e.agents(round((s-n_agents)/2)).x(4);
    else
        x_0((s*2)-1) = e.p_hat(1,s-n_entries);
        x_0(s*2) = e.p_hat(2,s-n_entries);
    end
end



t_interval =[0,1.5];
x_dot = @(t,x) M*x;

[t,x] = ode45(x_dot,t_interval,x_0);

figure,
hold on
for i=1:5
    plot(x(:,i*2 -1),x(:,i*2))
end
hold off
legend('agente1','agente2','agente3','agente4','agente5')

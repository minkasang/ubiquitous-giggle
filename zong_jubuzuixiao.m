%%
clc
clear
close all
%只是演示，实际应用中不用画图
%加入更改障碍物矩阵，
%在准备超车阶段，车辆应该是加速的，然而势场法是相反的，那时候受到的势场力抵消很多，变得不是很大。
%----------------------------------------------初始化设置（都是可以更改的）----------------------------------------------%
start=[1 -1.75 0 0];%起点
goal=[23 -1.75];%目标点
x=start(1,1);
y=start(1,2);
s=0.01;%小车步长
reptiao_x=5;
reptiao_y=1;
% [I map alpha]=imread('icon.png');%
% 
% markersize=[5 1];
% 
% h=image(I);
% set(h,'AlphaData',alpha);
%-----------------------------障碍物的运动
obs_1=[50 -1 -0.05 0;
      20 -2.5 0.35 -0.00001;
      25 1 0.05 -0.02];%前两个是障碍物位置，后两个分别是x y上的位移
  obs_1=[10 -1.75 0 0;];%前两个是障碍物位置，后两个分别是x y上的位移
  %% 
  
%-------------------------------------------------画图-------------------------------------------------%
hold on
%--------------------------------------------------路
ry=3.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% road_1 = [0,100,100,0];   % 逆时针遍历每个点的x值
% road_2= [-3.5,-3.5,3.5,3.5];   % 逆时针遍历每个点的y值（道路）
% grass_1 = [0,100,100,0];   % 逆时针遍历每个点的x值
% grass_2= [3.5,3.5,20,20];   % 逆时针遍历每个点的y值（道路）
% grass_3 = [0,100,100,0];   % 逆时针遍历每个点的x值
% grass_4= [-3.5,-3.5,-20,-20];   % 逆时针遍历每个点的y值（道路）
% fill(road_1,road_2,[0.16 0.14 0.25]);   % 填充函数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% road_1 = [0,25,25,0];   % 逆时针遍历每个点的x值
% road_2= [-3.5,-3.5,3.5,3.5];   % 逆时针遍历每个点的y值（道路）
% grass_1 = [0,25,25,0];   % 逆时针遍历每个点的x值
% grass_2= [3.5,3.5,25,25];   % 逆时针遍历每个点的y值（道路）
% grass_3 = [0,25,25,0];   % 逆时针遍历每个点的x值
% grass_4= [-3.5,-3.5,-25,-25];   % 逆时针遍历每个点的y值（道路）
% % fill(road_1,road_2,[0.16 0.14 0.25]);   % 填充函数
% fill(road_1,road_2,[1 1 1]);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fill(grass_1,grass_2,[0.35 0.6627 0.352941176]);
% % fill(grass_3,grass_4,[0.42 0.7 0.14]);
% fill(grass_3,grass_4,[0.35 0.6627 0.352941176]);
%%
len_line=25;
% plot([0, len_line],[0, 0], 'y--', 'linewidth',2);  %分界线
plot([0, len_line],[-10, 10], 'w', 'linewidth',2);  %分界线
plot([0, len_line],[-3.4,3.4 ], 'w', 'linewidth',2);  %分界线
% xlim([-10 10]);
% ylim([0 25]);

%%
%------------------------------------------------障碍物
D=plot(obs_1(:,1),obs_1(:,2),'ks','LineWidth',13);
DD=plot(obs_1(:,1)+1,obs_1(:,2),'ks','LineWidth',11);%两个障碍物和一起作为长方形障碍物的，没有实际意义

% axis equal
%% 
plot(goal(1),goal(2),'go','LineWidth',12)            %先画目标点图
plot(start(1,1),start(1,2),'r*','LineWidth',8)            %画起点图，只是出图要用
% xlim([-10 10]);
% ylim([0 25]);

axis([0 105 -3.6 3.6]) ;              %定义坐标轴
%-------------------------------------------------动图
count=0;
count1=0;
%%

new=[0 1];
last=[0 0];

%%
while sqrt((goal(1)-x)^2 + (goal(2)-y)^2)>1 %之所以是1因为函数gravitation输出值平凡根和不会超过1
    %---------------------自身的运动
   [gra_x, gra_y] = gravitation(x,y,goal(1),goal(2));%最大一百，可以改成恒值。定义一个求偏导数的函数gravitation
   [roadrep_y]= roadrepulsionnew(y, ry);
   rep_x=0;
   rep_y=0;
   for i=1:size(obs_1,1)
   obs_1(i,1)=obs_1(i,1)+obs_1(i,3);%更新移动障碍物的运动轨迹
   obs_1(i,2)=obs_1(i,2)+obs_1(i,4);
   [rep_xx, rep_yy] = repulsion(x,y,obs_1(i,1),obs_1(i,2));%定义一个求偏导数的函数repulsion
   rep_x=rep_x+rep_xx;
   rep_y=rep_y+rep_yy;
   end
   sx=gra_x+rep_x*reptiao_x;          %x轴上的分力
   sy=gra_y+rep_y*reptiao_y+roadrep_y;%y轴上的分力
   
   count=count+1;
%    count_1(1,count)=gra_y;
%    count_1(2,count)=roadrep_y;%查看里面输出情况时用
   
   xita=abs(atan(sy/sx));%合力与笛卡尔坐标之间的夹角;同时去掉cos(xita)的正负号，全部用gravitation函数输出的正负值即可。
   sx_1=sign(sx);%提取正负号
   sy_2=sign(sy);
   
   %------准备设置动力学约束
   
   
   sx=s*cos(xita)*sx;%步长在x轴上的位移
   sy=s*sin(xita)*sy;%步长在y轴上的位移
   
   
   
   %-------设置未完成，回头再弄
   x=x+sx;
   y=y+sy;
   
%    x_low = x - markersize(1)/2; %//Left edge of marker
% 
%    x_high = x + markersize(1)/2;%//Right edge of marker
% 
%    y_low = y - markersize(2)/2; %//Bottom edge of marker
% 
%    y_high = y + markersize(2)/2;%//Top edge of marker
%   
%    
%    imagesc([x_low x_high], [y_low y_high],I)
   
   
   %---------------------障碍物的运动
   
   last=new;
   new=[x y];
   last1(count,:)=last;
   new1(count,:)=new;
   sqrt((new(1,1)-last(1,1))^2+(new(1,2)-last(1,2))^2);
   
   %---------------局部极小值解决方法
   if sqrt((new(1,1)-last(1,1))^2+(new(1,2)-last(1,2))^2)<0.005%距离很小判定
     goalv=[10 1.75];%虚拟目标点设置，
%      break
     while y<-0.75
     [grav_x, grav_y] = gravitation(x,y,goalv(1),goalv(2));%最大一百，可以改成恒值。定义一个求偏导数的函数gravitation
     [gra_x, gra_y] = gravitation(x,y,goal(1),goal(2));
   [roadrep_y]= roadrepulsionnew(y, ry);
   rep_x=0;
   rep_y=0;
   for i=1:size(obs_1,1)
   obs_1(i,1)=obs_1(i,1)+obs_1(i,3);%更新移动障碍物的运动轨迹
   obs_1(i,2)=obs_1(i,2)+obs_1(i,4);
   [rep_xx, rep_yy] = repulsion(x,y,obs_1(i,1),obs_1(i,2));%定义一个求偏导数的函数repulsion
   rep_x=rep_x+rep_xx;
   rep_y=rep_y+rep_yy;
   end
   sx=gra_x+rep_x*reptiao_x+grav_x;          %x轴上的分力
   sy=gra_y+rep_y*reptiao_y+roadrep_y+grav_y;%y轴上的分力
   xita=abs(atan(sy/sx));%合力与笛卡尔坐标之间的夹角;同时去掉cos(xita)的正负号，全部用gravitation函数输出的正负值即可。
   sx_1=sign(sx);%提取正负号
   sy_2=sign(sy);
   
   sx=s*cos(xita)*sx;%步长在x轴上的位移
   sy=s*sin(xita)*sy;%步长在y轴上的位移
   
   x=x+sx;
   y=y+sy;
   Ex(count,1)=x;
   Ey(count,1)=y;
%    if sx==0%力大小判定
       zidong=1
%        break
plot(x,y,'b.')%自身运动
   axis image
%    axis equal
   set(D,'XData',obs_1(:,1),'YData',obs_1(:,2),'MarkerSize',5)%障碍物们的运动
   set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.32]);%图窗口大小
   set (gca,'position',[[0.1,0.1,0.8,0.8] ] );%图像大小
   drawnow
   pause(0.002);
     end

   end
   
   Ex(count,1)=x;
   Ey(count,1)=y;
   
   
   
%    if count>100
%        break
%    end    %当时为了极小值点用的
       
   plot(x,y,'r.')%自身运动
   axis image
%    axis equal
   set(D,'XData',obs_1(:,1),'YData',obs_1(:,2),'MarkerSize',5)%障碍物们的运动
   set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.32]);%图窗口大小
   set (gca,'position',[[0.1,0.1,0.8,0.8] ] );%图像大小
   drawnow
   pause(0.002);
   
end
% <!-- Zenoah G-26A -->
% <!-- 2.96 hp engine -->
% <!-- one horsepower equals 745.69987 Watts -->
MaxPower = 2207.27 ; % [W]
Propeller.ixx = 0.00085;
Propeller.d = 18*0.0254; % [m]
Propeller.bladenum = 2;
Propeller.maxpitch = 30; % [deg]
Propeller.minpitch = 30; % [deg]

JTable       =[0      0.1     0.2    0.3    0.4    0.5    0.6    0.7    0.8    1.0     1.4]';
CthrustTable =[0.0776 0.0744  0.0712 0.0655 0.0588 0.0518 0.0419 0.0318 0.0172 -0.0058 -0.0549]';
CpowerTable  =[0.0902 0.0893  0.0880 0.0860 0.0810 0.0742 0.0681 0.0572 0.0467 0.0167 -0.0803]';


% V= 32 ; % [m/s] ƽ���ٶȣ��趨ֵ
% thrinput = 0.15; % [-] ����������������
% rho = 1.225; % �����ܶ�
% n=(10:1:2300/60)'; % [r/s] 

V= 20 ; % [m/s] ƽ���ٶȣ��趨ֵ
thrinput = 0.15; % [-] ����������������
rho = 1.225; % �����ܶ�
n=(1:1:50)'; % [r/s] 

J=V./n/Propeller.d;
Cp=interp1(JTable,CpowerTable,J,'linear','extrap');
Ct=interp1(JTable,CthrustTable,J,'linear','extrap');
eta = Ct./Cp.*J
Pneed=Cp*rho.*n.^3*Propeller.d^5;
Pavailable = 0.*n+(MaxPower*thrinput).*eta;
plot(n,Pneed,'--',n,Pavailable,'-');
nbalance=40

Jbalance = V/nbalance/Propeller.d;
Ctbalance = interp1(JTable,CthrustTable,Jbalance);
T=Ctbalance*rho*nbalance^2*Propeller.d^4 % ���� N 

% ��֪��Ϊʲô�����㲻�ԡ����������ˣ���֮������2.96horsepower�µ�����Լ3kg���ң����������ͨ��flightgear���������˸�rascal�ܷ��濴properties���ݿ�����
% ��BWB3�ķɻ�����������������������ҲԼ3kg���ң��ҿ��Ǹɴ��ڸ�ʲô�ط��˸�6���ˡ�������



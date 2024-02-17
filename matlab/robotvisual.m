alpha = [0 -90 0 -90 -90 -90];
a = [2 0 6 2 0 0];
d = [4.5 0 6 0 0 3];
theta = deg2rad([60 90 0 150 120 0]);

H0_1 = Link([theta(1), d(1), a(1), alpha(1), 1]);
H0_1.qlim = [0 5];

H1_2 = Link([theta(2), d(2), a(2), alpha(2), 0]);
H1_2.qlim = [-pi/2 pi/2];

H2_3 = Link([theta(3), d(3), a(3), alpha(3), 0]);
H2_3.qlim = [-pi/2 pi/2];

H3_4 = Link([theta(4), d(4), a(4), alpha(4), 0]);
H3_4.qlim = [-pi/2 pi/2];

H4_5 = Link([theta(5), d(5), a(5), alpha(5), 0]);
H4_5.qlim = [-pi/2 pi/2];

H5_6 = Link([theta(6), d(6), a(6), alpha(6), 0]);
H5_6.qlim = [-pi/2 pi/2];

Kukarobot = SerialLink([H0_1 H1_2 H2_3 H3_4 H4_5 H5_6], 'name', 'KUKA KR8 R 1420 HW');
Kukarobot.plot([0 0 0 0 0 0], 'workspace', [-5 18 -18 18 0 18]);
Kukarobot.teach;
load('analysis.mat');
RRT = mean(RRT, 2);
RRT_Connect = mean(RRT_Connect, 2);
RRT_Star = mean(RRT_Star, 2);
PRM = mean(PRM, 2);

data = [RRT RRT_Connect RRT_Star PRM];
figure(1)
bar(data(1,:))
title('Avg. Plan Cost')

figure(2)
bar(data(2,:))
title('Avg. Runtime')

figure(3)
bar(data(3,:))
title('Avg. Plan Step')
%% calculate the cost from the goal point to current point
function cost = getHcost(startPoint,goalPoint,n)
[xs,ys]=ind2sub([n,n],startPoint.num);
[xg,yg]=ind2sub([n,n],goalPoint.num);
cost = abs(xs-xg)+abs(ys-yg);
end
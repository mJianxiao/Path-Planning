%% calculate the cost from the start point to current point
function cost = getGcost(cpoint,n)
global point
    [xf,yf]=ind2sub([n,n],cpoint.father);
    [xs,ys]=ind2sub([n,n],cpoint.num);
    if xs==xf || yf==ys
        cost = point(cpoint.father).Gcost+1.0;
    else
        cost = point(cpoint.father).Gcost+1.4;
    end
end
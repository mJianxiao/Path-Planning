%% find the way back
function [COOR]=routPlot(goalNum,n)
global point
ii = point(goalNum).num;
rout= [ii];
if ~isempty(point(goalNum).father)
    while point(ii).father ~= point(ii).num
        rout=[point(ii).father,rout];
        ii = point(ii).father;
    end
end
[y,x]= ind2sub([n,n],rout);
COOR(:,1) = x;
COOR(:,2) = y;
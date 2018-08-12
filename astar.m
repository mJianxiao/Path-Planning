function Pathpoints = astar(map,starpoint,targetpoint) 

botSim = BotSim(map);

limsMin = min(map);
limsMax = max(map);
dims = limsMax - limsMin;
res = 1;
iterators = dims/res;
iterators = ceil(iterators)+[1 1];
mapArray = zeros(iterators);

map(length(map)+1,:)=map(1,:);
edge=zeros(length(map)-1,4);
for i=1:size(edge,1)
    edge(i,:)=[map(i,:) map(i+1,:)];
end

for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1]*res;
        distan=min(disToLineSeg(testPos,edge));
        mapArray(i,j) = botSim.pointInsideMap(testPos);
        if distan<10
            mapArray(i,j)=0;
        end
    end
end
if iterators(1) > iterators(2)
    n = iterators(1);
else
    n = iterators(2);
end
starpoint = starpoint-limsMin;
starNum = (starpoint(1,1)-1)*n + starpoint(1,2);

targetpoint = targetpoint-limsMin;
goalNum = (targetpoint(1,1)-1)*n + targetpoint(1,2);

global point 
hh = 0;
banList = [];

for ii=1:n*n
    point(ii).num = ii;
    point(ii).father=[];
    point(ii).Gcost=[];
    point(ii).Hcost=[];
    if mapArray(ii) == 0
        hh = hh + 1;
        banList(hh) = ii;
    end
end

banList(find(banList==goalNum))=[];
for jj = 1:length(banList)
    if banList(jj)~=goalNum || banList(jj)~=starNum
        point(banList(jj)).Gcost = Inf;
    end
end
point(starNum).Gcost=0;
point(starNum).father = point(starNum).num;
point(starNum).Hcost=getHcost(point(starNum),point(goalNum),n);

openList = [];
closeList = [];
closeListNum=[];
openListNum=[];
openList = [openList,point(starNum)];
while length(openList)
    costList = getCost(openList,point(goalNum),n);
    currentPoint = openList(find(costList==min(costList),1));
    openList(find(min(costList)==costList,1))=[];
    closeList = [closeList,currentPoint];
    neighbourNum = getNeighbour(currentPoint,n);
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
    for ii = 1:length(neighbourNum)
        if neighbourNum(ii)==point(goalNum).num
            point(neighbourNum(ii)).father = currentPoint.num;
            point(goalNum).father = currentPoint.num;
            Ppoints = routPlot(goalNum,n);
            Pathpoints= Ppoints + limsMin;
            return;
        end
            log1=0;
            try
                tmp=point(neighbourNum(ii)).Gcost;
                if tmp ==inf
                    log1 = 1;
                end
            catch
                log1=0;
            end
            if log1 || ismember(neighbourNum(ii),closeListNum)
                continue;
            elseif (ismember(neighbourNum(ii),openListNum))
            oldGcost = getGcost(point(neighbourNum(ii)),n);
            father = point(neighbourNum(ii)).father;
            point(neighbourNum(ii)).father = currentPoint.num;
            newGcost = getGcost(point(neighbourNum(ii)),n);
            if newGcost>oldGcost
                point(neighbourNum(ii)).father = father;
            else
                point(neighbourNum(ii)).Gcost = newGcost;
            end
            continue;
        elseif ~ismember(neighbourNum(ii),closeListNum)
            point(neighbourNum(ii)).father = currentPoint.num;
            point(neighbourNum(ii)).Gcost = getGcost(point(neighbourNum(ii)),n);
            point(neighbourNum(ii)).Hcost = getHcost(point(neighbourNum(ii)),point(goalNum),n);
            openList = [openList,point(neighbourNum(ii))];
            end
    end
    closeListNum = cat(1,closeList.num);
    openListNum = cat(1,openList.num);
end
end


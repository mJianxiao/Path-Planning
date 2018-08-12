function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
robot = BotSim(modifiedMap);

%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(6); %spawn the particles in random locations
end

%% Localisation code
maxNumOfIterations = 40;
n = 0;
converged =0; %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    %% Write code for updating your particles scans
    for i = 1:num
        particleScan{i} = particles(i).ultraScan();
    end
    
    %% Write code for scoring your particles   
    maxscore=0;
    for i = 1:num
        a = particleScan{i}./botScan;
        b(1) = normpdf(a(1),1,0.4);
%         b(2) = normpdf(a(2),1,0.4);
        b(2) = normpdf(a(3),1,0.4);
%         b(4) = normpdf(a(4),1,0.4);
        b(3) = normpdf(a(5),1,0.4);
%         b(3) = normpdf(a(6),1,0.4);
        score(i) = prod(b);
        if score(i)>maxscore
            maxscore = score(i)
        end
    end
    
    sum=0;
    for i = 1:num
        sum = sum+score(i);
    end
    
    for i = 1:num
        norm(i) = score(i)/sum;
    end
    
    %% Write code for resampling your particles
    %p = ();
    p(1) = 0;
    for i = 1:num
        p(i+1) = norm(i)+p(i);
    end
    
    particles2(num,1) = BotSim;
    for i = 1:num
        x = rand;
        m = 1;
        while(x>p(m))
            m = m+1; 
        end
%         pos{i}=particles(m-1).getBotPos();
%         particles2(i).setBotPos(pos{i});
        particles2(i) = particles(m-1);
    end
    
    for i = 1:num
        position{i}=particles2(i).getBotPos();
        angle(i)=particles2(i).getBotAng();
        particles(i).setBotPos(position{i});
        particles(i).setBotAng(angle(i)+0.6*rand-0.3);
    end
    
     
    %% Write code to check for convergence   
    
    ParticlesPos = zeros(num,2);
%     ParticlesAng = zeros();
    for i = 1:num
        ParticlesPos(i,:) = particles(i).getBotPos();
        %ParticlesAng(i) = particles(i).getBotAng();
    end
    dev = std(ParticlesPos,1);
    if (dev(1,1)<0.8 && dev(1,2)<0.8 && n>12)
        converged = 1;
    else
        converged = 0;
    end
    
    if converged == 1 || n==maxNumOfIterations
        x = mean(ParticlesPos(:,1));
        y = mean(ParticlesPos(:,2));
        for i = 1:num
            particles(i).setBotPos([x y]);
        end
    end

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    if n<12
    num2 = num/4;
    for i = 1:num2
        j = 4*i;
        particles(j).randomPose(6);
    end
    end
    
    if n>12 && maxscore<0.9 && n<32  && converged ==0
        num3 = num-30;
        for i = 1:num3
            j = i;
            particles(j).randomPose(6);
        end
    end
        
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
%     if converged==0
%     turn = 0.5;
%     move = 2;
%     botSim.turn(turn); %turn the real robot.  
%     botSim.move(move); %move the real robot. These movements are recorded for marking 
%     for i =1:num %for all the particles. 
%         particles(i).turn(turn); %turn the particle in the same way as the real robot
%         particles(i).move(move); %move the particle in the same way as the real robot
%     end
%     end
    
    if converged==0 && n<maxNumOfIterations-1
        botdist = botSim.ultraScan();
        if botdist(1,1)<10 || botdist(2,1)<10 || botdist(6,1)<10
            turn = pi/3*(find(botdist==max(botdist))-1);
            move = max(botdist(1,1))*0.5;
        else
            turn = 0.5;
            move = 2;
        end
        botSim.turn(turn); %turn the real robot.  
        botSim.move(move); %move the real robot. These movements are recorded for marking 
        for i =1:num %for all the particles. 
            particles(i).turn(turn); %turn the particle in the same way as the real robot
            particles(i).move(move); %move the particle in the same way as the real robot
        end
    end
    
%     if converged==1
%         ParticlesAng=angle;
%     end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end

tx = round(target(1));
ty = round(target(2));
target = [tx ty];
ParticlesPos = zeros(num,2);
ParticlesAng = zeros();
    for i = 1:num
        ParticlesPos(i,:) = particles(i).getBotPos();
        ParticlesAng(i) = particles(i).getBotAng();
    end
cx = round(mean(ParticlesPos(:,1)));
cy = round(mean(ParticlesPos(:,2)));
cang = mean(ParticlesAng);
currPose=[cx cy];
robot.setBotPos([cx cy]);
robot.setBotAng(cang);
botSim.turn(-cang);
% robot.turn(-cang);
robot.drawBot(30,'r');
botSim.drawBot(30,'g');

path = astar(map,currPose,target);
step = length(path)-1;

for i=1:step
    sX = path(i+1,1)-path(i,1);
    sY = path(i+1,2)-path(i,2);
    sang(i) = atan2(sY,sX);
    dist(i)= distance([path(i,1) path(i,2)],[path(i+1,1) path(i+1,2)]);
end

for i=1:step
    botSim.turn(sang(i));
    botSim.move(dist(i));
    botSim.turn(-sang(i));
%     botSim.drawBot(1,'g');
end

% plot(tx,ty,'o','LineWidth',2);
end



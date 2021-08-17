%% Rosbag read 
data_dir = {'/media/jbs/ssd/experiment_dual_outdoor/bag/forest/success1.bag'};
topic = '/dual_chaser/wrapper/status';

for n = 1:length(data_dir)
    bag = rosbag(data_dir{n});
    bSel = select(bag,'Topic',topic);
    msgStruct = readMessages(bSel,'DataFormat','struct');
    msgStructSet{n} = msgStruct;
end

%% data logging 
nBag = length (data_dir);

panelXY = [1 2 5  6 9 10];
panel1 = [1 2 3 ];
panel2 = [4 5 6];
panel3 = [7 8 9];
tMax = 150; 
targetColor = [ 242, 31, 63 ;31, 190, 242];

for m = 1:nBag
    N = length (msgStructSet{m});
    msgStruct = msgStructSet{m};
    t = [];
    historyPlanningLocation = [];
    historyPlanningSpeed = [];
    historyPlanningAcceleration = [];
    historyPlanningEDF = [];
    historyPlanningBearing = []; 
    
    for i=1:2
        historyTargetLocation{i} = [];
        historyRelativeDistance{i} = [];
        historyBearingEDF{i} = [];
        historyBearingInterOcclusion{i} = [];   % should be recompted  
        historyInterOcclusion{i} = []; 
    end
    
    % ellispoid shape 
    rx = 0.3; 
    ry = 0.3;
    rz = 0.8;
    
    
  
    for n = 1:3:N
       data = msgStruct{n};
       
       
       % chaser 
       historyPlanningLocation = [historyPlanningLocation [data.PlanState.Position.X ; ...
           data.PlanState.Position.Y ; data.PlanState.Position.Z ]];
       historyPlanningSpeed = [historyPlanningSpeed data.PlanState.VelocityMag.Data];
       historyPlanningAcceleration = [historyPlanningAcceleration ...
           data.PlanState.AccelerationMag.Data];
       historyPlanningEDF = [historyPlanningEDF data.ObstacleCollision.Data];
       historyPlanningBearing = [historyPlanningBearing data.BearingAngle.Data];
       
       t = [t (double(data.Header.Stamp.Sec) + (1e-9) * double (data.Header.Stamp.Nsec)) ];
       
      
       % target related objectives 
       for i = 1:2
           chaserLocation = [data.PlanState.Position.X ; data.PlanState.Position.Y ; data.PlanState.Position.Z ];
           targetLocation = [data.TargetStates(i).Position.X ; data.TargetStates(i).Position.Y ; data.TargetStates(i).Position.Z];
           if i == 1
               otherTargetLocationIdx = 2;
           else
               otherTargetLocationIdx = 1;
           end
           otherTargetLocation = [data.TargetStates(otherTargetLocationIdx).Position.X ;...
               data.TargetStates(otherTargetLocationIdx).Position.Y ; data.TargetStates(otherTargetLocationIdx).Position.Z];
           
           
           historyTargetLocation{i} = [ historyTargetLocation{i} [data.TargetStates(i).Position.X ; 
               data.TargetStates(i).Position.Y ; data.TargetStates(i).Position.Z] ];   
           historyRelativeDistance{i} = [ historyRelativeDistance{i} data.RelativeDistances(i).Data];
           historyBearingEDF{i} = [historyBearingEDF{i} data.ObstacleOcclusions(i).Data ];
           
           % distance between ellipsoid between LOS
           chaserLocation = chaserLocation - otherTargetLocation;
           targetLocation = targetLocation - otherTargetLocation;
           nPnt = 10; 
           LOSpnts = [linspace(chaserLocation(1),targetLocation(1),nPnt) ; ...
               linspace(chaserLocation(2),targetLocation(2),nPnt) ; ...
               linspace(chaserLocation(3),targetLocation(3),nPnt)]';
           distSet = [];
           for mm = 1:nPnt
               [Xo,distToEllipsoid] = shortest_distance(LOSpnts(mm,:),[rx,ry,rz]);
               distSet = [distSet -distToEllipsoid];
           end
           historyInterOcclusion{i} = [ historyInterOcclusion{i} min(distSet)];          
       end      
    end
    t = t-t(1);
    
    
    figure(m)
    fontSize = 12; 
    clf
    
%     
%     subplot (3,3,panelXY)
%     hold on
%     plot(historyTargetLocation{1}(1,:),historyTargetLocation{1}(2,:),'r-')
%     plot(historyTargetLocation{2}(1,:),historyTargetLocation{2}(2,:),'b-')
%     plot(historyPlanningLocation(1,:),historyPlanningLocation(2,:),'g-')
%     axis equal 
    
    
    subplot(3,3,panel1);
    hold on
    smoothing = 10;
    lineWidth = 1; 
    hspeed = plot (t,smooth(historyPlanningSpeed,smoothing),'k:','LineWidth',1.5*lineWidth);
    haccel = plot (t,smooth(historyPlanningAcceleration,2*smoothing),'k-','LineWidth',lineWidth);
    set(gca,'XLim',[0 tMax])
    set(gca,'FontSize',fontSize)
    grid on
    legend([hspeed,haccel],{'$|\dot{\mathbf{x}}_p (t) |$ [m/s]','$|\ddot{\mathbf{x}}_p (t)|\; [\mathrm{m/{s}^2}]$'},'Interpreter','latex')

    
    h = subplot (3,3,panel3);
    smoothing = 50; 
    hold on
    hEDF = plot (t,smooth(historyPlanningEDF,smoothing),'k-');
    occlusionMargin = 0.3; 
    collisionMargin = 0.6; 
    hOccObstacle = [];
    hOccInter = [];
    xlabel('$t$ [s]','Interpreter','latex')

       
    for m = 1:2
        yline(occlusionMargin,'k--','LineWidth',2.0)
        hOccObstacle = [hOccObstacle plot(t,smooth(historyBearingEDF{m},smoothing),'Color',targetColor(m,:)/255,'LineWidth',lineWidth)];
%         hOccInter = [hOccInter plot(t,historyInterOcclusion{m},':','Color',targetColor(m,:)/255,'LineWidth',lineWidth)];
    end
    
    yline(collisionMargin,'k:','LineWidth',2.0)    
    set(gca,'YLim',[0.0 3.3])
    set(gca,'XLim',[0 tMax])
    set(gca,'FontSize',fontSize)
    grid on
%     legend([hOccObstacle hOccInter],{'$\psi_a (\mathbf{x}_p (t)) \; [\mathrm{m}]$','$\psi_b (\mathbf{x}_p (t)) \; [\mathrm{m}]$',...
%         '$e_a (\mathbf{x}_p (t) ; \mathbf{x}_q^{b} (t) ) \; [\mathrm{m}]$','$e_b (\mathbf{x}_p (t) ; \mathbf{x}_q^{a} (t) ) \; [\mathrm{m}]$'},...
%         'Interpreter','latex')
%     
    legend([hOccObstacle hEDF],{'$\psi_a (\mathbf{x}_p (t)) \; [\mathrm{m}]$','$\psi_b (\mathbf{x}_p (t)) \; [\mathrm{m}]$',...,
        '$\phi (\mathbf{x}_p (t))\; [\mathrm{m}]$'},...
        'Interpreter','latex')    
    
    subplot (3,3,panel2)
    hold on
    smoothing = 20;
    maxBearing = 1.25;
    desiredRelativeDistance = 3.5;
    hBearing = plot (t,historyPlanningBearing,'g-','LineWidth',1.2);
    yline(maxBearing,'k--','LineWidth',2.0)
    yline(desiredRelativeDistance,'k:','LineWidth',2.0)
    hRelDist = [];
    for m = 1:2
        hRelDist = [hRelDist plot(t,smooth(historyRelativeDistance{m},smoothing),'Color',targetColor(m,:)/255,'LineWidth',lineWidth)];
    end    
    set(gca,'XLim',[0 tMax])
    set(gca,'FontSize',fontSize)
    legend([hRelDist hBearing],{'$|\mathbf{x}_p(t) - \mathbf{x}_q^{a}(t)| \; [\mathrm{m}]$',...
        '$|\mathbf{x}_p(t) - \mathbf{x}_q^{b}(t)| \; [\mathrm{m}]$', '$\theta_{ab}(\mathbf{x}_p(t)) \; [\mathrm{rad}]$'},'Interpreter','latex')
    grid on
    
end











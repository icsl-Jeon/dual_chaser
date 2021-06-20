%% Rosbag read 
data_dir = {'forest_low_vis.bag'};
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
panel1 = [3 4];
panel2 = [7 8];
panel3 = [11 12];

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
    end
    
    
    for n = 1:2:N
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
       
       
       
       % target 
       for i = 1:2
           historyTargetLocation{i} = [ historyTargetLocation{i} [data.TargetStates(i).Position.X ; 
               data.TargetStates(i).Position.Y ; data.TargetStates(i).Position.Z] ];   
           historyRelativeDistance{i} = [ historyRelativeDistance{i} data.RelativeDistances(i).Data];
           historyBearingEDF{i} = [historyBearingEDF{i} data.ObstacleOcclusions(i).Data ];
          
       end
       
      
    end
    t = t-t(1);
    
    
    figure(m)
    clf
    subplot (3,4,panelXY)
    hold on
    plot(historyTargetLocation{1}(1,:),historyTargetLocation{1}(2,:),'r-')
    plot(historyTargetLocation{2}(1,:),historyTargetLocation{2}(2,:),'b-')
    
    plot(historyPlanningLocation(1,:),historyPlanningLocation(2,:),'g-')

    axis equal 
    
    
    subplot(3,4,panel1)
    hold on
    hspeed = plot (t,historyPlanningSpeed,'r-');
    haccel = plot (t,historyPlanningAcceleration,'k-');

    
    subplot (3,4,panel2)
    hold on
    plot (t,historyPlanningEDF,'k-')
    plot (t,historyBearingEDF{1},'r-')
    plot (t,historyBearingEDF{2},'b-')
    
    subplot (3,4,panel3)
    hold on
    maxBearing = 1.25;
    yline(maxBearing)
    plot (t,historyPlanningBearing,'g-')
    plot (t,historyRelativeDistance{1},'r-')
    plot (t,historyRelativeDistance{2},'b-')
   
end











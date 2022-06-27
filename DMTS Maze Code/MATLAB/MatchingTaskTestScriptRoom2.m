
% 1 refers to beads, 2 refers to yarn for the blocks

blocks=[2 1 2 1 2 1 2 1 2 1 2 1 2 1 2 1 2 1 2 1 2];

blocksize=1; % number of trials before switching rule
holdtime=2; % how long does he have to sit on top of the port
delaytime=8; % treadmill delay time

tasktimer=AutoMatchToSampleTask(blocks,blocksize,holdtime,delaytime);


%% for RANDOM

% 1 refers to beads, 2 refers to yarn for the blocks
blocks=randi([1,2], 1,100); % last number in this is the max trials
while any(diff(blocks,3)==0) % number after blocks is max in a row
    repeat=find(diff(blocks,3)==0,1,'first')+1; 
    blocks(repeat+1:end)=randi(2,length(blocks)-repeat,1);
end
blocksize=1; % number of trials before switching rule
holdtime=2; % how long does he have to sit ontop of the port
delaytime=8; % treadmill delay time

tasktimer=AutoMatchToSampleTask(blocks,blocksize,holdtime,delaytime);

%COM9 for treadmill

% this is the strobed version - automatchtosampletask

%% reset
if ~isempty(instrfind)
     fclose(instrfind);
      delete(instrfind);
end

%% make arduino
a=arduino();


%% make motorrs

motor1=servo(a,'D2');
motor2=servo(a,'D3');
motor3=servo(a,'D4'); % treadmill door
motor4=servo(a,'D5'); % study spinner
motor5=servo(a,'D7'); %3rd door

%% test motors

% test spinner
writePosition(motor1,0.35); pause(1); writePosition(motor1,1);
pause(1); writePosition(motor1,0.35); pause(1); 

% front door
writePosition(motor2,0); pause(1); writePosition(motor2,.45);
pause(1); writePosition(motor2,0); pause(1); writePosition(motor2,0); pause(1); writePosition(motor2,.45);
pause(1); writePosition(motor2,0); pause(1);

% back door
writePosition(motor3,0); pause(1); writePosition(motor3,.35);
pause(1); writePosition(motor3,0); pause(1); writePosition(motor3,0); pause(1); writePosition(motor3,.35);
pause(1); writePosition(motor3,0); pause(1);

% third door
writePosition(motor5,0.80); %open
writePosition(motor5,1) %closed%

% study spinner
writePosition(motor4,0); pause(1); writePosition(motor4,1);
pause(1); writePosition(motor4,0); pause(1);


writePosition(motor4,0) ;writePosition(motor3,0); writePosition(motor2,0);



%% test IR beams

configurePin(a,'D32','Pullup');
configurePin(a,'D33','Pullup');
configurePin(a,'D27','Pullup');
configurePin(a,'D25','Pullup');
configurePin(a,'D26','Pullup');
configurePin(a,'D23','Pullup');
writeDigitalPin(a,'D12',1);
for i=1:1000
    beam1=readDigitalPin(a,'D32');
    plot(i,beam1,'b*');
    beam2=readDigitalPin(a,'D33');
    plot(i,beam2+.1,'r*');
    beam3=readDigitalPin(a,'D27');
    plot(i,beam3+.2,'g*');
    beam4=readDigitalPin(a,'D25');
    plot(i,beam4+.3,'m*');
    beam5=readDigitalPin(a,'D26');
    plot(i,beam5+.4,'c*');
    beam6=readDigitalPin(a,'D23');
    plot(i,beam6+.5,'k*');
    hold on; drawnow; 
    
end
writeDigitalPin(a,'D12',0)

%% test solenoids

% study beads 
writeDigitalPin(a,'D50',0); pause(.3);
writeDigitalPin(a,'D50',1); 

% study yarn
writeDigitalPin(a,'D51',0); pause(.6);
writeDigitalPin(a,'D51',1);

% test yarn
writeDigitalPin(a,'D52',0); pause(.2);
writeDigitalPin(a,'D52',1); 

% test beads
writeDigitalPin(a,'D53',0); pause(.3);
writeDigitalPin(a,'D53',1); 

%%

backprob=.5;
blocksize=10; blocks=[];
 % change    1:X here for more blocks of trials
for blocks=1:4
    temptrials=(rand(1,blocksize)<backprob)+1+mod(blocks,2)*2;
    blocks=[blocks;temptrials'];
end
fprintf('Back Percentage %2.2f \n', nanmean(mod(blocks,2))*100);


% PotMatchToSample('trial list', 'nose hold time','treadmill walk time'                                     
[TaskTimer,winpct]=PotMatchToSampleTask(blocks,0.7,0);

%% 
stop(TaskTimer);
clear TaskTimer

%%
okpins={'D14','D15','D16','D17','D18','D19','D20','D21'}; %,'D37','D38','D39','D40','D41','D42','D43','D44','D45'};
% 3:16
for i=1:length(okpins)
    writeDigitalPin(a,okpins{i},1); writeDigitalPin(a,okpins{i},0)
    pause(.01);
end

%% test strobed events
a=arduino;
thispin='D38';
writeDigitalPin(a,thispin,1); writeDigitalPin(a,thispin,0);



%% mappins
mappins.sbeads='D15'; % event3
mappins.syarn='D16';  % event4
mappins.tbeads='D17'; % event5
mappins.tyarn='D18'; % event6

mappins.tmstart='D19'; % event7
mappins.tmstop='D20'; % event8

mappins.door1='D39'; % event10 empty
mappins.door3='D41'; % event14 empty
mappins.givefood='D40'; % event 15
mappins.wrong='D38'; % event16

%%
% random number generator with no repeats
blocks=randi(5, 1,100); % last number in this is the max trials
while any(diff(blocks)==0) % number after blocks is max in a row
    repeat=find(diff(blocks)==0,1,'first')+1;
    blocks(repeat)=randi(5);
end

% now multiply by 5 and add 20
blocks=blocks*5+20;
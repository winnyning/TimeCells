mydaq=daq.createSession('NI');

addDigitalChannel(mydaq,'dev1','port0/line1:7','OutputOnly')
addDigitalChannel(a,'dev1','port0/line0','InputOnly')

outputSingleScan(mydaq,[1 1 1 1 1 1 1]);

outputSingleScan(mydaq,[0 0 1 1 1 1 1]);

test=inputSingleScan(mydaq);
%%

% find servos


%%
a=arduino;


% one microsecond
%    0.0001
% this is for hte micro servos
s=servo(a,'D2','MinPulseDuration',0.0009,'MaxPulseDuration',0.0021);

% lets see if this is correct
%% add motor library, servo, and rotartEncoder
a=arduino('COM4','Mega2560','Libraries',{'Adafruit/MotorShieldV2', 'I2C', 'SPI', 'Servo','rotaryEncoder'});
% pulses per rev=cycles per rev=lines per rev=counts per rev/4
% this is a quadrature encoder, so it will be 1 pulse per cycle

% need 2 so you can do direction
encoder=rotaryEncoder(a,'D2','D3',100);



% max is 3 encoders per arduino
%%
for i=1:1000
    yyaxis left;
    speedy(i)=readSpeed(encoder);
    plot(i,speedy(i),'b*'); hold on; drawnow;
    yyaxis right;
    [distancy(i),times(i)]=readCount(encoder);
    plot(i,distancy(i),'r*'); hold on; drawnow;
    if abs(speedy(i))>25
        writeDigitalPin(a,'D13',1);
    else
        writeDigitalPin(a,'D13',0);
    end
    %pause(.2);
end

% for these, if you put 100

%%
multiplyer=((distancy(38)-distancy(37))/times(38)-times(37))/speedy(37)
%% super heavy duty servo
a=arduino;
s2=servo(a,'D2','MinPulseDuration',0.0005,'MaxPulseDuration',0.00025);

%% running a stepper using the motor shield, wayyyy better btw

a=arduino();
shield=addon(a,'Adafruit/MotorShieldV2');
% shield name, motor number is 1, 200 steps per revolution
% rpm is important and it can go real fast, and steptype is always double
sm=stepper(shield,1,200,'RPM',50,'StepType','Double');

% this moves it 50 steps
move(sm,50);
%%
a=arduino;
readpin='D21';
writepin='D24';
configurePin(a,readpin,'Pullup');

for i=1:1000
    sensorstat(i)=readDigitalPin(a,readpin);
    plot(i,sensorstat(i),'r*');
    hold on; drawnow;
    if sensorstat(i)<1
        writeDigitalPin(a,writepin,1);
    else
        writeDigitalPin(a,writepin,1);
    end
end

%% lets see if we can build a camera listener



imaqhwinfo()

vidobj=imaq.VideoDevice('winvideo',1);
for i=1:20
    
    
    
end

myobj=videoinput('winvideo',1);
myobj.SelectedSourceName='input1';
preview(myobj); myframe=[];
for i=1:20
    
    frame=getsnapshot(myobj);
    myframe(:,:,i)=frame(:,:,1);
end
%%
a=arduino;

%%

PinMap={'D2','D3','D4','D5','D6'};
ColorMap={'r*','g*','b*','c*','k*','o*'};
for j=1:5, configurePin(a,PinMap{j},'pullup'); end
figure;
for i=1:10000
    for j=1:5
        plot((i),readDigitalPin(a,PinMap{j})+.05*j,ColorMap{j}); hold on;
        drawnow;
    end
end

%%
% communicating through serial
temp=seriallist;

mymaze = serial(temp,'BaudRate',9600);
fopen(mymaze);
fprintf(mymaze,'04');


temp=fscanf(mymaze);


for i=1:1000
    if mymaze.BytesAvailable>0
        fscanf(mymaze)
        fprintf('\n');
    end
    pause(2);
end
%% for babies, run this first
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%% run with firmware


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
temp=seriallist;
temp'

mymaze = serial(temp{2},'BaudRate',9600);


% % generate trial list
% triallist{1}='1';
% for i=2:50
%     triallist{i}=num2str(randi(4));
% while triallist{i}==triallist{i-1}
% triallist{i}=num2str(randi(4));
% end
% end
%            Beads | Straws   
%           _____________
%     front |  1   |  2
%           ______________
%     back  |  3   |  4
 

triallist=[1 1];
for i=3:50
    triallist(i)=randi(2);
    while triallist(i)==triallist(i-1) && triallist(i)==triallist(i-2)
        triallist(i)=randi(2);
    end
end
triallist=repmat([1 2],1,30);
trialLUT={'1', '2'; '3', '4'};
trialpos=[1 2];

%% for blocks; for babies, run this second
 if strcmpi(mymaze.status,'open')
    fclose(mymaze);
 end
 
 pause(1);
 fopen(mymaze);
 pause(5);

trialperf=[]; runningtally=[];
for i=1:length(triallist)
    
   numcorrectinarow=5; %% change this for # correct in a row!
   trialperf=zeros(1,numcorrectinarow);
   %the 3 is for number of trials correct
   while sum(trialperf(end-(numcorrectinarow-1):end))<numcorrectinarow
       trialpos=[trialpos randi(2)];
       if trialpos(end)==trialpos(end-1) && trialpos(end)==trialpos(end-2)
           trialpos(end)=(~logical(trialpos(end-1)-1))+1;
       end
       mytrialtype=trialLUT(triallist(i),trialpos(end));
       
        fwrite(mymaze,mytrialtype{1});
        pause(1);
        trialtype=str2num(char(fread(mymaze,1)));
        
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        fprintf('   rat sampled\n');
        pause(.1);
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        fprintf('   rat got on treadmill\n');
        pause(.1);
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        if received==3
            fprintf('Correct! Overall: %.2f \n',mean(runningtally));
            trialperf=[trialperf 1];
            runningtally=[runningtally 1];
        else
            fprintf('Incorrect! Overall: %.2f \n', mean(runningtally));
            trialperf=[trialperf 0];
            runningtally=[runningtally 0];
        end
        
        pause(.2);
    end
    fprintf('Moving on to another trial type ran total of %d \n',length(runningtally));
    fprintf('Got %d wrong this trial \n', sum(trialperf(numcorrectinarow+1:end)==0));
    pause(.1);
end

%% for treadmill run this first

tmill=Treadmill('COM9'); %dont use 4, use 6
tmill.SetSpeed=30;
tmilldelay=8;

mymaze = serial('COM3','BaudRate',9600);

% % for diff treadmill speeds, uncomment to run normal 8
trialspeed=[25,40,45,30,45,25,35,30,35,25,35,30,40,25,35,40,25,45,35,45,...
    35,25,35,45,40,35,40,25,35,40,45,30,35,30,45,35,30,35,45,25,40,45,30,...
    35,25,45,25,35,40,30,40,25,35,40,30,45,25,35,30,35,30,35,45,30,35,30,...
    40,45,25,45,40,25,35,30,35,45,30,25,30,35,30,45,40,30,25,40,30,40,25,...
    45,25,30,25,30,40,25,45,30,35,45];

% thi is for random, with 3 trials in a row max
triallist=[1 2 2];
for i=4:100
    triallist(i)=randi(2);
    while triallist(i)==triallist(i-1) && triallist(i)==triallist(i-2) && triallist(i)==triallist(i-3)
        triallist(i)=randi(2);
    end
end


% this is for alternation
%triallist=repmat([1 2],1,30);

trialLUT={'1', '2'; '3', '4'};
trialpos=[1 2];



if strcmpi(mymaze.status,'open')
    fclose(mymaze);
 end
 
 pause(1);
 fopen(mymaze);
 pause(5);
 
%% for completely random with treadmill 
%run this second; 
%look at runningtally for # of trials
trialperf=[]; runningtally=[];
for i=1:length(triallist)
    
  
       % generate a trial position, and the position can only be the same
       % for 2 trials in a row
       trialpos=[trialpos randi(2)];
       if trialpos(end)==trialpos(end-1) && trialpos(end)==trialpos(end-2)
           trialpos(end)=(~logical(trialpos(end-1)-1))+1;
       end
       mytrialtype=trialLUT(triallist(i),trialpos(end));
       
        fwrite(mymaze,mytrialtype{1});
        pause(1);
        trialtype=str2num(char(fread(mymaze,1)));
        
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        fprintf('   rat sampled\n');
        pause(.1);
        while mymaze.BytesAvailable==0
        end
        
%         tmill.SetSpeed=30;
%         fprintf('   rat is runnin %d cm/s \n',tmill.SetSpeed);
        tmill.SetSpeed=trialspeed(i);
        fprintf('   rat is runnin %d cm/s \n',trialspeed(i));
        received= str2num(char(fread(mymaze,1)));
        fprintf('   rat got on treadmill\n');
        tmill.start;
        pause(tmilldelay);
        tmill.stop;
        fwrite(mymaze,'1');
        
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        if received==3
             trialperf=[trialperf 1];
            runningtally=[runningtally 1];
            fprintf('Correct! Overall: %.2f \n',mean(runningtally));
        else
            trialperf=[trialperf 0];
            runningtally=[runningtally 0];
            fprintf('Incorrect! Overall: %.2f \n', mean(runningtally));
        end
        fprintf('Total number of trials %d \n', length(runningtally));
        pause(.2);
    end
    fprintf('Moving on to another trial type ran total of %d \n',length(runningtally));
    
    %fprintf('Got %d wrong this trial \n', sum(trialperf(numcorrectinarow+1:end)==0));
    pause(.1);
%end


%% dont fuck w this!!!!!!!!!!!!!!!! for completely random diff positions
% generate trial list

temp=seriallist;
temp'
%tmill=Treadmill('COM4')
mymaze = serial(temp{1},'BaudRate',9600);

triallist{1}='1';
for i=2:80
    triallist{i}=num2str(randi(4));
    while triallist{i}==triallist{i-1}
        triallist{i}=num2str(randi(4));
    end
end



%%
% for fully random
 pause(1);
 fopen(mymaze);
 pause(5);

trialperf=[];
for i=1:length(triallist)
    
   trialperf=[0];
   %the 2 is for number of trials correct
   while sum(trialperf(end-0:end))<1
        fwrite(mymaze,triallist{i});
        pause(1);
        trialtype=str2num(char(fread(mymaze,1)));
        
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        fprintf('   rat sampled\n');
        pause(.1);
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        fprintf('   rat got on treadmill\n');
        pause(.1);
        while mymaze.BytesAvailable==0
        end
        received= str2num(char(fread(mymaze,1)));
        if received==3
            fprintf('Correct!\n');
            trialperf=[trialperf 1];
        else
            fprintf('Incorrect!\n');
            trialperf=[trialperf 0];
        end
        
        pause(.2);
    end
    fprintf('Moving on to another trial type \n');
    
    pause(.1);
end








%%
configurePin(a,'A2','AnalogInput');
figure;
for i=1:1000
    plot(i,log(4-readVoltage(a,'A2')),'*');
    hold on;
    drawnow;
    pause(.1);
end

%% An up and down servo, where its only two circuits:
% 1. the servo
% 2. two interrupts, where at each end the pullup goes to gnd

% initialize
a=arduino;
s1=servo(a,'D2');
configurePin(a,'D3','Pullup');

% the intial code
% go up
writePosition(s1,.45);
pause(.2);
while readDigitalPin(a,'D3')>0
    
end
writePosition(s1,.5);

% go down
writePosition(s1,.6);
pause(.2);
while readDigitalPin(a,'D3')>0
    
end
writePosition(s1,.5);
%%
% the linear servo
s1=servo(a,'D2');
writePosition(s1,1); % slow the servo
writeDigitalPin(a,'D3',1); writeDigitalPin(a,'D3',0);
writeDigitalPin(a,'D4',1); writeDigitalPin(a,'D4',0);


%% Writing digital pins to the map box

Mappins={'D40','D41','D42','D43','D44','D45','D46','D47','D48','D49'};

for i=1:length(Mappins)
    configurePin(a,Mappins{i},'DigitalOutput');
end

for i=1:length(Mappins)
    writeDigitalPin(a,Mappins{i},1); writeDigitalPin(a,Mappins{i},0);
    pause(.5);
end


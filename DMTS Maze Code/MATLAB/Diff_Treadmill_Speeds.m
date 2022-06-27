%% for DIFFERENT treadmill speeds for each trial
% RUN THIS PART FIRST

tmill=Treadmill('COM5'); %dont use 4, use 6
tmill.SetSpeed=30;
tmilldelay=8;

mymaze = serial('COM4','BaudRate',9600);

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
 % RUN THIS PART SECOND!

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

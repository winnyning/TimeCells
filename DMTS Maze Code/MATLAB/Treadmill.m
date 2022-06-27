classdef Treadmill < handle
% TREADMILL Control a treadmill attached to CI-Bus
%
%   Description
%      This class can be used to control a treadmill manufactured by
%      Columbus Instruments <http://www.colinst.com/> and attached to the
%      computer via a serial port (using the CI-Bus).
%
%   Syntax:
%      T = Treadmill
%      T = Treadmill(OPTIONS)
%   
%   Description:
%      T = Treadmill constructs a Treadmill object with saved or
%      automatically detected settings.
%
%      T = Treadmill(OPTIONS) constructs a Treadmill object with the
%      settings specified by 'OPTIONS'. Legal OPTIONS include the name of
%      a serial port, a number between 1 and 255 indicating the device ID
%      of the treadmill, or the string 'debug' (which enables 'Debugging
%      Mode'). If the device ID or serial port name are omitted, the
%      correct device ID and serial port will either be loaded from saved
%      preferences, or an attempt will be made to automatically detect the
%      correct values.
%
%   Example:
%      % Construct a Treadmill object for a treadmill with device ID 220.
%      t = Treadmill(220);
%
%      % Construct a Treadmill object for a treadmill with device ID 220,
%      % using serial port COM2.
%      t = Treadmill('COM2');
%
%      % Construct a Treadmill object, attempting to automatically
%      % determine the correct serial port and device ID (or loading values
%      % stored from the last Treadmill object.
%      t = Treadmill();
%
%   First Usage:
%      % Initialize the treadmill. If arguments are omitted the system
%      % attempts to detect the correct serial port and device ID. If more
%      % than one serial port is available, a list of serial ports will be
%      % displayed. If more than one treadmill is detected, a list of
%      % treadmills with their device ID and verbose ID will be produced.
%      % Once a serial port and device ID have been detected, they will be
%      % saved and used automatically in the future, unless you
%      % specifically request a different serial port or device ID.
%      t = Treadmill():
%
%      % Set the preferred units to use for distance and time.
%      % This setting will become the new default for future use.
%      t.DistUnit = 'cm';
%      t.TimeUnit = 's';
%
%      % Check the current Treadmill state and properties.
%      disp(t);
%
%      % List all properties of Treadmill and their current value.
%      get(t);
%
%      % Change the acceleration if necessary.
%      % Valid options include the strings 'max', 'min', or 'inf', or a
%      % or a number in the units specified above.
%      % 'max' means accelerate at a regulated 25.5 m/min/s.
%      % 'min' means accelerate at a regulated ~0.0004 m/min/s.
%      % 'inf' means accelerate as fast as possible (unregulated), this
%      % will be faster than the 'max' acceleration.
%      t.Acceleration = 'inf';
%
%      % Reset the odometer if desired.
%      t.resetOdometer;
%
%      % Set the desired speed of the treadmill.
%      % Valid speeds include string 'max', which will set the speed equal
%      % to t.MaxSpeed, or a speed between 0 and t.MaxSpeed.
%      t.SetSpeed = 'max';
%
%   Typical Usage (after the first usage):
%      t = Treadmill();    % Initialize the treadmill.
%      t.resetOdometer;    % Reset the odometer (optional).
%      t.SetSpeed = 'max'; % Set the desired speed.
%
%   Manual start/stop:
%      t.start();          % Start the treadmill.
%      disp(t.CurrentSpeed)% Display the current treadmill speed.
%      for ii = 1:10;
%          pause(1);
%          disp(t.CurrentSpeed)
%      end
%      t.stop();           % Stop the treadmill.
%
%   Automatic (timed) stop:
%      t.resetOdometer;    % Reset the odometer (optional).
%      t.start(10);        % Start (at SetSpeed) and run for 10 seconds.
%      for ii = 1:15;
%          pause(1);
%          disp(t.CurrentSpeed);
%          disp(t.Odometer);
%      end
%      dist = t.Odometer;  % Read the odometer (optional).
%
%   Properties
%      Acceleration  %             Desored treadmill acceleration
%      BeltMode      % (read-only) Position of the 'Belt Mode' switch
%      CurrentSpeed  % (read-only) Current speed of treadmill
%      DID           % (read-only) Device ID of the treadmill
%      DistUnit      %             Distance unit
%      MaxSpeed      % (read-only) Maximum speed of the treadmill
%      Odometer      % (read-only) Current odometer reading
%      Running       % (read-only) Is the treadmill currently running?
%      SetSpeed      %             Desired treadmill speed
%      TimeUnit      %             Time unit
%      VerboseID     % (read-only) Verbose treadmill ID
%
%   Methods (see separate help pages for more details):
%      manualMode();       % Switch the treadmill to user control.
%      resetController();  % Reset connection to the controller box.
%      resetOdometer();    % Reset the odometer.
%      resetSerial(port);  % Reset the serial port connection ('port' optional)
%      start(duration);    % Start the treadmill ('duration' optional)
%      stop();             % Stop the treadmill
%
%   Static Methods (see separate help pages for more details):
%      [port, status, info] = checkSerialPort(port, info);
%      [port, info] = findSerialPort(info);
%      [did, port, verboseids] = listDevices(varargin);
%
% Author: Benjamin Kraus (bkraus@bu.edu, ben@benkraus.com)
% Copyright (c) 2013, Benjamin Kraus

% $Id: Treadmill.m 4774 2013-04-02 21:40:24Z bkraus $

properties(Transient, SetAccess=private)
    DID = uint8(0);             % Device ID of the treadmill
    GID = uint8(0);             % Group ID of the treadmill
    VerboseID = '';             % Verbose treadmill ID
    Product = '';               % Product Number
    Firmware = '';              % Firmware Version
    Running = 'off';            % Is the treadmill currently running?
    Log = NaN(0,3);             % Log of speed changes
end

properties(Transient, SetAccess=protected, Hidden)
    BaudRate = 19200;           % Baud rate for serial communications
    OdometerUpdateRate = 0;     % Odometer Update Rate
    OdometerDistPerUpdate = 0;  % Odometer Distance Traveled per Update Rate
    s;                          % Serial port object
    timer;                      % Timer object for timed starting
    Port = '';                  % Serial port name
end

properties(Transient, AbortSet, SetAccess=public)
    DistUnit = 'cm';            % Distance unit
    TimeUnit = 's';             % Time unit
    MinRefresh = 1;             % Minimum refresh period for treadmill state (seconds).
end

properties(Dependent)
    SetSpeed;                   % Desired treadmill speed
    Acceleration;               % Treadmill acceleration
    AccelerationStep;           % Treadmill acceleration step size
    AccelerationInterval;       % Treadmill acceleration interval (seconds)
end

properties(Transient, Hidden)
    DebugMode = false;          % In debug mode, all commands are echoed to terminal.
end

properties(Dependent, SetAccess=private)
    MaxSpeed;                   % Maximum speed of the treadmill
    CurrentSpeed;               % Current speed of treadmill
    Odometer;                   % Current odometer reading
    BeltMode;                   % Position of the 'Belt Mode' switch
    ShockRate;                  % Unimplemented
end

properties(Transient, Access=private, Hidden)
    MaxSpeed_ = 0;              % Maximum speed of the treadmill (m/min)
    AccelerationStep_ = 0;      % Treadmill acceleration step size (.1m/min)
    AccelerationInterval_ = 0;  % Treadmill acceleration interval (seconds)
    RefreshAccel = false;       % Does new acceleration data need to be sent to treadmill?
    LastUpdate = 0;             % Last time the treadmill data was updated
    SetSpeed_ = 0;              % Desired treadmill speed (m/min)
    CurrentSpeed_ = 0;          % Current speed of treadmill (m/min)
    Odometer_ = 0;              % Current odometer reading (meters)
    BeltMode_ = '';             % Position of the 'Belt Mode' switch
    ShockRate_ = NaN;           % Unimplemented
end

methods
    function obj = Treadmill(varargin)
    % Construct a Treadmill object
    %
    %   Syntax:
    %      T = Treadmill
    %      T = Treadmill(OPTIONS)
    %   
    %   Description:
    %      T = Treadmill constructs a Treadmill object with saved or
    %      automatically detected settings.
    %
    %      T = Treadmill(OPTIONS) constructs a Treadmill object with the
    %      settings specified by 'OPTIONS'. Legal OPTIONS include the name of
    %      a serial port, a number between 1 and 255 indicating the device ID
    %      of the treadmill, or the string 'debug' (which enables 'Debugging
    %      Mode'). If the device ID or serial port name are omitted, the
    %      correct device ID and serial port will either be loaded from saved
    %      preferences, or an attempt will be made to automatically detect the
    %      correct values.
    %
    %   Example:
    %      % Construct a Treadmill object for a treadmill with device ID 220.
    %      t = Treadmill(220);
    %
    %      % Construct a Treadmill object for a treadmill with device ID 220,
    %      % using serial port COM2.
    %      t = Treadmill('COM2');
    %
    %      % Construct a Treadmill object, attempting to automatically
    %      % determine the correct serial port and device ID (or loading values
    %      % stored from the last Treadmill object.
    %      t = Treadmill();
        
        % Get information about available serial ports.
        info = instrhwinfo('serial');
        port = '';
        
        % Process input arguments
        for a = 1:nargin
            % If it is a string, first check if it is 'debug'
            if(ischar(varargin{a}) && strcmpi(varargin{a},'debug'))
                obj.DebugMode = true;
                
            % Otherwise, check if it is a Serial port name.
            elseif(ischar(varargin{a}) && ~isempty(varargin{a}))
                tf = strcmpi(varargin{a}, info.SerialPorts);
                if(sum(tf)==1)
                    port = info.SerialPorts{tf};
                else
                    error('Treadmill:InvalidSerialPort',...
                        'Specified serial port (''%s'') is invalid.',...
                        varargin{a});
                end
                
            % If it is a number between 1 and 255, assume it is a DeviceID.
            elseif(isnumeric(varargin{a}) && isscalar(varargin{a}) && ...
                    varargin{a} > 0 && varargin{a} < 256)
                did = varargin{a};
                
            % Otherwise throw and error.
            else
                error('Treadmill:InvalidArgument',...
                    'Invalid argument (number %d)', a);
            end
        end
        
        % Load saved preferences.
        if(ispref('Treadmill')); prefs = getpref('Treadmill');
        else prefs = struct();
        end
        
        % Load saved units preferences.
        if(isfield(prefs,'DistUnit')); obj.DistUnit = prefs.DistUnit; end
        if(isfield(prefs,'TimeUnit')); obj.TimeUnit = prefs.TimeUnit; end
        
        % Open a connection to the serial port.
        port = obj.resetSerial(port);
        
        % If a DeviceID was not provided, first check saved preferences.
        % If there is no DID in preferences, search for any devices.
        if(exist('did','var')~=1)
            if(isfield(prefs,'DID'))
                did = prefs.DID;
            else
                [obj.DID, ~, vids] = Treadmill.listDevices(obj.s);
                assert(numel(obj.DID)>0, 'Treadmill:DeviceNotFound',...
                    'Failed to find a treadmill on ''%s''', port);
                if(numel(obj.DID)>1)
                    didstr = '';
                    for d = 1:numel(obj.DID)
                        didstr = sprintf('%s%03d: %s\n',didstr,obj.DID(d),vids{d});
                    end
                    error('Treadmill:MultipleDevicesFound',...
                        'Multiple device IDs found on ''%s'':\n%s', port, didstr);
                end
            end
        end
        
        % Create shortcuts to these functions for future use.
        sendcommand = @(command) obj.sendcommand(command);
        readresponse = @(varargin) obj.readresponse(varargin{:});
        
        % If a DeviceID was specified, make sure it is present on the bus.
        if(exist('did','var')==1)
            sendcommand([did 016]);     % Reset CI-Bus (command 016)
            java.lang.Thread.sleep(20); % Pause for 20 ms
            sendcommand([did 017]);     % Return Device ID (command 017)
            [didout, tf] = readresponse(10, NaN, true,...
                sprintf('return device ID (%03d on ''%s'')', did, port));
            if(~tf); didout = []; end
            obj.DID = did;
        end
        
        % Check that we have found a good and active Device ID.
        assert(numel(obj.DID)==1 && obj.DID > 0, 'Treadmill:DeviceNotFound',...
            'Failed to find a treadmill on ''%s''', port);
        
        % Save both the port and DID for future use.
        prefs.DID = obj.DID;
        prefs.Port = port;
        setpref('Treadmill',fieldnames(prefs),struct2cell(prefs));
        
        % If we don't already have it, get the Product and Firmware Numbers
        if(exist('didout','var')~=1 || isempty(didout))
            sendcommand([obj.DID 017]); % Return Device ID (command 017)
            [didout, tf] = readresponse(10, NaN, true,...
                sprintf('return device ID (%03d on ''%s'')', obj.DID, port));
            if(~tf); didout = []; end
        end
        
        % Record the product and firmware numbers.
        if(numel(didout)==10)
            obj.Product = char(didout(2:5))';
            obj.Firmware = [char(didout(6:7))' '.' char(didout(8:9))'];
        end
        
        % Get and save the Verbose device ID string.
        sendcommand([obj.DID 020]); % Return Verbose ID (command 020)
        out = readresponse(NaN, 500, true, 'return verbose ID');
        obj.VerboseID = char(out(2:end-1))';
        
        % Stop the treadmill (in case it is currently running).
        assert(obj.sendSpeed(0), 'Treadmill:CommuncationError',...
            'Failed to stop treadmill');
        
        % Read the EEPROM for the acceleration, max speed, and odometer
        % settings.
        sendcommand([obj.DID 023 016 023]); % Read EEPROM (command 023)
        out = readresponse(26, NaN, true, 'read EEPROM');
        
        % Response:
        %   1   2   3   4       19  20  21  22  23  24  25  26
        % [DID ADR 016 017 ... 032 033 034 035 036 037 038 CHK]
        
        obj.GID                   = out( 4); % Adresss 017
        obj.AccelerationStep_     = out(19); % Address 032
        obj.AccelerationInterval_ = out(20); % Address 033
        obj.OdometerUpdateRate    = out(21); % Address 034
        obj.MaxSpeed_             = (out(22)+256*out(23))/10; % Address 035:036 [L:H]
        obj.OdometerDistPerUpdate = out(24)+256*out(25);      % Address 037:038 [L:H]
        
        % Check the belt mode and warn if it is set to 'Stopped'.
        obj.warnBeltMode;
        
        % Clear the log.
        obj.clearLog();
    end
    
    function set.DistUnit(obj, unit)
        % Set the desired unit to use for distance.
        % For example, if distance unit is 'cm' and time unit is 's', then
        % the unit used for distance is 'cm', speed is 'cm/s' and
        % acceleration is 'cm/s^2'.

        % List of recognized units:
        units =  {'mm', 'cm', 'm', 'km', 'inches', 'feet', 'miles'};
        unitsstr = sprintf('%s ', units{:});
        
        % Check that the requested unit is valid and recognized.
        assert(ischar(unit),'Treadmill:InvalidDistUnit',...
            'Distance unit must be a string.');
        tf = strcmpi(unit, units);
        assert(sum(tf)==1,'Treadmill:InvalidDistUnit',...
            'Distance unit must be one of: %s.', unitsstr);
        
        % Store the requested unit.
        obj.DistUnit = units{tf};
        setpref('Treadmill','DistUnit',obj.DistUnit);
    end
    
    function set.TimeUnit(obj, unit)
        % Set the desired unit to use for time.
        % For example, if distance unit is 'm' and time unit is 'min', then
        % the unit used for distance is 'm', speed is 'm/min' and
        % acceleration is 'm/min/s'.
        
        % List of recognized units:
        units =  {'s', 'sec', 'min', 'h', 'hours'};
        unitsstr = sprintf('%s ', units{:});
        
        % Check that the requested unit is valid and recognized.
        assert(ischar(unit),'Treadmill:InvalidTimeUnit',...
            'Time unit must be a string.');
        tf = strcmpi(unit, units);
        assert(sum(tf)==1,'Treadmill:InvalidTimeUnit',...
            'Time unit must be one of: %s.', unitsstr);
        
        % Store the requested unit.
        obj.TimeUnit = units{tf};
        setpref('Treadmill','TimeUnit',obj.TimeUnit);
    end
    
    function set.SetSpeed(obj, speed)
        % Set the desired speed of the treadmill. If the treadmill is
        % stopped, this speed will be used the next time the treadmill is
        % started. If the treadmill is running, this change will take
        % effect immediately.
        %
        % Valid speeds include string 'max', which will set the speed equal
        % to t.MaxSpeed, or a speed between 0 and t.MaxSpeed.
        
        % Check if 'max' was specified.
        if(ischar(speed))
            if(strcmpi(speed,'max')); obj.SetSpeed_ = obj.MaxSpeed_;
            else error('Treadmill:InvalidSpeed',...
                    'Speed must be a numeric scalar or ''max''.')
            end
            
        % Otherwise assume a speed was specified.
        elseif(isnumeric(speed) && isscalar(speed))
            [factor, units] = conversionFactor(obj, 'speed');
            
            % Check that the speed is not negative.
            if(speed<0)
                warning('Treadmill:NegativeSpeed',...
                    'Speed must be non-negative, setting speed to 0 %s.\n', units);
                speed = 0;
            end
            
            % Convert speed into meters/minute.
            % Both 'SetSpeed_' and 'MaxSpeed_' are stored in meters/minute.
            obj.SetSpeed_ = round(10*speed/factor)/10;
            
            % Check that the speed does not exceed maximum speed.
            if(obj.SetSpeed_ > obj.MaxSpeed_)
                nexps = max(ceil(-log10(factor))+1,0);
                warning('Treadmill:MaxSpeedExceeded',...
                    'Requested speed (%.*f %s) exceeds maximum speed (%.*f %s), %s',...
                    nexps, obj.SetSpeed, units, nexps, obj.MaxSpeed, units, ...
                    'using maximum speed instead.\n');
                obj.SetSpeed_ = obj.MaxSpeed_;
            end
            
        % If not a valid speed, throw an error.
        else error('Treadmill:InvalidSpeed',...
                'Speed must be a numeric scalar or ''max''.')
        end
        
        % If the treadmill is running, force a speed/acceleration update.
        if(strcmpi(obj.Running,'on')); obj.sendSpeed(); end
    end
    
    function speed = get.SetSpeed(obj)
        % Get the current set speed (in the current units)
        % 'SetSpeed_' is stored in meters/minute.
        factor = conversionFactor(obj, 'speed');
        speed = obj.SetSpeed_*factor;
    end
    
    function set.AccelerationStep(obj, step)
        % Change the acceleration step size (in the current units)
        if(obj.AccelerationInterval == 0)
            warning('Treadmill:StepIgnored',...
                ['Because the acceleration interval is currently set to 0, '...
                 'the value you specified for acceleration step will be ignored.\n'...
                 'Change ''Acceleration'' to change both values together.']);
        else
            obj.Acceleration = [step, obj.AccelerationInterval];
        end
    end
    
    function step = get.AccelerationStep(obj)
        % Get the acceleration step size (in the current units)
        % 'AccelerationStep_' is stored in decimeters/minute, so in
        % addition to multiplying by conversion factor, divide by 10 to
        % convert from decimeters/minute to meters/minute.
        factor = conversionFactor(obj, 'acceleration');
        step = obj.AccelerationStep_*factor/10;
    end
    
    function set.AccelerationInterval(obj, intv)
        % Change the acceleration interval (in seconds)
        if(obj.AccelerationStep == 0)
            warning('Treadmill:IntervalIgnored',...
                ['Because the acceleration step is currently set to 0, '...
                 'the value you specified for acceleration interval will be ignored.\n'...
                 'Change ''Acceleration'' to change both values together.']);
        else
            obj.Acceleration = [obj.AccelerationStep, intv];
        end
    end
    
    function intv = get.AccelerationInterval(obj)
        % Get the acceleration interval (in seconds)
        % 'AccelerationInterval_' and 'AccelerationInterval' are always in
        % seconds, regardless of the 'TimeUnit'.
        intv = obj.AccelerationInterval_;
    end
    
    function set.Acceleration(obj, accel)
        % Set the desired acceleration rate of the treadmill. If the
        % treadmill is stopped, this acceleration rate will be used the
        % next time the treadmill is started. If the treadmill is running,
        % this change will take effect immediately.
        %
        % Valid accelerations include the strings 'max', 'min', or 'inf',
        % a number in the units specified above, or a 2 element vector
        % specifying the [step interval] pair. The actual rate of
        % acceleration is given by step/interval.
        %
        % 'max' means accelerate at a regulated 25.5 m/min/s
        %       this sets a step size of 25.5 m/min, and interval of 1 sec.
        % 'min' means accelerate at a regulated ~0.0004 m/min/s.
        %       this sets a step size of 0.1 m/min, and interval of 255 sec.
        % 'inf' means accelerate as fast as possible (unregulated).
        %       this sets a step size and interval of 0, which disables
        %       regulation of the rate of acceleration. This will be faster
        %       than the 'max' acceleration.
        
        % Check if the input was a string.
        if(ischar(accel))
            switch lower(accel)
                case 'inf'; step = 0; intv = 0;
                case 'max'; step = 255; intv = 1;
                case 'min'; step = 1; intv = 255;
                otherwise; error('Treadmill:InvalidAcceleration',...
                        'Acceleration string must be either ''inf'', ''max'', or ''min''.');
            end
            
            % Display the results if in debugging mode.
            if(obj.DebugMode)
                [factor, unitstr] = conversionFactor(obj, 'speed');
                stepout = step*factor/10;
                if(step == 0 || intv == 0); accelout = inf;
                else accelout = stepout/intv;
                end
                
                nexp = ceil(-log10(1/intv))+1;
                if(~isfinite(nexp)); nexp = 0; end
                ndiga = max(ceil(log10(accelout+1)),1)+nexp+1;
                if(~isfinite(ndiga)); ndiga = 1; end
                ndigs = max(ceil(log10(stepout+1)),1)+2;
                
                fprintf(1,'Requested: %s\n',accel);
                fprintf(1,'   Actual: %*.*f %s/s (Step: %*.1f %s [%03d], Intv: %03d)\n',...
                    ndiga, nexp, accelout, unitstr, ndigs, stepout, unitstr, step, intv);
            end
            
        % Check if the input was a two element vector.
        elseif(isnumeric(accel) && numel(accel)==2)
            assert(all(accel >= 0),'Treadmill:InvalidAcceleration',...
                'Acceleration must be positive.');
            [factor, unitstr] = conversionFactor(obj, 'speed');
        
            % Convert into the standard units used by treadmill.
            step = round(accel(1)*10/factor); % decimeters/min
            intv = round(accel(2));  % seconds
            
            % Check that the values fall within the allowed ranges.
            if(step > 255)
                warning('Treadmill:CappingAccelerationStep',...
                    'Requested acceleration step (%.1f %s) exceeds maximum (%.1f %s).',...
                    step*factor/10, unitstr, 255*factor/10, unitstr);
                step = 255;
            end
            if(intv > 255)
                warning('Treadmill:CappingAccelerationInterval',...
                    'Requested acceleration interval (%d s) exceeds maximum (%d s).',...
                    intv, 255);
                intv = 255;
            end
            
            % Display the results if in debugging mode.
            if(obj.DebugMode)
                stepin = accel(1);
                intvin = accel(2);
                accelin = stepin/intvin;
                stepout = step*factor/10;
                if(step == 0 || intv == 0); accelout = inf;
                else accelout = stepout/intv;
                end

                nexp = ceil(-log10(1/intv))+1;
                if(~isfinite(nexp)); nexp = 0; end
                ndiga = max(ceil(log10(max(accelin,accelout)+1)),1)+nexp+1;
                if(~isfinite(ndiga)); ndiga = 1; end
                ndigs = max(ceil(log10(max(stepin ,stepout )+1)),1)+2;
                ndigi = max(ceil(log10(max(intvin ,intv    )+1)),1)+2;
                
                fprintf(1,'Requested: %*.*f %s/s (Step: %*.1f %s      , Intv: %*.1f s      )\n',...
                    ndiga, nexp, accelin, unitstr, ndigs, stepin, unitstr, ndigi, intvin);
                fprintf(1,'   Actual: %*.*f %s/s (Step: %*.1f %s [%03d], Intv: %*.1f s [%03d])\n',...
                    ndiga, nexp, accelout, unitstr, ndigs, stepout, unitstr, step, ndigi, intv, intv);
            end
            
        % Check if 'inf' speed was requested.
        elseif(isnumeric(accel) && numel(accel)==1 && isinf(accel))
            step = 0;
            intv = 0;
            
            % Display the results if in debugging mode.
            if(obj.DebugMode)
                fprintf(1,'Requested: %f\n',accel);
                fprintf(1,'   Actual: %f\n',inf);
            end
            
        % Check if single (finite) number was provided.
        elseif(isnumeric(accel) && numel(accel)==1 && isfinite(accel))
            assert(accel > 0,'Treadmill:InvalidAcceleration',...
                'Acceleration must be positive.');
            
            % Convert the desired acceleration into the standard units
            [factor, unitstr] = conversionFactor(obj, 'acceleration');
            accelin = accel;
            accel = accel*10/factor; % decimeters/min/second
            
            % Check that the acceleration falls within the valid range.
            if(accel > 255)
                warning('Treadmill:CappingAcceleration',...
                    'Requested acceleration (%.1f %s) exceeds maximum (%.1f %s).',...
                    accel*factor/10, unitstr, 255*factor/10, unitstr);
                accel = 255;
            elseif(accel < 1/255)
                warning('Treadmill:CappingAcceleration',...
                    'Requested acceleration (%.4f %s) exceeds minimum (%.4f %s).',...
                    accel*factor/10, unitstr, 255*factor/10, unitstr);
                accel = 1/255;
            end
            
            % Determine the optimal values for step and interval.
            intv = max(min(round(128/accel), 255), 1); % seconds
            step = round(accel*intv); % decimeters/min
            
            % Sanity check, this shouldn't be a problem.
            assert(step<256 && step>0,'Treadmill:CalcAccelerationStep',...
                'Something weird happended when calculating acceleration step.');
            
            % Display the results if in debugging mode.
            if(obj.DebugMode)
                stepout = step*factor/10;
                accelout = stepout/intv;

                nexp = ceil(-log10(1/intv))+1;
                if(~isfinite(nexp)); nexp = 0; end
                ndiga = max(ceil(log10(max(accelin,accelout)+1)),1)+nexp+1;
                if(~isfinite(ndiga)); ndiga = 1; end
                ndigs = max(ceil(log10(stepout+1)),1)+2;
                
                fprintf(1,'Requested: %*.*f %s\n',...
                    ndiga, nexp, accelin, unitstr);
                fprintf(1,'   Actual: %*.*f %s (Step: %*.1f [%03d], Intv: %03d)\n',...
                    ndiga, nexp, accelout, unitstr, ndigs, stepout, step, intv);
            end
            
        % Otherwise throw an error.
        else
            error('Treadmill:InvalidAccelerationStep',...
                'Acceleration must be specified as a string (''inf'', %s',...
                '''max'', or ''min''), scalar, or as a [step, interval] pair.');
        end
        
        % Check if 'inf' acceleration was requested.
        if(step == 0 || intv == 0); step = 0; intv = 0; end
        
        % If the acceleration has changed, then flag the setting to be
        % updated the next time the speed of the treadmill is changed.
        if(obj.AccelerationStep_ ~= step || obj.AccelerationInterval_ ~= intv)
            obj.AccelerationStep_ = step;
            obj.AccelerationInterval_ = intv;
            obj.RefreshAccel = true;
        end
        
        % If the treadmill is running, force a speed/acceleration update.
        if(strcmpi(obj.Running,'on')); obj.sendSpeed(); end
    end
    
    function accel = get.Acceleration(obj)
        % Check if we are at unregulated 'inf' acceleration.
        if(obj.AccelerationStep_ == 0 || obj.AccelerationInterval_ == 0)
            accel = inf;
            
        % Otherwise, calculate the acceleration in decimeters/min/second,
        % then convert to the requested units.
        % 'AccelerationStep_' is stored in decimeters/minute
        % 'AccelerationInterval_' is stored in seconds
        else
            factor = conversionFactor(obj, 'acceleration');
            accel = obj.AccelerationStep_*factor/obj.AccelerationInterval_/10;
        end
    end
    
    function speed = get.MaxSpeed(obj)
        % 'MaxSpeed_' is stored in meters/minute.
        factor = conversionFactor(obj, 'speed');
        speed = obj.MaxSpeed_*factor;
    end
    
    function speed = get.CurrentSpeed(obj)
        % First make sure we have recent data.
        refreshTreadmillData(obj);
        
        % 'CurrentSpeed_' is stored in meters/minute.
        factor = conversionFactor(obj, 'speed');
        speed = obj.CurrentSpeed_*factor;
    end
    
    function [speed, ts] = GetSpeed(obj)
        speed = obj.CurrentSpeed();
        ts = obj.LastUpdate;
    end
    
    function beltmode = get.BeltMode(obj)
        % First make sure we have recent data.
        refreshTreadmillData(obj);
        beltmode = obj.BeltMode_;
    end
    
    function odometer = get.Odometer(obj)
        % First make sure we have recent data.
        refreshTreadmillData(obj);
        
        % 'Odometer_' is stored in meters.
        factor = conversionFactor(obj, 'distance');
        odometer = obj.Odometer_*factor;
    end
    
    function shockrate = get.ShockRate(obj)
        % First make sure we have recent data.
        refreshTreadmillData(obj);
        shockrate = obj.ShockRate_;
    end
    
    function start(obj, duration)
        % Send start command to the treadmill
        %
        %   Syntax:
        %      start(t) or t.start()
        %      start(t, duration) or t.start(duration)
        %
        %   Description:
        %      start(t) starts the treadmill 't' at the desired 'SetSpeed'
        %      and runs until 'stop' is called.
        %
        %      start(t, duration) start the treadmill 't' at the desired
        %      'SetSpeed' and run for 'duration' seconds.
        %
        %   Examples:
        %   
        %   Manual start/stop:
        %      t = Treadmill();    % Initialize the treadmill.
        %      t.SetSpeed = 'max'; % Set the desired speed.
        %      t.start();          % Start the treadmill.
        %      disp(t.CurrentSpeed)% Display the current treadmill speed.
        %      for ii = 1:10;
        %          pause(1);
        %          disp(t.CurrentSpeed)
        %      end
        %      t.stop();           % Stop the treadmill.
        %
        %   Automatic (timed) stop:
        %      t = Treadmill();    % Initialize the treadmill.
        %      t.SetSpeed = 'max'; % Set the desired speed.
        %      t.start(10);        % Start and run for 10 seconds.
        %      for ii = 1:15;
        %          pause(1);
        %          disp(t.CurrentSpeed);
        %          disp(t.Odometer);
        %      end
        %      dist = t.Odometer;  % Read the odometer (optional).

        % Check that we aren't already running.
        assert(strcmpi(obj.Running,'off'),...
            'Treadmill:AlreadyRunning',...
            'Cannot start treadmill because it is already running.');
        
        % Check if a specified duration was provided.
        if(nargin == 2 && isnumeric(duration) && isscalar(duration))
            % Check that a timer isn't already running.
            assert(~isa(obj.timer,'timer') || ~isvalid(obj.timer),...
                'Treadmill:DuplicateTimer',...
                'Cannot start treadmill a timer object already exists.');
            
            % Create a timer object that will handle starting and stopping
            % the treadmill.
            obj.timer = timer('StartFcn',@(x,~) obj.start(),...
                'StartDelay',duration,'Name','Treadmill Timer',...
                'TimerFcn',@(~,~) obj.stop(),...
                'ErrorFcn',@(~,~) obj.stop()); %#ok<PROP,CPROP>
            
            % Start the timer.
            start(obj.timer);
        else
            % Send the SetSpeed to the treadmill
            obj.sendSpeed();
            
            % Mark the treadmill as 'Running'.
            obj.Running = 'on';
        end
    end
    
    function stop(obj)
        % Send stop command to the treadmill
        %
        %   Syntax:
        %      stop(t) or t.stop()
        %
        %   Description:
        %      stop(t) stops the treadmill 't' immediately. If a timed run
        %      is occurring, the timer is stopped and deleted.
        %      No check is done to see if the treadmill is currently
        %      running, so this function can also serve to 'reset' the
        %      treadmill to a stopped state if the code crashes while the
        %      treadmill is running.
        
        % Send the speed zero to the treadmill.
        obj.sendSpeed(0);
        
        % Mark the treadmill as not 'Running'.
        obj.Running = 'off';
        
        % Check if a timer object was being used. Stop and delete it.
        if(isa(obj.timer,'timer') && isvalid(obj.timer))
            if(strcmpi(obj.timer.Running,'on')); stop(obj.timer); end
            delete(obj.timer);
        end
    end
    
    function clearLog(obj)
        obj.Log = NaN(0,3);
    end
    
    function success = resetOdometer(obj)
        % Reset the odometer.
        %
        %   Syntax:
        %      resetOdometer(t) or t.resetOdometer()
        %      success = resetOdometer(t) or success = t.resetOdometer()
        %
        %   Description:
        %      resetOdometer(t) resets the odometer to zero. An error is
        %      thrown in the event of a communciation error.
        %
        %      success = resetOdometer(t) resets the odometer to zero. The
        %      output 'success' indicates whether a the command was
        %      successful.
        
        obj.sendcommand([obj.DID 035]); % Reset Odometer (comand 035)
        success = obj.readAcknowledge('reset odometer', nargout == 0);
        if(nargout == 0); clear('success'); end
    end
    
    function success = resetController(obj)
        % Reset the treadmill controller.
        %
        %   Syntax:
        %      resetController(t) or t.resetController()
        %      success = resetController(t) or success = t.resetController()
        %
        %   Description:
        %      resetController(t) resets the treadmill controller. An error
        %      is thrown in the event of a communciation error.
        %
        %      success = resetController(t) resets the treadmill
        %      controller. The output 'success' indicates whether a the
        %      command was successful.
        
        obj.sendcommand([obj.DID 016]); % Reset CI-Bus (command 016)
        java.lang.Thread.sleep(20);     % Pause for 20 ms
        obj.sendcommand([obj.DID 017]); % Return Device ID (command 017)
        [~, success] = obj.readresponse(10, NaN, nargout == 0);
        if(nargout == 0); clear('success'); end
    end
    
    function val = get(obj, prop)
        % Return the value of the specified property or properties.
        %
        %   Syntax:
        %      val = get(t, Property) or val = t.get(Property)
        %      val = get(t, {Property1, Property2, ...})
        %      val = get(t) or val = t.get()
        %
        %   Description:
        %      val = get(t, Property) returns the value of the specified
        %      property.
        %
        %      val = get(t, {Property1, Property2, ...}) returns the values of
        %      the specified properties in a cell array.
        %
        %      val = get(t) returns the value of all properties in a cell
        %      array.

        if(nargin < 2)
            props = properties(obj);
            val = cell2struct(obj.get(props), props);
            if(nargout == 0)
                disp(val)
                clear('val')
            end
        elseif(iscell(prop))
            val = cellfun(@(x) obj.get(x), prop, 'UniformOutput',false);
        elseif(ischar(prop)); val = obj.(prop);
        else error('MATLAB:class:InvalidArgument',...
                'Second argument must be string or cell array.');
        end
    end
    
    function port = resetSerial(obj, port)
        % Reset the serial port connection to the treadmill controller.
        %
        %   Syntax:
        %      port = t.resetSerial()
        %      port = t.resetSerial(port)
        %
        %   Description:
        %      port = t.resetSerial() resets the connection to the
        %      treadmill controller using either the current serial port
        %      (if available), the serial port saved in settings (if
        %      available), or the only available serial port. If multiple
        %      serial ports exist and it isn't clear which to use, a list
        %      is produced showing all available serial ports. The port
        %      used is returned as an output argument.
        %
        %      t.resetSerial(port) resets the connection to the treadmill
        %      controller using the specified port (if available).
        
        % Check if we already have a valid serial port object.
        % If so, delete the existing object before creating a new one.
        if(isa(obj.s,'serial') && isvalid(obj.s))
            if(isempty(obj.Port)); obj.Port = obj.s.Port; end
            delete(obj.s);
        end
        
        % Check if a specific port was requested. If not, try to use the
        % port stored in the object.
        if(exist('port','var')~=1 || ~ischar(port) || isempty(port))
            port = obj.Port;
        end
        
        % If no port was stored in object, use port saved in preferences.
        if(isempty(port) && ispref('Treadmill','Port'))
            port = getpref('Treadmill','Port');
        end
        
        % If we still don't have a port name, then try to find one:
        if(isempty(port)); port = Treadmill.findSerialPort(); end
        
        % Check that the requested port is valid and available.
        port = Treadmill.checkSerialPort(port);
        
        % Create a serial port object.
        obj.s = serial(port, 'BaudRate', obj.BaudRate, 'DataBits', 8,...
            'Timeout', 1, 'Parity', 'none', 'StopBits', 1,...
            'FlowControl', 'none');
        
        % Open a connection to the serial port.
        fopen(obj.s);
        obj.Port = port;
        
        % Clear the output if it was not requested.
        if(nargout == 0); clear('port'); end
    end
    
    function set.DebugMode(obj, mode)
        % Debugging mode will print out commands as they are sent to the
        % treadmill controller, and the response. As well as some other
        % more verbose outputs.
        
        % Mode can be either 'on'/'off' or logical true/false.
        if(ischar(mode)); mode = lower(mode);
        else assert(islogical(mode) && isscalar(mode),...
                'Treadmill:InvalidDebugMode',...
                'Debug mode must be either ''true'' or ''false''');
        end
        
        % Map from requested mode to either true or false.
        switch mode
            case {true, 'on', 'true'}; obj.DebugMode = true;
            case {false, 'off', 'false'}; obj.DebugMode = false;
            otherwise
                error('Treadmill:InvalidDebugMode',...
                    'Debug mode must be either ''true'' or ''false''');
        end
    end
    
    function success = manualMode(obj)
        % Set the treadmill controller into manual mode.
        %
        %   Syntax:
        %      t.manualMode()
        %      success = t.manualMode()
        %
        %   Description:
        %      manualMode(t) sets the treadmill controller into manual mode
        %      so that the front panel switches are functional. The
        %      computer will automatically switch back to computer control
        %      mode when the treadmill is told to 'start' or 'stop'. An
        %      error is thrown in the event of a communciation error.
        %
        %      success = resetController(t) sets the treadmill controller
        %      into manual mode. The output 'success' indicates whether a
        %      the command was successful.
        
        obj.sendcommand([obj.DID 048 000]); % Select Interface (comand 048)
        success = obj.readAcknowledge('select interface', nargout == 0);
        if(nargout == 0); clear('success'); end
    end
    
    function disp(obj)
        % Display information about treadmill object.
        %
        %   Syntax:
        %      disp(t) or t.disp()
        %
        %   Description:
        %      disp(t) displays a user-friendly summary of the treadmill
        %      object.
        
        % Get the proper conversion factors and units.
        [dfact,dunit] = conversionFactor(obj, 'distance');
        [sfact,sunit] = conversionFactor(obj, 'speed');
        [afact,aunit] = conversionFactor(obj, 'acceleration');
        if(strcmpi(obj.TimeUnit,{'sec','s'})); iunit = obj.TimeUnit;
        else iunit = 's';
        end
        
        % Determine how many decimal digits to display.
        nexpd = max(ceil(-log10(dfact))+1,0);
        nexps = max(ceil(-log10(sfact))+1,0);
        nexpa = max(ceil(-log10(afact/obj.AccelerationInterval))+1,0);
        
        % Display the information.
        fprintf(1,'Treadmill Controller Object\n');
        fprintf(1,'\nTreadmill Details\n');
        fprintf(1,'       Device ID: %03d\n',obj.DID);
        fprintf(1,'        Group ID: %03d\n',obj.GID);
        fprintf(1,'      Verbose ID: %s\n',obj.VerboseID);
        fprintf(1,'         Product: %s\n',obj.Product);
        fprintf(1,'        Firmware: %s\n',obj.Firmware);
        fprintf(1,'\nTreadmill Settings\n');
        fprintf(1,'       Set Speed: %.*f %s\n', nexps, obj.SetSpeed, sunit);
        fprintf(1,'    Acceleration: %.*f %s\n', nexpa, obj.Acceleration, aunit);
        fprintf(1,'            Step: %.*f %s [%03d]\n',...
            nexps, obj.AccelerationStep, sunit, obj.AccelerationStep_);
        fprintf(1,'        Interval: %d %s [%03d]\n',...
            obj.AccelerationInterval, iunit, obj.AccelerationInterval);
        fprintf(1,'       Max Speed: %.*f %s\n', nexps, obj.MaxSpeed, sunit);
        fprintf(1,'\nTreadmill Status\n');
        fprintf(1,'         Running: %s\n', obj.Running);
        fprintf(1,'   Current Speed: %.*f %s\n', nexps, obj.CurrentSpeed, sunit);
        fprintf(1,'        Odometer: %.*f %s\n', nexpd, obj.Odometer, dunit);
        fprintf(1,'Belt Mode Switch: %s\n', obj.BeltMode);        
    end
    
    function delete(obj)
        % Disconnect from the treadmill and delete treadmill object.
        %
        %   Syntax:
        %      delete(t) or t.delete()
        %
        %   Description:
        %      delete(t) will stop the treadmill, stop and delete any
        %      timers associated with the object, switch the treadmill back
        %      into manual model, and disconnect from the treadmill.
        
        % Just in case, check if a timer object exists. Stop and delete it.
        if(isa(obj.timer,'timer') && isvalid(obj.timer))
            if(strcmpi(obj.timer.Running,'on')); stop(obj.timer); end
            delete(obj.timer);
        end
        
        % Check if a valid serial port object exists.
        if(isa(obj.s,'serial') && isvalid(obj.s))
            % Check if the connection is open and we have a valid DID.
            if(strcmpi(obj.s.Status,'open') && obj.DID > 0)
                % Stop the treadmill
                try
                    stop(obj);
                catch ME
                    fprintf(2,'Error stopping treadmill during Treadmill deconstructor.\n');
                    fwrite(1,ME.getReport());
                end
                
                % Switch back to manual mode if possible.
                try
                    success = obj.manualMode(); %#ok<NASGU>
                catch ME
                    fprintf(2,'Error switching to manual mode during Treadmill deconstructor.\n');
                    fwrite(1,ME.getReport());
                end
            end
            
            % Delete the serial port object.
            delete(obj.s);
        end
    end
end

methods(Access=protected)
    function sendcommand(obj, command)
        % Send the specified command to the treadmill controller.
        %
        % The command consists of a vector of numbers between 0 and 255.
        % The checksum is automatically computed and added to the end of
        % the command.
        
        % Compute the checksum and add it to the end of the command.
        command = Treadmill.checksum(command);
        
        % If in debugging mode, report the command to the terminal.
        if(obj.DebugMode)
            fprintf(1,'  Sending:%s (%s)\n',sprintf(' %03d', command),...
                Treadmill.commandstr(command(2)));
        end
        
        % Silently fail if the device ID (or group ID) is zero.
        if(command(1)==0);
            if(obj.DebugMode)
                fprintf(1,'Send aborted because first value is zero.\n');
            end
            return;
        end
        
        % Write the command to the serial port.
        fwrite(obj.s,command);
    end
    
    function [out, tf] = readresponse(obj, bytes, timeout, varargin)
        % Read the response returned from the treadmill controller.
        % 
        % Three modes of operation:
        %   Mode 1: Read until number of bytes is returned or serial port
        %   timeout is reached. Used when bytes > 0 and timeout not specified.
        %
        %   Mode 2: Poll until number of bytes is available, or specified
        %   timeout is reached. Used when bytes > 0 and timeout is specified.
        %
        %   Mode 3: wait specified timeout, read whatever is available.
        %   Used when bytes == 0 and timeout is specified.
        %
        % Optional string input is treated as the name of the operation
        % (command string) that produced the response. This is used in
        % error messages. If a command string is provided without 'err',
        % then an error will be thrown in the event of an unexpected (or
        % missing) response.
        %
        % Optional logical input is treated as 'err' and specifies
        % whether to throw an error in the event of an unexpected (or
        % missing) response. If 'err' is not provided, an error will be
        % thrown if the 'tf' output is not requested (nargin < 2), or if a
        % command string was provided.
        
        % Process inputs.
        for a = 1:numel(varargin)
            if(ischar(varargin{a}))
                commandstr = varargin{a};
            elseif(islogical(varargin{a}) && isscalar(varargin{a}))
                err = varargin{a};
            end
        end
        
        % Determine whether to throw any errors, or just return 'tf'.
        if(exist('err','var')~=1);
            if(exist('commandstr','var')==1); err = true;
            else err = (nargout < 2);
            end
        end
        
        % Compose the string describing the command.
        if(exist('commandstr','var')==1)
            commandstr = sprintf(' to %s command', commandstr);
        else
            commandstr = '';
        end
        
        % Check the 'timeout' input.
        if(exist('timeout','var')==1)
            if(isscalar(timeout) && isfinite(timeout)); havetimeout = true;
            else havetimeout = false;
            end
        else havetimeout = false;
        end
        
        % Check the 'bytes' input.
        if(exist('bytes','var')==1)
            if(~isscalar(bytes) || ~isfinite(bytes))
                if(havetimeout); bytes = 0;
                else bytes = 3;
                end
            end
        else bytes = 3;
        end
        
        % Three modes of operation:
        %   Mode 1: Read until number of bytes is returned or serial port
        %   timeout is reached. Used when bytes > 0 and timeout not specified.
        if(bytes > 0 && ~havetimeout)
            [out, count, ~] = fread(obj.s, bytes);
            
            % Dump any extra bytes left in the queue.
            if(obj.s.BytesAvailable > 0)
                fread(obj.s, obj.s.BytesAvailable);
            end
            
        %   Mode 2: Poll until number of bytes is available, or specified
        %   timeout is reached. Used when bytes > 0 and timeout is specified.
        elseif(bytes > 0 && havetimeout)
            % Loop until we have enough bytes, or reach the timeout.
            tick = 0;
            dt = 10;
            while((obj.s.BytesAvailable < bytes) && (tick + dt <= timeout))
                java.lang.Thread.sleep(dt);
                tick = tick + dt;
            end
            
            % Read whatever data is available.
            if(obj.s.BytesAvailable > 0)
                [out, count, ~] = fread(obj.s, obj.s.BytesAvailable);
            else out = zeros(1,0); count = 0;
            end
        
        %   Mode 3: wait specified timeout, read whatever is available.
        %   Used when bytes == 0 and timeout is specified.
        elseif(bytes == 0 && havetimeout)
            java.lang.Thread.sleep(timeout)
            if(obj.s.BytesAvailable > 0)
                [out, count, ~] = fread(obj.s, obj.s.BytesAvailable);
            else out = zeros(1,0); count = 0;
            end
        else error('Treadmill:InvalidReadRequest',...
                'Must specify either bytes > 0 or a timeout (or both).');
        end
        
        % If we are in debugging mode, print the data received.
        if(obj.DebugMode)
            fprintf(1,'Receiving:%s\n',sprintf(' %03d', uint8(out)));
        end
        
        % Throw an error if appropriate.
        assert(~err || count>0, 'Treadmill:NoResponse',...
            'No response received%s.',commandstr);
        assert(~err || bytes == 0 || count==bytes,...
            'Treadmill:IncompleteResponse',...
            'Response received%s was shorter than expected (%d/%d)',...
            commandstr, count, bytes);
        
        % Verify the checksum.
        tf = Treadmill.verifyChecksum(out, err) && (count == bytes);
    end

    function tf = readAcknowledge(obj, commandstr, err)
        % Look for an acknowledgement response from the treadmill
        %
        % If no output is requested, or if 'err' is true, an error will be
        % thrown if no (or an invalid) acknowlegement is received.
        
        % Compose the string describing the command.
        if(exist('commandstr','var')==1)
            commandstr = sprintf(' to %s command', commandstr);
        else
            commandstr = '';
        end
        
        % Acknowledge message is 3 bytes long.
        bytes = 3;
        
        % Determine whether to throw any errors, or just return 'tf'.
        if(exist('err','var')~=1 || ~islogical(err))
            err = (nargout == 0);
        end
        
        % Read the response from the serial port.
        [out, count, ~] = fread(obj.s, 3);
        
        % Dump any extra bytes left in the queue.
        if(obj.s.BytesAvailable > 0)
            fread(obj.s, obj.s.BytesAvailable);
        end
        
        % If we are in debugging mode, print the data received.
        if(obj.DebugMode)
            fprintf(1,'Receiving:%s\n',sprintf(' %03d', uint8(out)));
        end
        
        % Throw an error if appropriate.
        assert(~err || count>0, 'Treadmill:NoResponse',...
            'No acknowledgement%s received.',commandstr);
        assert(~err || count==bytes,'Treadmill:IncompleteResponse',...
            'Acknowledgement received% was shorter than expected (%d/%d)',...
            commandstr, count, bytes);
        
        % Verify checksum and that the response was '6' (for 'Acknowledge')
        tf = Treadmill.verifyChecksum(out, err) ...
            && (count == bytes) ...
            && out(2) == 6;
        
        % Throw an error for an invalid acknowledgement
        assert(~err || tf,'Treadmill:InvalidAcknowledgement',...
            'Invalid acknowledgement%s received.',commandstr);
    end
    
    function refreshTreadmillData(obj)
        % Refresh the speed, belt mode, odometer reading, and shock rate.
        
        % Check if we have (fresh) cached values, or if we should do
        % another read.
        if((now-obj.LastUpdate)*24*60*60 < obj.MinRefresh); return; end

        % Request the data from the treadmill.
        obj.sendcommand([obj.DID 034]); % Return Treadmill Data (command 034)
        
        % Record when the command was sent.
        obj.LastUpdate = now;
        
        % Read the data.
        out = obj.readresponse(8, NaN, true, 'return treadmill data');
        
        % Parse the response.
        obj.CurrentSpeed_ = (out(3)+256*out(2))/10;
        obj.Odometer_ = (out(6)+256*out(7))/10;
        
        % Parse the 'BeltMode' and 'ShockRate' response.
        bits = logical(bitget(out(5),[0 1 4 5]+1));
        if    (~bits(1) && ~bits(2)); obj.BeltMode_ = 'Run';
        elseif( bits(1) && ~bits(2)); obj.BeltMode_ = 'Stopped';
        elseif(~bits(1) &&  bits(2)); obj.BeltMode_ = 'Acceleration';
        else                          obj.BeltMode_ = '';
        end
        
        obj.ShockRate_ = bits(3)*2+bits(4)+1;
    end
    
    function beltdisabled = warnBeltMode(obj)
        % Warn if the 'Treadmill Belt' switch is in the wrong mode. If the
        % switch is on 'Stop' the computer interface is disabled.
        
        beltdisabled = strcmp(obj.BeltMode,'Stopped');
        if(beltdisabled)
            warndlg({'Treadmill is currently disabled in hardware.',...
                'Move ''Treadmill Belt'' switch to either ''Running'' or ''Acceleration'' to re-enable.'},...
                'Treadmil Disabled');
        end
    end
    
    function success = sendSpeed(obj, speed)
        % Send the requested speed to the treadmill (in meters/minute).
        
        % Check if a speed was provided, otherwise use the 'SetSpeed'.
        if(nargin < 2); speed = obj.SetSpeed_; end
        
        % Calculate the high and low bits.
        SPH = uint8(floor(speed*10/256));
        SPL = uint8(mod(speed*10,256));
        
        % Make sure the treadmill is in 'computer controlled' mode.
        obj.sendcommand([obj.DID 048 001]); % Select Interface (comand 048)
        success = obj.readAcknowledge('select interface', nargout == 0);
        if(~success); return; end
        
        % If the acceleration has changed since the last call to
        % 'sendSpeed', then send both speed and acceleration.
        if(obj.RefreshAccel)
            ASP = uint8(mod(obj.AccelerationStep_    ,256));
            AIN = uint8(mod(obj.AccelerationInterval_,256));
            obj.sendcommand([obj.DID 049 SPH SPL ASP AIN]); % Set Speed and Acceleration(comand 049)
            
        % Otherwise just send the speed, which will use the existing
        % acceleration setting.
        else
            obj.sendcommand([obj.DID 049 SPH SPL]); % Set Speed (comand 049)
        end
        
        % Record the time the command was sent to the treadmill.
        ts = now();
        
        % Look for an aknowledgement.
        success = obj.readAcknowledge('set speed', nargout == 0);
        
        % Record event if successful.
        if(success)
            factor = conversionFactor(obj, 'speed');
            obj.Log(end+1,:) = [ts factor*speed obj.Acceleration];
            obj.RefreshAccel = false;
        end
        
        % Warn if the belt mode switch is preventing the treadmill from
        % actually starting. I'm calling this after actually starting the
        % treadmill to avoid a delay when starting the treadmill.
        if(success && speed > 0); obj.warnBeltMode; end
    end
    
    function [factor, unitstr] = conversionFactor(obj, type)
        % Determine the conversion factor between the default units (meters
        % and minutes) and the requested units.
        
        % List of valid distance units.
        distunits =  {'mm', 'cm', 'm', 'km', 'inches', 'feet', 'miles'};
        distunitsstr = sprintf('%s ', distunits{:});
        
        % List of valid time units.
        timeunits =  {'s', 'sec', 'min', 'h', 'hours'};
        timeunitsstr = sprintf('%s ', timeunits{:});
        
        % Figure out the correct distance conversion.
        %       [newunit] = distconv * [meters]
        switch obj.DistUnit
            case 'mm';      distconv = 1000;        % 1000 mm =    1 meter
            case 'cm';      distconv = 100;         %  100 cm =    1 meter
            case 'm' ;      distconv = 1;           %    1  m =    1 meter
            case 'km';      distconv = 1/1000;      %    1 km = 1000 meters
            case 'inches';  distconv = 10000/254;
            case 'feet';    distconv = 10000/254/12;
            case 'miles';   distconv = 10000/254/12/5280;
            otherwise; error('Treadmill:InvalidDistUnit',...
                    'Distance unit must be one of: %s.', distunitsstr);
        end
        
        % Figure out the correct time conversion.
        switch obj.TimeUnit
            case {'s','sec'};    timeconv = 60;     % 60 sec  =  1 min
            case 'min';          timeconv = 1;      %  1 min  =  1 min
            case {'h', 'hours'}; timeconv = 1/60;   %  1 hour = 60 min
            otherwise; error('Treadmill:InvalidTimeUnit',...
                    'Time unit must be one of: %s.', timeunitsstr);
        end
        
        % Determine whether we are looking for a conversion from distance,
        % speed, or acceleration.
        switch lower(type)
            case {'distance','dist'}
                factor = distconv;
                unitstr = obj.DistUnit;
            case {'speed','velocity'}
                factor = distconv/timeconv;
                unitstr = sprintf('%s/%s',obj.DistUnit,obj.TimeUnit);
            case {'accel','acceleration'}
                % Acceleration is always 'per second', so conversion is
                % the same as speed conversion.
                factor = distconv/timeconv;
                
                % If the time unit is 'sec' or 's' then use 'sec^2' or 's^2'
                % If user wants 'sec', use 'sec'. Otherwise prefer 's'.
                if(any(strcmp(obj.TimeUnit,{'s','sec'})))
                    unitstr = sprintf('%s/%s^2',obj.DistUnit,obj.TimeUnit);
                else
                    unitstr = sprintf('%s/%s/s',obj.DistUnit,obj.TimeUnit);
                end
            case 'time'
                factor = timeconv;
                unitstr = obj.TimeUnit;
            otherwise; error('Treadmill:InvalidConversionType',...
                    'Unrecognized conversion type (%s), please choose: %s',...
                    lower(type), 'dist(ance) , speed, accel(eration), or time');
        end
    end
end

methods(Static, Access=public)
    function command = checksum(command)
        % Convert to 'uint8' and calculate the checksum for the command.
        % Checksum is the sum of the command mod 256.
        % Input: numerical vector specifying command.
        
        command = uint8(mod(command,256));
        command = [command uint8(mod(sum(command),256))];
    end
    
    function tf = verifyChecksum(command, warn)
        % Verify that the checksum on the response is valid.
        % Checksum is the sum of the command mod 256.
        % Input: numerical vector specifying command, and logical 'warn'
        % indicating whether to generate a warning when the checksum is
        % invalid.
        
        % Make sure we are dealing with 'uint8'.
        command = uint8(mod(command,256));
        
        % Must have at least 1 element (the checksum) to validate.
        if(numel(command)<1)
            tf = false;
            return
        end
        
        % Calculate the correct checksum.
        cs = mod(sum(command(1:end-1)),256);
        
        % Verify if it matchces the last element of the response.
        tf = (cs == command(end));
        
        % Determine whether to throw a warning, or just return 'tf'.
        if(exist('warn','var')~=1 || ~islogical(warn))
            warn = (nargout == 0);
        end
        
        % Generate a warning if appropriate.
        if(warn && ~tf)
            warning('Treadmill:VerifyChecksumFailed',...
                'Checksum (%03d) did not match expected value (%03d).',...
                command(end), cs);
        end
    end
    
    function [port, status, info] = checkSerialPort(port, info)
        % Check if the named serial port is valid and available.
        %
        %   Syntax:
        %      [port, status, info] = checkSerialPort(port)
        %      [port, status, info] = checkSerialPort(port, info)
        %
        %   Description:
        %      [port, status, info] = checkSerialPort(port) checks to see
        %      if the 'port' specified (as a string) is among the valid
        %      serial ports on the system, and whether it is available.
        %      'port' is the specified port name with capitalization
        %      standardized to match the system.
        %      'status' is either:
        %         0 = not a valid serial port
        %         1 = valid serial port that is unavailable
        %         2 = valid serial port that is avaialble
        %      'info' is the output of the function 'instrhwinfo'
        %
        %      [port, status, info] = checkSerialPort(port, info) as above,
        %      except the output from 'instrhwinfo' is provided as input so
        %      it doesn't need to be run again.
        
        % Get information about serial ports on the system.
        if(nargin < 2); info = instrhwinfo('serial'); end
        
        % Check if the requested port matches one of the serial ports.
        tf = strcmpi(port,info.SerialPorts);
        if(sum(tf)~=1)
            status = 0;
            return
        end
        port = info.SerialPorts{tf};
        
        % Check if the requested port is available.
        tf = strcmpi(port,info.AvailableSerialPorts);
        if(sum(tf)~=1); status = 1;
        else status = 2;
        end
        
        % Produce an error if appropriate.
        if(nargout < 2)
            if(status == 0)
                error('Treadmill:InvalidSerialPort',...
                'Specified serial port (''%s'') is invalid.', port);
            elseif(status == 1)
                error('Treadmill:COMPortInUse',...
                'Requested serial port (''%s'') is already in use.', port);
            end
        end
    end
    
    function [port, info] = findSerialPort(info)
        % Find a valid and available serial port.
        %
        %   Syntax:
        %      [port, info] = findSerialPort()
        %      [port, info] = findSerialPort(info)
        %
        %   Description:
        %      [port, info] = findSerialPort() looks for an available
        %      serial port on the system. If exactly one serial port is
        %      found, and that serial port is available, this function
        %      returns the name of that port as the 'port' output. If
        %      multiple ports are found, an error is thrown including a
        %      list of the available ports for you to choose from. If no
        %      ports are found, or all the ports are in use, an error
        %      is thrown.
        %
        %      [port, info] = findSerialPort(info) as above, except the
        %      output from 'instrhwinfo' is provided as input so it doesn't
        %      need to be run again.
        
        % Get information about serial ports on the system.
        if(nargin < 1); info = instrhwinfo('serial'); end
        
        % Find out which ports are available.
        available = ismember(info.SerialPorts, info.AvailableSerialPorts);
        
        % Create a string that lists ports for use later.
        portlist = '';
        for p = 1:numel(info.SerialPorts)
            if(available(p))
                portlist = sprintf('%s%s: %s\n',portlist,...
                    info.SerialPorts{p}, 'Available');
            else
                portlist = sprintf('%s%s: %s\n',portlist,...
                    info.SerialPorts{p}, 'In Use');
            end
        end
        
        % Determine whether exactly one available port is found, or throw
        % the appropriate error.
        if(isempty(info.SerialPorts))
            error('Treadmill:NoCOMPorts',...
                'Could not find any serial ports.');
        elseif(isempty(info.AvailableSerialPorts))
            error('Treadmill:AllSerialPortsInUse',...
                'All serial ports are currently in use.\n%s%s',portlist,...
                'Use ''Treadmill.deleteSerial(port)'' to free port.');
        elseif(numel(info.SerialPorts)==1)
            port = info.AvailableSerialPorts{1};
        else
            error('Treadmill:MultipleSerialPorts',...
                'Multiple serial ports detected, please select one:\n%s%s',...
                portlist,'Use ''Treadmill.deleteSerial(port)'' to free port.');
        end
    end
    
    function [did, port, verboseids] = listDevices(varargin)
        % List the treadmill controllers found attached to the system.
        %
        %   Syntax:
        %      listDevices()
        %      [did, port, verboseids] = listDevices()
        %      ... = listDevices(OPTIONS)
        %
        %   Description:
        %      listDevices() attempts to find all the treadmill controllers
        %      attached to the system using automatically detected serial
        %      port and the full range of available device IDs. Information
        %      about devices detected is printed to the screen.
        %
        %      [did, port, verboseids] = listDevices() as above, except no
        %      output is sent to the screen.
        %      'did' is the device ID of any detected devices.
        %      'port' is the serial port the devices were detected on.
        %      'verboseids' is the verbose IDs of the detected devices.
        %
        %      ... = listDevices(OPTIONS) attempts to find all the
        %      treadmill controllers attached to the system.
        %      Legal OPTIONS include: a list of device IDs to look for
        %      (default 1:239), a named serial port (by default
        %      'findSerialPort' is used to locate the correct serial port),
        %      or a serial port object.
        
        % Process input arguments.
        for a = 1:nargin
            if(ischar(varargin{a}) && ~isempty(varargin{a}))
                port = varargin{a};
            elseif(isnumeric(varargin{a}))
                list = varargin{a};
            elseif(isa(varargin{a},'serial') && isvalid(varargin{a}))
                sp = varargin{a};
                port = sp.Port;
            end
        end
        
        % Get information about available serial ports.
        info = instrhwinfo('serial');
        
        % Get the list of device IDs to check, default 1:239.
        if(exist('list','var')~=1); list = 1:239;
        else list = unique(list(:))';
        end
        
        % Check that a valid serial port is available, and open connection.
        if(exist('sp','var')~=1)
            if(exist('port','var')==1)
                port = Treadmill.checkSerialPort(port, info);
            else
                port = Treadmill.findSerialPort(info);
            end
        
            % Create the serial port object and open the connection.
            sp = serial(port, 'BaudRate', 19200, 'DataBits', 8, 'Timeout', 10, ...
                'Parity', 'none', 'StopBits', 1, 'FlowControl', 'none');
            fopen(sp);
            
            % Automatically close the connection when we are done.
            cu1 = onCleanup(@(x,y) delete(sp));
        elseif(strcmpi(sp.Status,'closed'))
            fopen(sp);
            
            % If we opened the connection, then automatically close the
            % connection when we are done.
            cu1 = onCleanup(@(x,y) fclose(sp));
        end
        
        % Shortcuts to commonly used functions:
        function sendcmd(command)
            fwrite(sp,Treadmill.checksum(command));
            % sleep for 50 ms, probably to let it execute before you do
            % anything else
            java.lang.Thread.sleep(50);
        end
        verifyChecksum = @(out) Treadmill.verifyChecksum(out);
        
        % Send the 'reset' and 'return device ID' to the broadcast address.
        sendcmd([255 016]); % Reset CI-Bus (command 016)
        sendcmd([255 017]); % Return Device ID (command 017)
        
        % Create a status bar
        h = waitbar(0,sprintf('Scanning CI-Bus on %s (%02.0f%%)\n', port, 0));
        cu2 = onCleanup(@(x,y) close(h));

        % Scan through the list of device IDs, and see which respond to a
        % request for their Device ID.
        did = nan(size(list));
        verboseids = cell(size(list));
        for d = 1:numel(did);
            % Send the 'retransmit' command to a specific DID. If the
            % device is available, it will reply with it's device ID.
            sendcmd([list(d) 024]); % Retransmit (command 24)
            
            % Look for a (valid) response.
            if(sp.BytesAvailable == 10 && ...
                    verifyChecksum(fread(sp, sp.BytesAvailable)))
                
                % If a valid response was found, then a valid device ID was
                % found, now request the verbose ID from that device.
                did(d) = list(d);
                sendcmd([list(d) 020]); % Return Verbose ID (command 020)
                
                % Sleep long enough for a full reply.
                java.lang.Thread.sleep(500);
                
                % Read the full verbose ID.
                if(sp.BytesAvailable >= 3)
                    out = fread(sp, sp.BytesAvailable);
                    if(verifyChecksum(out))
                        verboseids{d} = char(out(2:end-1))';
                    end
                end
            end
            
            % If no output is requested, then print the discovered device
            % information to the screen.
            if(nargout == 0 && ~isnan(did(d)))
                fprintf(1,'%3d: %s\n',list(d),verboseids{d});
            end
            
            % Update the status bar.
            if(ishandle(h))
                waitbar(d/numel(did),h,sprintf('Scanning CI-Bus on %s (%02.0f%%)\n',...
                    port, 100*d/numel(did)));
                
            % If the status bar was closed, then abort.
            else break
            end
        end
        
        % Keep only those device IDs that were actually found.
        verboseids = verboseids(~isnan(did));
        did = did(~isnan(did));
        
        % If we found any valid devices, then save the serial port that was
        % used for future use.
        if(numel(did)>0); setpref('Treadmill','Port',port); end
    end
    
    function deleteSerial(port)
        % Delete any serial port objects attached to the specified port.
        %
        %   Syntax:
        %      deleteSerial()
        %      deleteSerial(port)
        %
        %   Description:
        %      deleteSerial() delete all serial port objects on the system.
        %
        %      deleteSerial(port) delete any serial port objects that are
        %      associated with the specified 'port'.
        
        if(nargin == 0)
            delete(instrfindall);
        else
            [port, ~] = checkSerialPort(port);
            delete(instrfindall('Port',port));
        end
    end
    
    function str = commandstr(commandid)
        % Return the command string associated with each command code.
        %
        %   Syntax:
        %      str = commandstr(commandid)
        %
        %   Description:
        %      str = commandstr(commandid) returns a string indicating the
        %      purpose of the command with the specified 'commandid'.

        assert(isnumeric(commandid) && isscalar(commandid),...
            'Treadmill:InvalidCommandID',...
            'Command ID must be numeric scalar');
        
        switch commandid
            case 016; str = 'reset';
            case 017; str = 'return device ID';
            case 018; str = 'set group address';
            case 019; str = 'set baud rate';
            case 020; str = 'return verbose ID';
            case 022; str = 'write EEPROM memory';
            case 023; str = 'read EEPROM memory';
            case 024; str = 'retransmit';
            case 025; str = 'echo data';
            case 034; str = 'return treadmill data';
            case 035; str = 'reset odometer';
            case 048; str = 'select interface';
            case 049; str = 'set speed (and acceleration)';
            case 050; str = 'set shock disable (and rate)';
            case 065; str = 'set acceleration parameters';
            case 066; str = 'return acceleration parameters';
            case 137; str = 'copy speed to DAC';
            otherwise;str = 'unknown command';
        end
    end
end

end
        
% +================================================+
% |                                                |
% | Documentation Provided by Columbus Instruments |
% | Contact: Ken Porter <k.porter@colinst.com>     |
% |                                                |
% +================================================+

%{

CI-Bus Serial Interface Protocol
================================

The serial port should be initialized at 19,200 baud, 8 bits, no parity
and 1 stop bit (19200,8,N,1). Packets of information are sent and
received with 4 fields as follows:

(Device ID)(Data ... Data)(Checksum)(Dead-time)

1. The Device ID or Destination Address field (1 byte).
2. The Data field (Typically: Command + Arguments).
3. The Checksum field (1 byte).
4. Dead-Time: 2 or more milliseconds.

The Device ID (DID) field contains 1 byte that determines which
device(s) should receive the packet.

Address     Description
000         Null address - all devices ignore packet.
001–239     Individual device address - only one device accepts packet.
240–254     Programmable group addresses - a group accepts the packet.
255         Broadcast address - all devices accept the packet.

The Data field contains the data of the packet. When sent by the host,
this field typically consists of a command byte followed by arguments
required for the command. When sent by the device, this field
typically consists of the requested data or response only.

The Checksum (CHK) field contains the last byte of the packet which is
equal to the sum of all preceding bytes, modulo 256. This is used to
ensure that the packet is not corrupt.


Notes on Group Addresses
========================

Using group addresses allows the host PC to send the same packet to
multiple devices at the same time. This can be useful to start, stop
and sample data from all devices/channels at the same time during an
experiment. Likewise, loading experiment parameters that are all
common to groups or all devices participating in the experiment. As
more than one device may accept the packet, NO response is returned.

• When a packet is sent to an individual address, the corresponding
device will load it’s serial buffer with the requested data and
immediately send it to the host.

• When a packet is sent to a group address, all corresponding devices
will load their serial buffers with the requested data but NOT send
it to the host. After a group command has been sent, the host can
use the “Retransmit” command to retrieve the response from each
device of the group. This would allow the host to verify that the
group command was received and acknowledged by all.

• Upon power-up or after reception of a “CI-Bus Reset” command, each
device will NOT have a group address. The host must initialize each
device with a specific group address if needed.


Notes on Command Formats
========================

The following pages detail common CI-Bus commands which all devices may
implement but not change. Entire packets for each command will be
displayed. The first byte of the packet represents the device ID. The
data field will contain the command and required arguments that are
BOLD lettered. The last byte is the checksum byte. If the bytes in
any of the fields are known, then they will contain the actual decimal
number. Else:

• “DID” represents the device ID.
• “CHK” represents the calculated checksum.

All commands can be used with any type of address unless specified
otherwise.

After a device powers-up or resets for any reason, ONLY the “CI-Bus
Reset” command is expected. After the “CI-Bus Reset” command has been
accepted, the next command, “Return Device ID”, will also be accepted.
After each device has received these two commands, all other commands
will be accepted.


Treadmill Controller Commands
=============================
016 – Bus Reset

    Can be used with Broadcast, Group or Individual Address.
    Restarts the firmware in the target device.
        ‘016’ is the Bus Reset command.

       send: DID, 016, CHK.
    receive: (nothing)

017 - Return Device ID

    Can be used with Broadcast, Group or Individual Address.
    Returns the device ID ASCII string. The first 4 bytes are the product
    number (XXXX). The next 4 bytes are the firmware version number
    (##.##).
        ‘017’ is the Return Device ID command.
        “XXXX####” should be returned.

       send: DID, 017, CHK.
    receive: DID, X, X, X, X, #, #, #, #, CHK.

018 - Set Group Address

    Can be used with Individual Address only.
    This command is used to assign a group address (240-254) to a device.
    Addresses 000 – 239, 255 will remove the group address.
        ‘018’ is the Set Group Address command.
        ‘GRP’ is the group address.

       send: DID, 018, GRP, CHK.
    receive: DID, GRP, CHK.

019 - Set Baud Rate

    Can be used with Broadcast Address only.
    The bit rates (BRS) are defined as follows:
    000 - 19,200 bps
    001 - 38,400 bps
    002 - 57,600 bps
    003 - 115,200 bps
    004 - 128,000 bps
    005 - 230,400 bps

        ‘019’ is the Set Baud Rate command.
        ‘BRS’ is the baud rate selector (000-005).

       send: 255, 019, BRS, CHK.
    receive: (nothing)

020 - Return Verbose ID

    Can be used with Broadcast, Group or Individual Address.
    Returns the Verbose ID string.
        ‘020’ is the Return Verbose ID command.
        ‘ASC’ are ACSII data bytes.

       send: DID, 020, CHK.
    receive: DID, ASC, ASC, ASC, ..., ASC, CHK.

022 – Write EEPROM Memory

    Can be used with Group or Individual Address.
    Writes up to 75 bytes into the EEPROM at one time.
        ‘022’ is the Write EEPROM command.
        ‘ADR’ is the starting address.
        ‘DAT’ are data bytes to write to the EEPROM.
        ‘006’ is ‘Acknowledge’.

       send: DID, 022, ADR, DAT, (DAT, DAT, …), CHK.
    receive: DID, 006, CHK.

023 – Read EEPROM Memory

    Can be used with Group or Individual Address.
    Reads up to 67 bytes from the EEPROM at one time.
        ‘023’ is the Write EEPROM command.
        ‘ADR’ is the starting address.
        ‘NUM’ is the number of bytes to read from the EEPROM.
        ‘DB1’ is the first data byte.
        ‘DBN’ is the Nth data byte.

       send: DID, 023, ADR, (NUM), CHK.
    receive: DID, DB1, (DB2, DB3, ...), CHK.

024 – Retransmit

    Can be used with Individual Address only.
    Resends the previously transmitted packet.
        ‘024’ is the Retransmit command.
        ‘DB1’ is the first data byte.
        ‘DBN’ is the Nth data byte.

       send: DID, 024, CHK.
    receive: DID, DB1, ..., DBN, CHK.

025 – Echo Data

    Can be used with Broadcast, Group or Individual Address.
    Returns the data bytes that were sent with this command.
        ‘025’ is the Echo Data command.
        ‘DB1’ is the first data byte.
        ‘DBN’ is the Nth data byte.

       send: DID, 025, DB1, ..., DBN, CHK.
    receive: DID, DB1, ..., DBN, CHK.

034 – Return Treadmill Data

    Can be used with Group or Individual Address.
    Returns the Set Speed, Belt Mode, Shock Rate and Odometer.
    ‘034’ is the Return Treadmill Data command.
    ‘SPH:SPL’ is the speed, big-endian (high:low byte).
    ‘BNS’ is the belt mode and shock rate.
    ‘ODL:ODH’ is the odometer, little-endian (low:high byte).

       send: DID, 034, CHK.
    receive: DID, SPH, SPL, 0x00, BNS, ODL, ODH, CHK.

    The bits of the ‘BNS’ are as follows:

      7   6   5   4   3   2   1   0
    +---+---+---+---+---+---+---+---+
    | 0 | 0 | X | X | 0 | 0 | X | X |
    +---+---+---+---+---+---+---+---+
             \_____/         \_____/
                |               |
                |               +-> ‘0 0’ = Run
                |                   ‘0 1’ = Stopped
                |                   ‘1 0’ = Acceleration
                |                   ‘1 1’ = -
                +-----------------> ‘0 0’ = 1Hz Shock Rate
                                    ‘0 1’ = 3Hz Shock Rate
                                    ‘1 0’ = 2Hz Shock Rate
                                    ‘1 1’ = -

035 – Reset Odometer

    Can be used with Group or Individual Address.
    Resets the Odometer to zero.
        ‘035’ is the Reset Odometer command.
        ‘006’ is ‘Acknowledge’.

       send: DID, 035, CHK.
    receive: DID, 006, CHK.

048 – Select Interface

    Can be used with Group or Individual Address.
    Selects which interface controls the treadmill. The “User Interface”
    (000) is default upon power-up, after an odometer reset by front panel
    switch or after reception of the CI-Bus reset command (016).
    Interfaces outside the range of (000-001) are ignored.
    000 – User Interface, front panel controls enabled.
    001 – Computer Interface, software controls enabled.

        ‘048’ is the Select Interface command.
        ‘IFS’ is the interface selector.
        ‘006’ is ‘Acknowledge’.

       send: DID, 048, IFS, CHK
    receive: DID, 006, CHK

049 – Set Speed (and Acceleration)

    Can be used with Group or Individual Address.
    Sets the speed (and optionally the acceleration) and if enabled by the
    treadmill belt switch on the front panel (RUN or ACCEL) will
    immediately apply the settings.
        ‘049’ is the Set Speed command.
    	‘SPH:SPL’ is the speed, big-endian (high-byte:low-byte).
    	‘ASP’ is acceleration step size (in .1m/min).
        ‘AIN’ is acceleration interval (in seconds).
        ‘006’ is ‘Acknowledge’.

       send: DID, 049, SPH, SPL, (ASP, AIN,) CHK
    receive: DID, 006, CHK

050 – Set Shock Disable (and Rate)

    Can be used with Group or Individual Address.
    Used to disable/enable the shock (and optionally the repetition rate).
    The shock disable range is 000 (on) or 001 (off)
    The rate range is (025-250) x 10milliseconds.
        ‘050’ is the Set Shock Disable command.
        ‘SDB’ is the shock disable (0x00, 0x01).
        ‘SRT’ is the shock rate (25-250) x 10ms.
        ‘006’ is ‘Acknowledge’.

       send: DID, 050, SDB, (SRT,) CHK
    receive: DID, 006, CK

065 – Set Acceleration Parameters

    Can be used with Group or Individual Address.
    Writes the acceleration parameters to the EEPROM.
        ‘065’ is the Set Acceleration Parameters command.
        ‘ASP’ is acceleration step size (in .1m/min).
        ‘AIN’ is acceleration interval (in seconds).
        ‘006’ is ‘Acknowledge’.

       send: DID, 065, ASP, AIN, CHK
    receive: DID, 006, CHK

066 – Return Acceleration Parameters

    Can be used with Group or Individual Address.
    Returns the acceleration parameters currently ready or in use.
        ‘066’ is the Return Acceleration Parameters command.
        ‘ASP’ is acceleration step size (in .1m/min).
        ‘AIN’ is acceleration interval (in seconds).

       send: DID, 066, CHK
    receive: DID, ASP, AIN, CHK

137 – Copy Speed to DAC

    Can be used with Group or Individual Address.
    This command does nothing but is reserved because of original code...
        ‘137’ is the Copy Speed to DAC command
        ‘006’ is ‘Acknowledge’.

       send: DID, 137, CHK
    receive: DID, 006, CHK


EEPROM Memory Map
=================

Address     Description
016         Device ID
017         Group ID
032         Acceleration Step
033         Acceleration Interval
034         Odometer Update Rate (calculated during calibration)
035         \ Maximum Speed [L:H]
036         /
037         \ Odometer Distance Traveled per Update Rate [L:H]
038         / (calculated during calibration)


Further Information
===================

The Treadmill Controller has a default CI-Bus address of 220. You can use
the calibration menu of the Treadmill Controller to verify or change the
CI-Bus address anywhere within the range of (1-239).

Use the following steps to properly initialize and control the Treadmill
Controller:
 
1. Initialize the serial port, 19200, 8, N, 1. Be sure to disable
software flow control. The treadmill controller loops-back the output
line: DTR to the input line DSR. At this point you may wish to use
software to verify the loop-back connection. If the test fails, then the
wrong serial port has been selected or the treadmill controller is not
connected.
 
2. Send the "CI-Bus Reset" command using the broadcast address: <255, 016,
015> and wait at least 100ms (for the controller to reset and load default
settings).
 
3. Send the "Return ID" command using the controller's individual address:
<220, 017, 237>. The controller should respond immediately.
 
4. Set the maximum speed to 102.3m/m using the "Write EEPROM" command:
<220, 022, 035, 255, 003, 023>. The values in the EEPROM limit the maximum
speed.
 
5. Send the "Select Interface" command: <220, 048, 001, 013>. This should
select the "computer" interface. The current manual settings are copied
into the computer's settings. The "S" on the far left of the controller's
LCD screen should switch to "C". All of the controls on the front panel,
except the treadmill belt mode switch, are unresponsive. The treadmill
belt mode switch is left active as a fail-safe or emergency stop switch
which can disable (STOP) the motor if needed. That being said, the switch
must be in "RUN" or "ACCEL" for the computer's speed commands to be
applied.
 
6. Set the speed to zero and acceleration (to maximum) using the "Set
Speed and Acceleration" command: <220, 049, 000, 000, 100, 001, 114>. The
acceleration is saved in the EEPROM. The acceleration (or deceleration) of
the speed is set within the range of (.004 to 100 meters/minute/second) by
the calculated fraction of (<step_size>/<step_time>). If the "Set Speed
and Acceleration" command, only specifies a new speed, then the speed will
change by the current acceleration rate. If the command also specifies the
acceleration rate, then it is saved and becomes current for subsequent
"speed only" commands.
 
Optionally, you may wish to use the "Return Treadmill Data" to check the
selection of the treadmill's belt mode switch and verify that it is not in
the "STOP" position. Therefore you can prompt the user to change the
selection before proceeding as the software cannot override the mode switch
setting.

%}

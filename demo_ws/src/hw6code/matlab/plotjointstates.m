function  [t, p, v, a] = plotjointstates(bagfilename, joint, truedt)
%
%   plotjointstates(bagfilename, joint, truedt)
%
%   Plot the /joint_states topic saved in the bag file.  If
%   'bagfilename' is not given or given as 'latest', use the most
%   recent bag file.  If 'joint' is given and not 'all', plot only the
%   joint given by an index (number) or name (string).  If 'truedt' is
%   given, use the bag file timing vs assuming a constant time step.
%
%   Also return the time/pos data, with each column representing one
%   message.  The vel/acc data are numerically differentiated.
%


%
%   Load the bag file.
%
% If no bagfile is specified, use the most recent.
if (~exist('bagfilename') || strcmp(bagfilename, 'latest'))
    % Get a list of .bag files in the current folder.
    d = dir('*.bag');

    % Make sure we have at least one bag file.
    if (~size(d,1))
        error('Unable to find a bag file');
    end

    % Find the most recently modified file
    [~, idx] = max([d.datenum]);

    % Use as the bag file.
    bagfilename = d(idx).name;
    disp(['Using bag file ''' bagfilename '''']);
end

% Load the bag.
try
    bag = rosbag(bagfilename);
catch
    error(['Unable to open the bag file ''' bagfilename '''']);
end


%
%   Extract the data from the bag.
%
%   This creates one column per message (time).
%
% Isolate the /joint_states topic.
msgs = select(bag, 'Topic', '/joint_states');
if (~msgs.NumMessages)
    error(['No /joint_states data in bag file ''' bagfilename '''']);
end

% Convert the messages into structure array.
data = cell2mat(readMessages(msgs, 'DataFormat', 'struct'));

% Double-check the type.
if (~strcmp(data(1).MessageType, 'sensor_msgs/JointState'))
    error(['The /joint_states data is not of type sensor_msgs/JointState']);
end

% Extract the names of the joints and double-check.
names = data(1).Name;
if (length(names) == 0)
    error(['The /joint_states data contains no joints']);
end

% Extract the time (converting from sec/nsec).
headers = [data(:).Header];
stamps  = [headers(:).Stamp];

sec  = double([stamps(:).Sec]);
nsec = double([stamps(:).Nsec]);

t = (sec - sec(1)) + 1e-9 * (nsec - nsec(1));

% Extract the data.
pos = double([data(:).Position]);
vel = double([data(:).Velocity]);
eff = double([data(:).Effort]);


%
%   Check the data content
%
Nnam = length(names);
Npos = size(pos, 1);
Nvel = size(vel, 1);
Neff = size(eff, 1);
Nmax = max([Nnam, Npos, Nvel, Neff]);

if (~Npos)
    warning('No position channels!');
elseif (Npos < Nnam)
    warning('Fewer position channels than names!');
elseif (Npos > Nnam)
    warning('More position channels than names!');
end

if 0,
    if (~Nvel)
        warning('No velocity channels!');
    elseif (Nvel < Nnam)
        warning('Fewer velocity channels than names!');
    elseif (Nvel > Nnam)
        warning('More velocity channels than names!');
    end

    if (~Neff)
        warning('No effort channels!');
    elseif (Neff < Nnam)
        warning('Fewer effort channels than names!');
    elseif (Neff > Nnam)
        warning('More effort channels than names!');
    end
end


%
%   Potentially isolate a single joint.
%
if (exist('joint') && (~strcmp(joint, 'all')))
    % For a numeric joint specification.
    if (isnumeric(joint))
        if (numel(joint) ~= 1)
            error('Bad joint index specification');
        end
            
        % Grab the joint number and use as index.
        ind = floor(joint);
        if ((ind < 1) || (ind > Nmax))
            error(['Out of range joint index ' num2str(ind)]);
        end
        disp(['Using only joint index ' num2str(ind)]);

    % For a joint name specification.
    elseif (ischar(joint))
        % Find the index for the joint name.
        ind = find(strcmp(names, joint));
        if (isempty(ind))
            error(['Unable to isolate joint ''' joint ''''])
        end
        disp(['Using only joint ''' joint '''']);

    % Otherwise can't do anything.
    else
        error('Bad joint argument');
    end
    
    % Isolate the data.
    if (ind <= Nnam) names = names(ind); else names = []; end
    if (ind <= Npos) pos   = pos(ind,:); else pos   = []; end
    if (ind <= Nvel) vel   = vel(ind,:); else vel   = []; end
    if (ind <= Neff) eff   = eff(ind,:); else eff   = []; end
end


%
%   Numerically differentiate.
%
%   This ignores any velocity from the bagfile!
%
% Decide whether to use the true time or a constant step.
if (~exist('truedt'))
    t = linspace(t(1), t(end), length(t));
end

% Check the dimensions.
dofs    = size(pos, 1);
samples = size(pos, 2);
if (samples < 3)
    error('The data contains insufficient samples')
elseif (dofs== 0)
    error('The data contains no position information')
end

% Grab the position only.
p = pos;

% Find the mid-sample times and velocities.
tmid = 0.5*(t(1:end-1) + t(2:end));
vmid = diff(p,1,2) ./ (ones(dofs,1) * diff(t,1,2));

% Extend the mid-sample times/velocities at beginning and end.
tm = [tmid(1)-(t(2)-t(1))  tmid  tmid(end)+(t(end)-t(end-1))];
vm = [   zeros(dofs,1)     vmid         zeros(dofs,1)       ];

% Average the velocity, to get the velocity at the sample times.
v = 0.5 * (vm(:,1:end-1) + vm(:,2:end));

% Compute the acceleration at the sample tiems.
a = diff(vm,1,2) ./ (ones(dofs,1) * diff(tm,1,2));


%
%   Plot.
%
% Skip if outputing data.
if (nargout)
    disp('Skipping the plot when outputing data.');
    return;
end

figure(gcf);
clf;

ax(1) = subplot(3,1,1);
plot(t, p);
grid on;
ylabel('Position (rad)');

ax(2) = subplot(3,1,2);
plot(tmid, vmid);
grid on;
ylabel('Velocity (rad/sec)');

ax(3) = subplot(3,1,3);
plot(t, a);
grid on;
ylabel('Acceleration (rad/sec^2)');
xlabel('Time (sec)');

linkaxes(ax, 'x');

subplot(3,1,1);
legend(names);
title(bagfilename);

set(gcf, 'Name', 'Joint Data');
set(gcf, 'PaperPosition', [0.2500    0.2500    8.0000   10.5000]);

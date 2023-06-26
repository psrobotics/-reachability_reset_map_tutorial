function x_out = updateStateWithResetMap(obj, u, T, x0, d, params)

% If no state is specified, use current state
if nargin < 4 || isempty(x0)
  x0 = obj.x;
end
% If time horizon is 0, return initial state
if T == 0
  x_out = x0;
  return
end
% Default disturbance
if nargin < 5
  d = [];
end

% Check whether there's disturbance (this is needed since not all vehicle
% classes have dynamics that can handle disturbance)
odeOpts = odeset('Events', @(t,x)sys_reset_event(t,x));

% sim next state
if isempty(d)
  [ts, x] = ode113(@(t,x) obj.dynamics(t, x, u), [0 T], x0, odeOpts);
else
  [~, x] = ode113(@(t,x) obj.dynamics(t, x, u, d), [0 T], x0, odeOpts);
end
% get next state, (last one)
x_1 = x(end, :)';
% check if triggers state reset
[value_end, ~, ~] = sys_reset_event(ts(end), x_1);

if value_end < 0
    fprintf('state_flip\n');
    % Apply state reset
    x_post = zeros(3, 1);
    for i=1:length(params.state_fcn_arr)
        x_post(i) =...
            params.state_fcn_arr{i}(x_t, params.o_params);
    end
    obj.x = x_post;
else
    obj.x = x_1;
end

% check if heading angle goes beyond range -pi~pi
if obj.x(3) > pi
    obj.x(3) = obj.x(3) - 2*pi;
elseif obj.x(3) < -pi
    obj.x(3) = obj.x(3) + 2*pi;
end

x_out = obj.x;
% Update the state, state history, control, and control history
obj.u = u;
obj.xhist = cat(2, obj.xhist, obj.x);
obj.uhist = cat(2, obj.uhist, u);
end

function [value, isterminal, direction] = sys_reset_event(t, x)
% check if the current state triggers the reset event
% if y<0 && heading down
% -1 - triggers, 0 - others
if x(2) < 0 && x(3) < 0
    value = -1;
else
    value = 1;
end
% for multiple reset conditions
% if condition_1 || condition_2 || ...condition_n
%    value = -1;
% else
%    value = 1;
% end
isterminal = 1;
direction = 1;
end
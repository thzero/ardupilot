function [force_gain, fin_radius_m] = rocket_fin_gain(fin, tab)
% ROCKET_FIN_GAIN  One fin's control authority from the fin planform and the
% trailing-edge tab, all in millimetres. The tab is what steers the airframe, so this
% derives the authority from real geometry instead of an assumed number.
%
% Keep the formula in sync with SIM_Rocket recompute_fin_geometry() and
% ork_to_rocket.py derive_fin() -- same physics, three places.
%
% fin (mm):  .root_chord  .tip_chord  .semispan  .sweep  .body_radius
% tab (mm):  .width   flap depth FORWARD from the trailing edge (drives effectiveness)
%            .height  tab length ALONG the trailing edge (spanwise)
%            .root    spanwise distance from the fin root to the tab's inboard end
%            .axis    hinge inset aft of the tab's forward edge (0 = hinge at that edge)
%            .max_deg tab deflection at full command
%
% force_gain    N per Pa per unit command, ONE fin  (-> P.fin_force_gain)
% fin_radius_m  radius of the tab's spanwise centre from the body axis (the spin arm)

mm = 1e-3;
Cr = fin.root_chord*mm;  Ct = fin.tip_chord*mm;  b = fin.semispan*mm;
sweep = fin.sweep*mm;    rb = fin.body_radius*mm; %#ok<NASGU>  % sweep kept for the arm note below

% ---- fin planform lift capability ----
S_fin = 0.5*(Cr + Ct)*b;                  % exposed area of one fin
AR    = 2*b^2 / S_fin;                    % aspect ratio
CLa   = 2*pi*AR / (2 + sqrt(AR^2 + 4));   % Helmbold-Diederich lift slope
Kfb   = 1 + rb/(b + rb);                  % body-fin interference

% ---- tab spanwise extent and centre ----
y0 = tab.root*mm;                         % inboard edge, from the fin root
y1 = y0 + tab.height*mm;                  % outboard edge
yc = 0.5*(y0 + y1);                       % spanwise centre
span_frac = (y1 - y0)/b;                  % fraction of the fin span the tab covers

% ---- moving flap chord as a fraction of the LOCAL fin chord at the tab ----
c_tab = Cr + (Ct - Cr)*(yc/b);            % local chord at the tab centre
flap_chord = max(tab.width - tab.axis, 0)*mm;
E = min(max(flap_chord / c_tab, 0), 1);   % flap-chord fraction
theta = acos(2*E - 1);
tau = 1 - (theta - sin(theta))/pi;        % thin-airfoil trailing-edge flap effectiveness

% ---- authority: fin lift x flap effectiveness x span coverage x deflection ----
force_gain   = S_fin * CLa * Kfb * (tau * 0.85 * span_frac) * deg2rad(tab.max_deg);

% Spin arm: the tab's lift acts at its spanwise centre, out from the body axis.
fin_radius_m = rb + yc;
end

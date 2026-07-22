function rho = rocket_air_density(alt_amsl_m)
% ROCKET_AIR_DENSITY  Air density at altitude, ISA standard atmosphere.
%
% Mirrors what SIM_Rocket gets for free from the SITL base class
% (AP_Baro::get_air_density_for_alt_amsl, SIM_Aircraft.cpp:736). This model
% previously used a fixed sea-level 1.225 kg/m^3, which is ~30% wrong at the
% ~3900 m apogee -- and since fin authority is proportional to density, that
% directly overstated control authority through the whole upper coast.
%
% Troposphere only (below 11 km), which covers any hobby flight.

T0   = 288.15;      % K, sea level standard temperature
P0   = 101325.0;    % Pa, sea level standard pressure
L    = 0.0065;      % K/m, temperature lapse rate
R    = 287.053;     % J/(kg K), specific gas constant for dry air
g    = 9.80665;     % m/s^2

alt = min(max(alt_amsl_m, -500), 11000);

T = T0 - L*alt;
P = P0 * (T/T0).^(g/(L*R));
rho = P ./ (R*T);
end

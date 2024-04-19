# manx_test_1.py
#
# Created: Feb 2024, O.N Afify
# Modified: ------

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units
import sys,os
import pylab as plt
import time


from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():

    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()
    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plot results
    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'manx'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------
    # mass properties
    vehicle.mass_properties.takeoff           = 3.23 * Units.kg
    vehicle.mass_properties.operating_empty   = 3.23 * Units.kg
    vehicle.mass_properties.max_takeoff       = 3.23 * Units.kg

    # basic parameters
    vehicle.reference_area                    = 0.594 * Units.meter**2
    vehicle.envelope.maximum_dynamic_pressure = 0.5*1.225*(12**2.) #Max q

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = 3. * Units.meter
    wing.aspect_ratio            = (wing.spans.projected**2)/wing.areas.reference
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.014
    wing.taper                   = 0.48 # 0.6
    wing.dynamic_pressure_ratio  = 1.0
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.twists.root             = 0. * Units.degrees
    wing.twists.tip              = -1.5 * Units.degrees
    wing.number_ribs             = 32.
    wing.number_end_ribs         = 2.
    wing.high_lift               = False
    wing.highlift                = False
    wing.vertical                = False
    wing.symmetric               = True
    wing.origin                  = [[0.12,0.0,0.0]] # meters from nose
    wing.aerodynamic_center      = [0.075,0.0,0.0] # meters from nose


    print('Croot, Ctip: ',wing.chords.root,wing.chords.tip)

    # Wing Segments
    segment                             = SUAVE.Components.Wings.Segment()
    segment.tag                         = 'Root'
    segment.percent_span_location       = 0.0
    segment.twist                       = 1.5 * Units.degrees
    segment.root_chord_percent          = 1.
    segment.thickness_to_chord          = 0.014
    segment.sweeps.quarter_chord        = 0.
    wing.append_segment(segment)

    segment                             = SUAVE.Components.Wings.Segment()
    segment.tag                         = 'Break1'
    segment.percent_span_location       = 0.083
    segment.twist                       = 0. * Units.degrees
    segment.root_chord_percent          = 1.
    segment.thickness_to_chord          = 0.014
    segment.sweeps.quarter_chord        = 0.
    wing.append_segment(segment)

    segment                             = SUAVE.Components.Wings.Segment()
    segment.tag                         = 'Break2'
    segment.percent_span_location       = 0.43
    segment.twist                       = 0. * Units.degrees
    segment.root_chord_percent          = 0.94
    segment.thickness_to_chord          = 0.015
    segment.sweeps.quarter_chord        = 0.
    wing.append_segment(segment)

    segment                             = SUAVE.Components.Wings.Segment()
    segment.tag                         = 'Break3'
    segment.percent_span_location       = 0.8
    segment.twist                       = 0. * Units.degrees
    segment.root_chord_percent          = 0.65
    segment.thickness_to_chord          = 0.021
    segment.sweeps.quarter_chord        = 0.
    wing.append_segment(segment)

    segment                             = SUAVE.Components.Wings.Segment()
    segment.tag                         = 'Tip'
    segment.percent_span_location       = 1.
    segment.twist                       = -1.5 * Units.degrees
    segment.root_chord_percent          = 0.48
    segment.thickness_to_chord          = 0.029
    segment.sweeps.quarter_chord        = 0.
    wing.append_segment(segment)

    # Fills out more segment properties automatically
    wing = segment_properties(wing)
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)


    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.87
    wing.sweeps.quarter_chord    = 0 * Units.deg
    wing.thickness_to_chord      = 0.027
    wing.taper                   = 1.0
    wing.areas.reference         = 0.095 * Units.meter
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 0.8 * wing.areas.wetted
    wing.areas.affected          = 0.6 * wing.areas.wetted
    wing.spans.projected         = np.sqrt(wing.aspect_ratio*wing.areas.reference)
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.vertical                = False
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9
    wing.number_ribs             = 11
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.origin                  = [[1.6,0.0,0.02]] # meters from nose
    wing.aerodynamic_center      = [0.05,0.0,0.0] # meters from nose
    print('Croot, Ctip: ',wing.chords.root,wing.chords.tip)
    #sys.exit()

    # add to vehicle
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'


    wing.aspect_ratio            = 1.458
    wing.sweeps.quarter_chord    = 0 * Units.deg
    wing.thickness_to_chord      = 0.038
    wing.taper                   = 1.0
    wing.areas.reference         = 0.032 * Units.meter
    wing.spans.projected         = np.sqrt(wing.aspect_ratio*wing.areas.reference)
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 0.8 * wing.areas.wetted
    wing.areas.affected          = 0.6 * wing.areas.wetted
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.origin                  = [[1.45,0.01,0.0]] # meters from nose
    wing.aerodynamic_center      = [0.0325,0.0,0.0] # meters from nose
    wing.symmetric               = True
    wing.vertical                = True
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
    wing.number_ribs             = 7.

    # add to vehicle
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing)

    vehicle.append_component(wing)


    #------------------------------------------------------------------
    # Propulsor - Battery_Propeller
    #------------------------------------------------------------------

    # build network
    net = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines = 2.
    net.voltage                     = 11.1
    net.identical_propellers        = True

    # Component 1 The ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 2 The Propeller
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades    = 2.0
    prop.freestream_velocity = 12. * Units['m/s']# freestream 10.12
    prop.angular_velocity    = 5000 * Units['rpm'] # 9X6 APC Prop ;1984.45, 548.63, 2553
    prop.tip_radius          = 4.5 * Units.inches
    prop.hub_radius          = 0.4 * Units.inches
    prop.design_Cl           = 0.7
    prop.design_altitude     = 0.12 * Units.km
    prop.design_power        = None
    prop.design_thrust       = 2. # CHECK CHANGED 12
    prop                     = propeller_design(prop)

    origins                  = [[0.12, 0.45, 0.], [0.12, -0.45, 0.]] # From the nose

    #for ii in range(2):
        #rotor          = deepcopy(prop)
        #rotor.tag      = 'propeller'
        #rotor.origin   = [origins[ii]]
    #net.propellers.append(rotor)
    net.propellers.append(prop) # Check if propeller polars are necessary

    # Component 3 the Motor
    motor = SUAVE.Components.Energy.Converters.Motor()
    motor.resistance           = 0.0075
    motor.no_load_current      = 0.85  * Units.ampere
    motor.speed_constant       = 1150 * Units['rpm/volt'] # RPM/volt converted to (rad/s)/volt change to 41.26 , 82.2
    motor                      = size_from_kv(motor)
    motor.propeller_radius     = prop.tip_radius
    motor.propeller_Cp         = prop.design_power_coefficient
    motor.gear_ratio           = 1. # Gear ratio
    motor.gearbox_efficiency   = 1. # Gear box efficiency
    motor.expected_current     = 25. * Units.ampere # Expected current
    motor.mass_properties.mass = 0.06  * Units.kg

    net.propeller_motors.append(motor)


    # Component 4 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 0. * Units.watts
    payload.mass_properties.mass = 0.0 * Units.kg
    net.payload                  = payload

    # Component 5 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 1. #Watts
    net.avionics        = avionics

    # Component 8 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 1.186 * Units.kg #
    bat.specific_energy      = 187.18 * Units.Wh/Units.kg
    bat.max_voltage          = 11.1
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat

    # add the battery network to the vehicle
    vehicle.append_component(net)

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    configs.append(config)

    return configs

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()

    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_UAV()
    weights.settings.empty = \
        SUAVE.Methods.Weights.Correlations.Human_Powered.empty
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks #what is called throughout the mission (at every time step))
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    # done!
    return analyses


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'The Test Mission'

    mission.atmosphere  = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.planet      = SUAVE.Attributes.Planets.Earth()

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery

    # ------------------------------------------------------------------
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "Cruise"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)

    # segment attributes
    segment.state.numerics.number_control_points = 32 # 4, 8 or 16.
    segment.start_time     = time.strptime("Tue, May 02 11:30:00  2023", "%a, %b %d %H:%M:%S %Y",) # Manx first flight.

    segment.battery_energy = vehicle.networks.battery_propeller.battery.max_energy*0.3
    segment.mach           = 0.03
    segment.distance       = 3.  * Units.km
    segment.air_speed      = 12. * Units['m/s']
    segment.altitude       = 0.12 * Units.km

    segment.latitude       = 51.   # do not use Units.degrees
    segment.longitude      = 0. # this defaults to degrees



    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.08)


    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    # done!
    return missions

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_mission(results):

    # Plot Flight Conditions
    plot_flight_conditions(results)

    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)

    # Drag Components
    plot_drag_components(results)

    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results)

    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results)

    plot_battery_cell_conditions(results)

    # Plot Propeller Conditions
    plot_propeller_conditions(results)

    # Plot Electric Motor and Propeller Efficiencies
    plot_eMotor_Prop_efficiencies(results)

    return

if __name__ == '__main__':
    main()

    plt.show()

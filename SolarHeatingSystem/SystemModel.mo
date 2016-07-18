within SolarHeatingSystem;
model SystemModel
  "Solar heating system as an optimization example"
  extends Modelica.Icons.Example;
  parameter Real UValueWindows = 1.0;
  parameter Real UValueFloor = 0.35;
  parameter Real UValueRoof = 0.34;
  parameter Real UValueWalls = 1.0/(1.0/25.0+0.2/1.35+thicknessInsulation/0.04+1.0/7.692);
  // Optimization parameters
  parameter Modelica.SIunits.Volume VStorage(min = 1.0, max = 40.0) = 10.0;
  parameter Modelica.SIunits.Area ACollector(min = 4.0, max = 40.0) = 20.0;
  parameter Modelica.SIunits.Length thicknessInsulation(min = 0.06, max = 0.30) = 0.1;
  // Optimization parameters
  parameter Real lifetimeCollector(unit = "a") = 20.0;
  parameter Real costCollector(unit = "Euro/a") = 200.0*ACollector/lifetimeCollector;
  parameter Real lifetimeStorage(unit = "a") = 20.0;
  parameter Real costStorage(unit = "Euro/a") = (1649.81*VStorage^(-0.464))/lifetimeStorage;
  parameter Real energyPrice(unit = "Euro/kWh") = 0.08;
  Real costHeaterEnergy(unit = "Euro/a") = energyPrice*QHeater/3600.0/1000.0;
  parameter Real lifetimeInsulation(unit = "a") = 30.0;
  parameter Real costInsulation(unit = "Euro/a") = sum(building.AExt)*(2.431*thicknessInsulation*100.0+87.35)/lifetimeInsulation
    annotation(Documentation(info="<html><p>BMVBS-Online-Publikation, Nr. 07/2012 Kosten energierelevanter Bau- und Anlagenteile bei der energetischen Modernisierung von WohngebÃÂ¤uden</p>/html>"));
  parameter Real penaltyFactor(unit = "Euro/a") = 1000.0;
  Modelica.SIunits.Energy QHeater(start = 0.0);
  Modelica.SIunits.Energy QRadiator(start = 0.0001);
  Real solarfraction(unit = "-");
  Real costfunction(unit = "Euro/a");

  package Medium1 = Annex60.Media.Water (
    T_min = 273.15 - 40.0,
    T_max = 273.15 + 300.0);
  package Medium2 = Modelica.Media.Air.SimpleAir (
    T_min = 273.15 - 40.0,
    T_max = 273.15 + 100.0);
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal= 0.1;

  Annex60.Fluid.Storage.ExpansionVessel exp1(
    redeclare package Medium = Medium1, V_start=0.1)
    "Expansion vessel model"
    annotation (Placement(transformation(extent={{22,-54},{34,-42}})));
  SolarHeatingSystem.Components.Pipe pipe1(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1,
    diameter=0.02)
    "Pipe model"
    annotation (Placement(transformation(extent={{12,-2},{32,-22}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T heater(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=10.0,
    Q_flow_maxCool=0.0)
    "Boiler model"
    annotation (Placement(transformation(extent={{8,-70},{-12,-50}})));
  Annex60.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 radiator(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    Q_flow_nominal=10000.0,
    VWat=0.005,
    mDry=0.0001,
    nEle=5,
    fraRad=0,
    TAir_nominal=273.15 + 20.0,
    n=1.3,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=273.15 + 45.0,
    T_b_nominal=273.15 + 30.0)
    "Radiator model"
    annotation (Placement(transformation(extent={{-12,-22},{8,-2}})));
  SolarHeatingSystem.Components.Pipe pipe2(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1)
    "Pipe model"
    annotation (Placement(transformation(extent={{-16,-70},{-36,-50}})));
  Modelica.Blocks.Sources.Constant TSet(
    k=273.15 + 45.0)
    annotation (Placement(transformation(extent={{18,-56},{14,-52}})));
  SolarHeatingSystem.Components.Pipe pipe3(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    diameter=0.02,
    length=1)
    "Pipe model"
    annotation (Placement(transformation(extent={{-60,-2},{-40,-22}})));
  Annex60.Fluid.Actuators.Valves.Data.Generic datVal(
    y={0,0.1667,0.3333,0.5,0.6667,1},
    phi={0, 0.19, 0.35, 0.45, 0.5, 0.65}/0.65)
    "Valve characteristics"
    annotation (Placement(transformation(extent={{-60,8},{-40,28}})));
  Annex60.Fluid.Actuators.Valves.TwoWayTable valve(
    redeclare package Medium = Medium1,
    filteredOpening=false,
    from_dp=true,
    flowCharacteristics=datVal,
    CvData=Annex60.Fluid.Types.CvTypes.Kv,
    Kv=0.65,
    m_flow_nominal=m_flow_nominal)
    "Valve model with opening characteristics based on a table"
    annotation (Placement(transformation(extent={{-36,-22},{-16,-2}})));
  Modelica.Blocks.Continuous.LimPID thermostat(
    k=0.5,
    controllerType=Modelica.Blocks.Types.SimpleController.P,
    yMax=1.0,
    yMin=0.0,
    wp=1.0)
    "Thermostat, modelled by a limeted p-controller"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},rotation=90,origin={-26,8})));
  Annex60.Fluid.Movers.FlowControlled_dp pump1(
    redeclare package Medium =Medium1, m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-84,-22},{-64,-2}})));
  Modelica.Blocks.Sources.Constant dpSet(
    k=12000.0)
    "Set presure for the pump model"
    annotation (Placement(transformation(extent={{-68,6},{-72,10}})));
  Modelica.Blocks.Sources.Constant TAirSet(
    k=273.15 + 20.0)
    annotation (Placement(transformation(extent={{-20,16},{-24,20}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TAmb(
    each T=293.15)
    annotation (Placement(transformation(extent={{-66,-46},{-54,-34}})));
  SolarHeatingSystem.Components.ThermalStorages.FluidStorage storage(
    HX_2=false,
    AdditionalFluidPorts=true,
    nEle=10,
    HX_1=true,
    height = 2.0,
    redeclare SolarHeatingSystem.Components.ThermalStorages.BaseClasses.BuoyancyModels.Buoyancy1 HeatBuoyancy,
    redeclare package Medium = Medium1,
    V=VStorage,
    thickness_ins=0.1,
    thickness_wall=0.003,
    T_start=293.15)
    annotation (Placement(transformation(extent={{38,-34},{58,-14}})));
  SolarHeatingSystem.Components.SolarThermal.ThermalCollector collector(
    redeclare package Medium = Medium1,
    angleDegAzi=0.0,
    angleDegTil=30.0,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=10,
    redeclare SolarHeatingSystem.Components.SolarThermal.Data.Collectors.ComercialsCollectors.FlatPlate.AgenaAZUR8plus_AC28H collectorData,
    height=1,
    width=ACollector,
    T_start=293.15)
    annotation (Placement(transformation(extent={{92,-22},{112,-2}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow pump2(
    redeclare package Medium = Medium1,
    m_flow_nominal=ACollector*40.0/3600.0)
    annotation (Placement(transformation(extent={{84,-70},{64,-50}})));
  SolarHeatingSystem.Components.Pipe pipe4(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1,
    diameter=0.02)
    "Pipe model"
    annotation (Placement(transformation(extent={{62,-2},{82,-22}})));
  SolarHeatingSystem.Components.Pipe pipe5(
    redeclare package Medium = Medium1,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1,
    diameter=0.02)
    "Pipe model"
    annotation (Placement(transformation(extent={{88,-50},{108,-70}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{84,10},{92,18}})));
  Modelica.Blocks.Sources.Constant colOn(k=ACollector*40.0/3600.0)
    annotation (Placement(transformation(extent={{92,-32},{88,-28}})));
  Annex60.Fluid.Storage.ExpansionVessel exp2(
    redeclare package Medium = Medium1,
    V_start=0.1)
    "Expansion vessel model"
    annotation (Placement(transformation(extent={{48,-54},{60,-42}})));
  Modelica.Blocks.Logical.Hysteresis control(
    uLow=1,
    uHigh=4)
    annotation (Placement(transformation(extent={{116,-40},{108,-32}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{84,-40},{76,-32}})));
  Modelica.Blocks.Sources.Constant ColOff(
    k=0.0)
    annotation (Placement(transformation(extent={{92,-44},{88,-40}})));
  Modelica.Blocks.Math.Add add(
    k1=1,
    k2=-1)
    annotation (Placement(transformation(extent={{128,-40},{120,-32}})));
  Annex60.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTilColl(
    each til=0.5235987755983,
    each lat=0.87266462599716,
    azi=0)
    "Calculates direct solar radiation on titled solr collector"
    annotation (Placement(transformation(extent={{14,30},{26,42}})));
  Annex60.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTilColl(
    each outSkyCon=true,
    each outGroCon=true,
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi=0)
    "Calculates diffuse solar radiation on titled surface of the solar collector"
    annotation (Placement(transformation(extent={{-16,48},{-4,60}})));
  Annex60.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
    calTSky=Annex60.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    filNam= "modelica://Annex60/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos",
    computeWetBulbTemperature=false)
    "Weather data reader"
    annotation (Placement(transformation(extent={{-96,180},{-76,200}})));
    //filNam="modelica://Annex60/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos",
  Annex60.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil[4](
    each outSkyCon=true,
    each outGroCon=true,
    til={1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966},
    each  lat=0.87266462599716,
    azi={3.141592653589793, -1.5707963267948966, 0.0, 1.5707963267948966})
    "Calculates diffuse solar radiation on titled surface for all directions"
    annotation (Placement(transformation(extent={{-44,126},{-24,146}})));
  Annex60.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil[4](
    til={1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966},
    each  lat=0.87266462599716,
    azi={3.141592653589793, -1.5707963267948966, 0.0, 1.5707963267948966})
    "Calculates direct solar radiation on titled surface for all directions"
    annotation (Placement(transformation(extent={{-44,180},{-24,200}})));
  Annex60.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane corGDoublePane(n=4, UWin=0.9813955738135063)
    "Correction factor for solar transmission"
    annotation (Placement(transformation(extent={{30,182},{50,202}})));
  Annex60.ThermalZones.ReducedOrder.RC.FourElements building(
    redeclare package Medium = Medium2,
    VAir=336.0,
    alphaExt=21.7,
    alphaWin=1.6999999999999997,
    gWin=0.6,
    ratioWinConRad=0.03,
    nExt=1,
    RExt={0.000221375084495*0.35/UValueWalls},
    CExt={86200772.7108},
    alphaRad=7.947818768410714,
    AInt=318.48,
    alphaInt=6.328522984174829,
    nInt=1,
    RInt={0.00028734526051},
    CInt={43260210.4851},
    RWin=0.0258840291977*1.0/UValueWalls,
    RExtRem=0.0123709459756*0.35/UValueWalls,
    AFloor=73.44,
    alphaFloor=1.7,
    nFloor=1,
    RFloor={0.000657855370917*0.35/UValueFloor},
    RFloorRem=0.0367624625505*0.35/UValueFloor,
    CFloor={29007444.7758},
    ARoof=73.44,
    alphaRoof=1.7,
    nRoof=1,
    RRoof={0.000660213665372*0.34/UValueRoof},
    RRoofRem=0.0387053267895*0.34/UValueRoof,
    CRoof={29132420.995},
    nOrientations=4,
    AWin={5.6,5.6,16.8,5.6},
    ATransparent={5.6,5.6,16.8,5.6},
    AExt={66.96,42.16,66.96,42.16},
    nPorts=2)
    "Building"
    annotation (Placement(transformation(extent={{68,120},{116,156}})));
  Annex60.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow eqAirTemp(
    n=4,
    wfGro=0.0,
    wfWall={0.30681818181818177, 0.1931818181818182, 0.30681818181818177, 0.1931818181818182},
    wfWin={0.16666666666666669, 0.16666666666666669, 0.5000000000000001, 0.16666666666666669},
    withLongwave=true,
    aExt=0.7,
    alphaWallOut=5.0,
    alphaRadWall=5.0,
    alphaWinOut=20.0,
    alphaRadWin=5.0,
    aWin=0.0,
    eExt=0.8999999999999999,
    eWin=0.10714285714285714,
    TGro=286.15)
    "Computes equivalent air temperature"
    annotation (Placement(transformation(extent={{0,94},{20,114}})));
  Modelica.Blocks.Math.Add solRad[4]
    "Sums up solar radiation of both directions"
    annotation (Placement(transformation(extent={{-12,132},{-4,140}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature1
    "Prescribed temperature for exterior walls outdoor surface temperature"
    annotation (Placement(transformation(extent={{34,118},{44,128}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature2
    "Prescribed temperature for windows outdoor surface temperature"
    annotation (Placement(transformation(extent={{34,138},{44,148}})));
  Modelica.Thermal.HeatTransfer.Components.Convection thermalConductorWin
    "Outdoor convective heat transfer of windows"
    annotation (Placement(transformation(extent={{62,138},{52,148}})));
  Modelica.Thermal.HeatTransfer.Components.Convection thermalConductorWall
    "Outdoor convective heat transfer of walls"
    annotation (Placement(transformation(extent={{62,128},{52,118}})));
  Modelica.Blocks.Sources.Constant const2[4](
    each k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{2,120},{8,126}})));
  Modelica.Blocks.Sources.Constant alphaWall(
    k=10.0*218.23999999999998)
    "Outdoor coefficient of heat transfer for walls"
    annotation (Placement(transformation(extent={{-3,-3},{3,3}},rotation=90,origin={57,111})));
  Modelica.Blocks.Sources.Constant alphaWin(
    k=24.999999999999996*33.6)
    "Outdoor coefficient of heat transfer for windows"
    annotation (Placement(transformation(extent={{3,-3},{-3,3}},rotation=90,origin={57,155})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow sourcesRad
    "Radiative heat flow of internal sources"
    annotation (Placement(transformation(extent={{72,58},{92,78}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow sourcesConv
    "Convective heat flow of internal sources"
    annotation (Placement(transformation(extent={{72,40},{92,60}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperatureFloor
    "Prescribed temperature for floor plate outdoor surface temperature"
    annotation (Placement(transformation(extent={{-3,-3.5},{3,3.5}},rotation=90,origin={92.5,115})));
  Modelica.Blocks.Sources.Constant TSoil(
    k=286.15)
    "Outdoor surface temperature for floor plate"
    annotation (Placement(transformation(extent={{-3,-3},{3,3}},rotation=180,origin={99,105})));
  Annex60.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007 eqAirTempVDI(
    aExt=0.7,
    eExt=0.9,
    wfGro=0,
    alphaWallOut=5.0,
    alphaRadWall=5.0,
    n=1,
    wfWall={1.0},
    wfWin={0},
    TGro=285.15)
    "Computes equivalent air temperature for roof"
    annotation (Placement(transformation(extent={{28,156},{48,176}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperatureRoof
    "Prescribed temperature for roof outdoor surface temperature"
    annotation (Placement(transformation(extent={{-3,-3},{3,3}},rotation=-90,origin={91,181})));
  Modelica.Thermal.HeatTransfer.Components.Convection thermalConductorRoof
    "Outdoor convective heat transfer of roof"
    annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,origin={91,169})));
  Modelica.Blocks.Sources.Constant alphaRoof(
    k=10.0*73.44)
    "Outdoor coefficient of heat transfer for roof"
    annotation (Placement(transformation(extent={{4,-3.5},{-4,3.5}},rotation=0,origin={104,169.5})));
  Modelica.Blocks.Sources.Constant const3[1](
    each k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{3,-3},{-3,3}},rotation=90,origin={64,199})));
  Modelica.Blocks.Math.Add solRadRoof[1]
    "Sums up solar radiation of both directions"
    annotation (Placement(transformation(extent={{-12,210},{-4,218}})));
  Annex60.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTilRoof[1](
    til={0.0},
    each lat=0.87266462599716,
    azi={3.141592653589793})
    "Calculates direct solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-44,206},{-24,226}})));
  Annex60.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTilRoof[1](
    til={0.0},
    each  lat=0.87266462599716,
    azi={3.141592653589793})
    "Calculates diffuse solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-44,154},{-24,174}})));
  Annex60.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(transformation(extent={{-78,108},{-44,140}}),iconTransformation(extent={{-70,-12},{-50,8}})));
  Modelica.Blocks.Sources.Constant intSou(k=0.0)
    "Outdoor coefficient of heat transfer for roof"
    annotation (Placement(transformation(extent={{-5,-5.5},{5,5.5}},rotation=0,origin={57,59.5})));
  Annex60.Fluid.Sources.MassFlowSource_T ventilationIn(
    use_m_flow_in=true,
    use_T_in=true,
    nPorts=1,
    redeclare package Medium = Medium2)
    "Outside air"
    annotation (Placement(transformation(extent={{54,92},{62,100}})));
  Annex60.Fluid.Sources.MassFlowSource_T ventilationOut(
    use_m_flow_in=true,
    nPorts=1,
    use_T_in=false,
    redeclare package Medium = Medium2)
    "Exhaust air"
    annotation (Placement(transformation(extent={{54,80},{62,88}})));
  Modelica.Blocks.Math.Gain gain(
    k=building.VAir/3600.0*1.2)
    "Conversion to kg/s"
    annotation (Placement(transformation(extent={{38,96},{44,102}})));
  Modelica.Blocks.Math.Gain gain1(
    k=-1)
    "Reverses ventilation rate"
    annotation (Placement(transformation(extent={{46,84},{52,90}})));
  Modelica.Blocks.Sources.Constant airchange(
    k=0.5)
    "Ventilation rate in 1/h"
    annotation (Placement(transformation(extent={{3,-3},{-3,3}},rotation=180,origin={29,99})));
  Modelica.Blocks.Logical.And cutOverheating
    annotation (Placement(transformation(extent={{102,-40},{94,-32}})));
  Modelica.Blocks.Logical.LessThreshold lessThreshold(
    threshold=273.15 + 100.0)
    annotation (Placement(transformation(extent={{116,-52},{108,-44}})));
  Interfaces.GenOptInterface genOptInterface
    annotation (Placement(transformation(extent={{110,200},{130,220}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=costfunction)
    annotation (Placement(transformation(extent={{80,218},{100,238}})));
equation
  der(QRadiator) = - radiator.Q_flow;
  der(QHeater) = heater.Q_flow;
  solarfraction = (QRadiator - QHeater)/ QRadiator;
  costfunction = costCollector + costStorage + costInsulation + costHeaterEnergy + penaltyFactor * (1.0 - solarfraction);

  connect(radiator.port_b, pipe1.port_a) annotation (Line(
      points={{8,-12},{12,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe2.port_a, heater.port_b) annotation (Line(
      points={{-16,-60},{-12,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(heater.TSet, TSet.y) annotation (Line(
      points={{10,-54},{13.8,-54}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pipe3.port_b, valve.port_a) annotation (Line(
      points={{-40,-12},{-36,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(valve.port_b, radiator.port_a) annotation (Line(
      points={{-16,-12},{-12,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump1.port_b, pipe3.port_a) annotation (Line(
      points={{-64,-12},{-60,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe2.port_b, pump1.port_a) annotation (Line(
      points={{-36,-60},{-90,-60},{-90,-12},{-84,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(dpSet.y, pump1.dp_in) annotation (Line(
      points={{-72.2,8},{-74.2,8},{-74.2,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(thermostat.y, valve.y) annotation (Line(
      points={{-26,3.6},{-26,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TAirSet.y,thermostat.u_s) annotation (Line(
      points={{-24.2,18},{-26,18},{-26,12.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TAmb.port, pipe3.heatPort)
    annotation (Line(points={{-54,-40},{-50,-40},{-50,-17}}, color={191,0,0}));
  connect(TAmb.port, pipe1.heatPort) annotation (Line(points={{-54,-40},{-54,-40},
          {22,-40},{22,-17}}, color={191,0,0}));
  connect(TAmb.port, pipe2.heatPort) annotation (Line(points={{-54,-40},{-54,-40},
          {-26,-40},{-26,-55}}, color={191,0,0}));
  connect(pipe1.port_b, storage.port_a[5]) annotation (Line(points={{32,-12},{38,
          -12},{38,-24},{41,-24}}, color={0,127,255}));
  connect(storage.port_b1, heater.port_a) annotation (Line(points={{41,-15},{36,
          -15},{36,-60},{8,-60}}, color={0,127,255}));
  connect(exp1.port_a, heater.port_a)
    annotation (Line(points={{28,-54},{28,-60},{8,-60}}, color={0,127,255}));
  connect(pipe4.port_b, collector.port_a) annotation (Line(points={{82,-12},{92,
          -12}},color={0,127,255}));
  connect(pump2.port_a, pipe5.port_a)
    annotation (Line(points={{84,-60},{86,-60},{88,-60}},
                                                 color={0,127,255}));
  connect(pipe5.port_b, collector.port_b) annotation (Line(points={{108,-60},{134,
          -60},{134,-12},{112,-12}}, color={0,127,255}));
  connect(prescribedTemperature.port, collector.heatPortCon)
    annotation (Line(points={{92,14},{107,14},{107,-3}}, color={191,0,0}));
  connect(pipe5.heatPort, TAmb.port) annotation (Line(points={{98,-65},{98,-78},
          {-50,-78},{-50,-40},{-54,-40}}, color={191,0,0}));
  connect(pipe4.heatPort, TAmb.port) annotation (Line(points={{72,-17},{72,-17},
          {72,-40},{48,-40},{48,-78},{-50,-78},{-50,-40},{-54,-40}}, color={191,0,0}));
  connect(exp2.port_a, pump2.port_b) annotation (Line(points={{54,-54},{54,-60},
          {64,-60}},                                                                       color={0,127,255}));
  connect(switch1.y, pump2.m_flow_in) annotation (Line(points={{75.6,-36},{74.2,
          -36},{74.2,-48}}, color={0,0,127}));
  connect(colOn.y, switch1.u1) annotation (Line(points={{87.8,-30},{84.8,-30},{84.8,
          -32.8}}, color={0,0,127}));
  connect(ColOff.y, switch1.u3) annotation (Line(points={{87.8,-42},{84.8,-42},{
          84.8,-39.2}}, color={0,0,127}));
  connect(collector.TSeg[10], add.u1) annotation (Line(points={{105.2,-21},{130,
          -21},{130,-33.6},{128.8,-33.6}},        color={0,0,127}));
  connect(storage.port_HX_1_b, pump2.port_b) annotation (Line(
      points={{55,-30},{60,-30},{60,-60},{64,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(storage.port_HX_1_a, pipe4.port_a) annotation (Line(
      points={{55,-28},{60,-28},{60,-12},{62,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(HDirTilColl.H, collector.IrrDir) annotation (Line(points={{26.6,36},{62,
          36},{98.3,36},{98.3,-4.3}}, color={0,0,127}));
  connect(HDirTilColl.inc, collector.inc) annotation (Line(points={{26.6,33.6},{
          105.1,33.6},{105.1,-4.3}}, color={0,0,127}));
  connect(HDifTilColl.H, collector.IrrDif) annotation (Line(points={{-3.4,54},{-3.4,
          54},{48,54},{48,32},{101.7,32},{101.7,-4.3}}, color={0,0,127}));
  connect(eqAirTemp.TEqAirWin,prescribedTemperature2.T)
    annotation (Line(points={{21,107.8},{24,107.8},{24,143},{33,143}},color={0,0,127}));
  connect(eqAirTemp.TEqAir, prescribedTemperature1.T) annotation (Line(points={{21,104},
          {28,104},{28,123},{33,123}},color={0,0,127}));
  connect(weaDat.weaBus,weaBus)
    annotation (Line(points={{-76,190},{-50,190},{-50,136},{-60,136},{-60,130},{-61,130},{-61,124}},
    color={255,204,51},thickness=0.5), Text(string="%second",index=1,extent={{6,3},{6,3}}));
  connect(weaBus.TDryBul,eqAirTemp. TDryBul)
    annotation (Line(points={{-61,124},{-61,98},{-2,98}},color={255,204,51},
    thickness=0.5), Text(string="%first",index=-1,extent={{-6,3},{-6,3}}));
  connect(const2.y, eqAirTemp.sunblind) annotation (Line(points={{8.3,123},{10,123},
          {10,116}},color={0,0,127}));
  connect(HDifTil.HSkyDifTil,corGDoublePane.HSkyDifTil)
    annotation (Line(
    points={{-23,142},{-4,142},{18,142},{18,194},{28,194}},color={0,0,127}));
  connect(HDirTil.H,corGDoublePane.HDirTil)
    annotation (Line(points={{-23,190},{28,190},{28,198}},
    color={0,0,127}));
  connect(HDirTil.H,solRad.u1)
    annotation (Line(points={{-23,190},{-18,190},{-18,138.4},{-12.8,138.4}},color={0,0,127}));
  connect(HDirTil.inc,corGDoublePane.inc)
    annotation (Line(points={{-23,186},{2,186},{28,186}},color={0,0,127}));
  connect(HDifTil.H,solRad.u2)
    annotation (Line(points={{-23,136},{-23,133.6},{-12.8,133.6}},color={0,0,127}));
  connect(HDifTil.HGroDifTil,corGDoublePane.HGroDifTil)
    annotation (Line(
    points={{-23,130},{20,130},{20,190},{28,190}},color={0,0,127}));
  connect(solRad.y,eqAirTemp.HSol)
    annotation (Line(points={{-3.6,136},{-2,136},{-2,110}},
    color={0,0,127}));
  connect(weaDat.weaBus,HDifTil[1].weaBus)
  annotation (Line(
  points={{-76,190},{-50,190},{-50,136},{-44,136}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDirTil[1].weaBus)
  annotation (Line(
  points={{-76,190},{-44,190}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDifTil[2].weaBus)
  annotation (Line(
  points={{-76,190},{-50,190},{-50,136},{-44,136}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDirTil[2].weaBus)
  annotation (Line(
  points={{-76,190},{-44,190}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDifTil[3].weaBus)
  annotation (Line(
  points={{-76,190},{-50,190},{-50,136},{-44,136}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDirTil[3].weaBus)
  annotation (Line(
  points={{-76,190},{-44,190}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDifTil[4].weaBus)
  annotation (Line(
  points={{-76,190},{-50,190},{-50,136},{-44,136}},
  color={255,204,51},
  thickness=0.5));
  connect(weaDat.weaBus,HDirTil[4].weaBus)
  annotation (Line(
  points={{-76,190},{-44,190}},
  color={255,204,51},
  thickness=0.5));
  connect(sourcesRad.port, building.intGainsRad) annotation (Line(points={{92,68},
          {108,68},{124,68},{124,146},{116.2,146}}, color={191,0,0}));
  connect(thermalConductorWin.solid, building.window) annotation (Line(points={{
          62,143},{64,143},{64,142},{67.8,142}}, color={191,0,0}));
  connect(prescribedTemperature2.port,thermalConductorWin.fluid)
    annotation (Line(points={{44,143},{52,143}},       color={191,0,0}));
  connect(building.extWall, thermalConductorWall.solid) annotation (Line(points=
         {{67.8,134},{64,134},{64,123},{62,123}}, color={191,0,0}));
  connect(thermalConductorWall.fluid, prescribedTemperature1.port) annotation (
      Line(points={{52,123},{48,123},{44,123}},          color={191,0,0}));
  connect(alphaWall.y,thermalConductorWall.Gc)
    annotation (Line(points={{57,114.3},{57,118}},        color={0,0,127}));
  connect(alphaWin.y,thermalConductorWin.Gc)
    annotation (Line(points={{57,151.7},{57,148}},       color={0,0,127}));
  connect(weaBus.TBlaSky,eqAirTemp.TBlaSky)
    annotation (Line(
    points={{-61,124},{-60,124},{-60,104},{-2,104}},
    color={255,204,51},
    thickness=0.5), Text(
    string="%first",index=-1,
    extent={{-6,3},{-6,3}}));
  connect(sourcesConv.port, building.intGainsConv) annotation (Line(points={{92,50},
          {120,50},{120,142},{116,142}},     color={191,0,0}));
  connect(corGDoublePane.solarRadWinTrans, building.solRad) annotation (Line(
        points={{51,192},{58,192},{64,192},{64,153},{67,153}}, color={0,0,127}));
  connect(prescribedTemperatureFloor.port, building.floor)
    annotation (Line(points={{92.5,118},{92,118},{92,120}}, color={191,0,0}));
  connect(prescribedTemperatureRoof.port,thermalConductorRoof. fluid)
    annotation (Line(points={{91,178},{91,178},{91,174}},
                                                       color={191,0,0}));
  connect(thermalConductorRoof.solid, building.roof) annotation (Line(points={{91,
          164},{90.8,164},{90.8,155}}, color={191,0,0}));
  connect(thermalConductorRoof.Gc,alphaRoof.y)
    annotation (Line(points={{96,169},{96,169.5},{99.6,169.5}},color={0,0,127}));
  connect(eqAirTempVDI.TDryBul,eqAirTemp.TDryBul)
    annotation (Line(points={{26,160},{-22,160},{-22,152},{-72,152},{-72,98},{-2,98}},
    color={0,0,127}));
  connect(eqAirTempVDI.TBlaSky,eqAirTemp.TBlaSky)
    annotation (Line(points={{26,166},{26,164},{-80,164},{-80,86},{-34,86},{-8,86},
          {-8,104},{-2,104}},color={0,0,127}));
  connect(const3.y,eqAirTempVDI.sunblind)
  annotation (Line(points={{64,195.7},{64,178},{38,178}},color={0,0,127}));
  connect(eqAirTempVDI.TEqAir,prescribedTemperatureRoof.T) annotation (Line(
      points={{49,166},{80,166},{80,188},{90,188},{91,188},{91,186},{91,184.6}},
                                                          color={0,0,127}));
  connect(weaDat.weaBus,HDifTilRoof[1].weaBus) annotation (Line(
    points={{-76,190},{-50,190},{-50,164},{-44,164}},
    color={255,204,51},
    thickness=0.5));
  connect(weaDat.weaBus,HDirTilRoof[1].weaBus) annotation (Line(
    points={{-76,190},{-50,190},{-50,216},{-44,216}},
    color={255,204,51},
    thickness=0.5));
  connect(HDifTilRoof.H,solRadRoof.u2) annotation (Line(points={{-23,164},{-16,
        164},{-16,211.6},{-12.8,211.6}},color={0,0,127}));
  connect(HDifTilColl.weaBus, weaBus) annotation (Line(points={{-16,54},{-61,54},{-61,124}},
    color={255,204,51},thickness=0.5), Text(string="%second",index=1,extent={{6,3},{6,3}}));
  connect(HDirTilColl.weaBus, weaBus) annotation (Line(
    points={{14,36},{-61,36},{-61,124}},color={255,204,51},thickness=0.5), Text(string="%second",index=1,extent={{6,3},{6,3}}));
  connect(radiator.heatPortRad, building.intGainsRad) annotation (Line(points={{
        0,-4.8},{0,26},{128,26},{128,146},{116.2,146}}, color={191,0,0}));
  connect(radiator.heatPortCon, building.intGainsConv) annotation (Line(points={
        {-4,-4.8},{-4,22},{132,22},{132,142},{116,142}}, color={191,0,0}));
  connect(building.TAir, thermostat.u_m) annotation (Line(points={{117,154},{136,
        154},{136,8},{-21.2,8}}, color={0,0,127}));
  connect(TSoil.y, prescribedTemperatureFloor.T) annotation (Line(points={{95.7,
        105},{92.5,105},{92.5,111.4}}, color={0,0,127}));
  connect(weaBus.TDryBul, prescribedTemperature.T) annotation (Line(
    points={{-61,124},{-61,46},{60,46},{60,14},{83.2,14}},
    color={255,204,51},thickness=0.5), Text(string="%first",index=-1,extent={{-6,3},{-6,3}}));
  connect(eqAirTempVDI.HSol, solRadRoof.y) annotation (Line(points={{26,172},{6,
        172},{6,214},{-3.6,214}}, color={0,0,127}));
  connect(solRadRoof.u1, HDirTilRoof.H) annotation (Line(points={{-12.8,216.4},{
        -13.3,216.4},{-13.3,216},{-23,216}}, color={0,0,127}));
  connect(intSou.y, sourcesRad.Q_flow) annotation (Line(points={{62.5,59.5},{66,
        59.5},{66,68},{72,68}}, color={0,0,127}));
  connect(intSou.y, sourcesConv.Q_flow) annotation (Line(points={{62.5,59.5},{66,
        59.5},{66,50},{72,50}}, color={0,0,127}));
  connect(ventilationIn.ports[1], building.ports[1]) annotation (Line(points={{62,
        96},{105.475,96},{105.475,120.85}}, color={0,127,255}));
  connect(ventilationOut.ports[1], building.ports[2]) annotation (Line(points={{
        62,84},{86,84},{108.525,84},{108.525,120.85}}, color={0,127,255}));
  connect(gain.y, ventilationIn.m_flow_in)
  annotation (Line(points={{44.3,99},{54,99},{54,99.2}}, color={0,0,127}));
  connect(gain1.y, ventilationOut.m_flow_in) annotation (Line(points={{52.3,87},
        {52.15,87},{52.15,87.2},{54,87.2}}, color={0,0,127}));
  connect(gain.y, gain1.u)
  annotation (Line(points={{44.3,99},{44.3,87},{45.4,87}}, color={0,0,127}));
  connect(weaBus.TDryBul, ventilationIn.T_in) annotation (Line(
    points={{-61,124},{-62,124},{-62,76},{34,76},{34,108},{50,108},{50,97.6},{53.2,97.6}},
    color={255,204,51},thickness=0.5), Text(string="%first",index=-1,extent={{-6,3},{-6,3}}));
  connect(airchange.y, gain.u) annotation (Line(points={{32.3,99},{35.15,99},{37.4,
        99}}, color={0,0,127}));
  connect(add.u2, storage.T[1]) annotation (Line(points={{128.8,-38.4},{130,-38.4},
        {130,-76},{34,-76},{34,-18.9},{40.6,-18.9}}, color={0,0,127}));
  connect(control.u, add.y)
  annotation (Line(points={{116.8,-36},{119.6,-36}}, color={0,0,127}));
  connect(cutOverheating.u1, control.y)
  annotation (Line(points={{102.8,-36},{107.6,-36}}, color={255,0,255}));
  connect(cutOverheating.y, switch1.u2)
  annotation (Line(points={{93.6,-36},{84.8,-36}}, color={255,0,255}));
  connect(cutOverheating.u2, lessThreshold.y) annotation (Line(points={{102.8,-39.2},
        {106,-39.2},{106,-48},{107.6,-48}}, color={255,0,255}));
  connect(storage.T[1], lessThreshold.u) annotation (Line(points={{40.6,-18.9},{
        38,-18.9},{38,-18},{34,-18},{34,-76},{120,-76},{120,-48},{116.8,-48}},color={0,0,127}));

  connect(realExpression.y, genOptInterface.costFunction)
    annotation (Line(points={{101,228},{120,228},{120,218}}, color={0,0,127}));
  annotation(experiment(StartTime=15638400, StopTime=47174400),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{140,240}},initialScale=0.1),graphics={
    Text(extent={{-76,-50},{114,-124}},lineColor={0,0,255},textString="Solar heating system with low-order building model")}),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,40}})));
end SystemModel;

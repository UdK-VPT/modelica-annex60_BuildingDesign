within SolarHeatingSystem;
model SystemModel "Solar heating system"
  extends Modelica.Icons.Example;
  package Medium = Annex60.Media.Water;
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal= 0.1;

  Annex60.Fluid.Storage.ExpansionVessel exp1(
    redeclare package Medium = Medium, V_start=0.1)
    "Expansion vessel model"
    annotation (Placement(transformation(extent={{20,-54},{32,-42}})));
  Annex60.Experimental.SolarHeatingSystem.Components.Pipe  pip1(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1,
    diameter=0.02)
    "Pipe model"
    annotation (Placement(transformation(extent={{12,-2},{32,-22}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T hea(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=10.0,
    Q_flow_maxCool=0.0)
    "Boiler model"
    annotation (Placement(transformation(extent={{8,-70},{-12,-50}})));
  Annex60.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 rad(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    Q_flow_nominal=4000.0,
    VWat=0.005,
    mDry=0.0001,
    nEle=5,
    fraRad=0,
    T_a_nominal=273.15 + 90.0,
    T_b_nominal=273.15 + 70,
    TAir_nominal=273.15 + 20.0,
    n=1.3,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
    "Radiator model"
    annotation (Placement(transformation(extent={{-12,-22},{8,-2}})));
  Annex60.Experimental.SolarHeatingSystem.Components.Pipe pip2(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1)
    "Pipe model"
    annotation (Placement(transformation(extent={{-16,-70},{-36,-50}})));
  Modelica.Blocks.Sources.Constant TSet(k=273.15 + 60.0)
    annotation (Placement(transformation(extent={{18,-56},{14,-52}})));
  Annex60.Experimental.SolarHeatingSystem.Components.Pipe pip3(
    redeclare package Medium = Medium,
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
    annotation (Placement(transformation(extent={{-60,6},{-40,26}})));
  Annex60.Fluid.Actuators.Valves.TwoWayTable val(
    redeclare package Medium = Medium,
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
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},rotation=90,origin={-26,10})));
  Annex60.Fluid.Movers.FlowControlled_dp pump1(
    redeclare package Medium =Medium, m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-84,-22},{-64,-2}})));
  Modelica.Blocks.Sources.Constant dpSet(
    k=12000.0)
    "Set presure for the pump model"
    annotation (Placement(transformation(extent={{-68,12},{-72,16}})));
  Modelica.Blocks.Sources.Constant TAirSet(
    k=273.15 + 20.0)
    annotation (Placement(transformation(extent={{-20,18},{-24,22}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TAmb(
    each T=293.15)
    annotation (Placement(transformation(extent={{-72,-46},{-60,-34}})));
  Annex60.Experimental.SolarHeatingSystem.Components.ThermalStorages.FluidStorage storage(
    HX_2=false,
    AdditionalFluidPorts=true,
    nEle=10,
    HX_1=true,
    height = 2.0,
    redeclare Annex60.Experimental.SolarHeatingSystem.Components.ThermalStorages.BaseClasses.BuoyancyModels.Buoyancy1 HeatBuoyancy,
    redeclare package Medium = Medium,
    V=20.0)
    annotation (Placement(transformation(extent={{38,-40},{58,-20}})));
  Annex60.Experimental.SolarHeatingSystem.Components.SolarThermal.ThermalCollector collector(
    redeclare package Medium = Medium,
    angleDegAzi=0,
    angleDegTil=30,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=10,
    redeclare Annex60.Experimental.SolarHeatingSystem.Components.SolarThermal.Data.Collectors.ComercialsCollectors.FlatPlate.AgenaAZUR8plus_AC28H collectorData,
    height=5,
    width=5)
    annotation (Placement(transformation(extent={{92,-26},{112,-6}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow pump2(redeclare package Medium = Medium,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{84,-70},{64,-50}})));
  Annex60.Experimental.SolarHeatingSystem.Components.Pipe pip4(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1,
    diameter=0.02)
    "Pipe model"
    annotation (Placement(transformation(extent={{62,-4},{82,-24}})));
  Annex60.Experimental.SolarHeatingSystem.Components.Pipe pip5(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    nNodes=2,
    thicknessIns=0.02,
    lambdaIns=0.04,
    length=1,
    diameter=0.02)
    "Pipe model"
    annotation (Placement(transformation(extent={{94,-50},{114,-70}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
    prescribedTemperature
    annotation (Placement(transformation(extent={{84,-2},{92,6}})));
  Modelica.Blocks.Sources.Constant const(
    k=0.02)
    annotation (Placement(transformation(extent={{92,-36},{88,-32}})));
  Annex60.Fluid.Storage.ExpansionVessel exp2(
    redeclare package Medium = Medium,
    V_start=0.1)
    "Expansion vessel model"
    annotation (Placement(transformation(extent={{48,-58},{60,-46}})));
  Modelica.Blocks.Logical.Hysteresis control(
    uLow=1,
    uHigh=4)
    annotation (Placement(transformation(extent={{102,-44},{94,-36}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{84,-44},{76,-36}})));
  Modelica.Blocks.Sources.Constant const1(
    k=0.0)
    annotation (Placement(transformation(extent={{92,-48},{88,-44}})));
  Modelica.Blocks.Math.Add add(
    k1=1,
    k2=-1)
    annotation (Placement(transformation(extent={{114,-44},{106,-36}})));
  BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
    calTSky=Annex60.BoundaryConditions.Types.SkyTemperatureCalculation.
    HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
      "modelica://Annex60/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"),
    HSou=Annex60.BoundaryConditions.Types.RadiationDataSource.File,
    pAtmSou=Annex60.BoundaryConditions.Types.DataSource.File,
    ceiHeiSou=Annex60.BoundaryConditions.Types.DataSource.File,
    totSkyCovSou=Annex60.BoundaryConditions.Types.DataSource.File,
    opaSkyCovSou=Annex60.BoundaryConditions.Types.DataSource.File,
    TDryBulSou=Annex60.BoundaryConditions.Types.DataSource.File,
    TDewPoiSou=Annex60.BoundaryConditions.Types.DataSource.File,
    relHumSou=Annex60.BoundaryConditions.Types.DataSource.File,
    winSpeSou=Annex60.BoundaryConditions.Types.DataSource.File,
    winDirSou=Annex60.BoundaryConditions.Types.DataSource.File,
    HInfHorSou=Annex60.BoundaryConditions.Types.DataSource.File,
    TBlaSkySou=Annex60.BoundaryConditions.Types.DataSource.File)
    "Weather data reader"
    annotation (Placement(transformation(extent={{-80,146},{-60,166}})));
  BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil[2](
    each outSkyCon=true,
    each outGroCon=true,
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={3.1415926535898,4.7123889803847})
    "Calculates diffuse solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-52,114},{-32,134}})));
  BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil[2](
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={3.1415926535898,4.7123889803847})
    "Calculates direct solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-52,146},{-32,166}})));
  ThermalZones.ReducedOrder.CorrectionSolarGain.CorrectionGDoublePane corGDoublePane(
    UWin=2.1, n=2) "Correction factor for solar transmission"
    annotation (Placement(transformation(extent={{22,140},{42,160}})));
  Modelica.Blocks.Math.Sum aggWindow(
    nin=2, k={0.5,0.5}) "Aggregates both windows to one"
    annotation (Placement(transformation(extent={{50,143},{64,157}})));
  ThermalZones.ReducedOrder.ReducedOrderZones.ThermalZoneFourElements building(
    redeclare package Medium = Modelica.Media.Air.SimpleAir,
    VAir=336.0,
    alphaExt=21.7,
    alphaWin=1.6999999999999997,
    gWin=0.6,
    ratioWinConRad=0.03,
    nExt=1,
    RExt={0.000221375084495},
    CExt={86200772.7108},
    alphaRad=7.947818768410714,
    AInt=318.48,
    alphaInt=6.328522984174829,
    nInt=1,
    RInt={0.00028734526051},
    CInt={43260210.4851},
    RWin=0.0258840291977,
    RExtRem=0.0123709459756,
    AFloor=73.44,
    alphaFloor=1.7,
    nFloor=1,
    RFloor={0.000657855370917},
    RFloorRem=0.0367624625505,
    CFloor={29007444.7758},
    ARoof=73.44,
    alphaRoof=1.7,
    nRoof=1,
    RRoof={0.000660213665372},
    RRoofRem=0.0387053267895,
    CRoof={29132420.995},
    nOrientations=4,
    AWin={5.6, 5.6, 16.8, 5.6},
    ATransparent={5.6, 5.6, 16.8, 5.6},
    AExt={66.96, 42.16, 66.96, 42.16})
    "Thermal zone"
    annotation (Placement(transformation(extent={{60,92},{108,128}})));
  ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow eqAirTemp(
    wfGround=0,
    withLongwave=true,
    aExt=0.7,
    alphaExtOut=20,
    alphaRad=5,
    alphaWinOut=20,
    aWin=0.03,
    eExt=0.9,
    n=2,
    wfWall={0.3043478260869566,0.6956521739130435},
    wfWin={0.5,0.5},
    TGround=285.15)
    "Computes equivalent air temperature"
    annotation (Placement(transformation(extent={{-8,80},{12,100}})));
  Modelica.Blocks.Math.Add solRad[2]
    "Sums up solar radiation of both directions"
    annotation (Placement(transformation(extent={{-22,100},{-12,110}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature1
    "Prescribed temperature for exterior walls outdoor surface temperature"
    annotation (Placement(transformation(extent={{24,88},{36,100}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature2
    "Prescribed temperature for windows outdoor surface temperature"
    annotation (Placement(transformation(extent={{24,108},{36,120}})));
  Modelica.Thermal.HeatTransfer.Components.Convection thermalConductorWin
    "Outdoor convective heat transfer of windows"
    annotation (Placement(transformation(extent={{54,110},{44,120}})));
  Modelica.Thermal.HeatTransfer.Components.Convection thermalConductorWall
    "Outdoor convective heat transfer of walls"
    annotation (Placement(transformation(extent={{52,100},{42,90}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow personsRad
    "Radiative heat flow of persons"
    annotation (Placement(transformation(extent={{64,52},{84,72}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow personsConv
    "Convective heat flow of persons"
    annotation (Placement(transformation(extent={{64,38},{84,58}})));
  Modelica.Blocks.Sources.CombiTimeTable internalGains(
    table=[0,0,0,0; 3600,0,0,0; 7200,0,0,0; 10800,0,0,0; 14400,0,0,0; 18000,0,0,
        0; 21600,0,0,0; 25200,0,0,0; 25200,80,80,200; 28800,80,80,200; 32400,80,
        80,200; 36000,80,80,200; 39600,80,80,200; 43200,80,80,200; 46800,80,80,200;
        50400,80,80,200; 54000,80,80,200; 57600,80,80,200; 61200,80,80,200; 61200,
        0,0,0; 64800,0,0,0; 72000,0,0,0; 75600,0,0,0; 79200,0,0,0; 82800,0,0,0;
        86400,0,0,0],
    columns={2,3,4},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic) "Table with profiles for persons (radiative and convective) and machines
    (convective)"
    annotation (Placement(transformation(extent={{36,40},{52,56}})));
  Modelica.Blocks.Sources.Constant const2[2](
    each k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{-4,108},{2,114}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow machinesConv
    "Convective heat flow of machines"
    annotation (Placement(transformation(extent={{64,22},{84,42}})));
  Modelica.Blocks.Sources.Constant alphaWall(
    k=25*11.5)
    "Outdoor coefficient of heat transfer for walls"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=90,
    origin={46,78})));
  Modelica.Blocks.Sources.Constant alphaWin(
    k=20*14)
    "Outdoor coefficient of heat transfer for windows"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},
    rotation=90,origin={48,132})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperatureFloor
    "Prescribed temperature for floor plate outdoor surface temperature"
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
    rotation=90,origin={83,82})));
  Modelica.Blocks.Sources.Constant TSoil(k=283.15)
    "Outdoor surface temperature for floor plate"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},
    rotation=180,origin={100,72})));
  ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007 eqAirTempVDI(
    aExt=0.7,
    eExt=0.9,
    n=1,
    wfWall={1},
    wfWin={0},
    wfGround=0,
    alphaExtOut=20,
    alphaRad=5,
    TGround=285.15)
    "Computes equivalent air temperature for roof"
    annotation (Placement(transformation(extent={{46,168},{66,188}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperatureRoof
    "Prescribed temperature for roof outdoor surface temperature"
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},rotation=-90,
    origin={83,158})));
  Modelica.Thermal.HeatTransfer.Components.Convection thermalConductorRoof
    "Outdoor convective heat transfer of roof"
    annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,
    origin={83,141})));
  Modelica.Blocks.Sources.Constant alphaRoof(k=25*11.5)
    "Outdoor coefficient of heat transfer for roof"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},rotation=0,
    origin={102,141})));
  Modelica.Blocks.Sources.Constant const3(each k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{84,184},{78,190}})));
  BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(transformation(extent={{-84,84},{-50,116}}),
    iconTransformation(extent={{-70,-12},{-50,8}})));
  BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTilColl(
    each til=0.5235987755983,
    each lat=0.87266462599716,
    azi=0)
    "Calculates direct solar radiation on titled solr collector"
    annotation (Placement(transformation(extent={{14,20},{26,32}})));
  BoundaryConditions.SolarIrradiation.DiffusePerez HDifTilColl(
    each outSkyCon=true,
    each outGroCon=true,
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi=0)
    "Calculates diffuse solar radiation on titled surface of the solar collector"
    annotation (Placement(transformation(extent={{-12,34},{0,46}})));
equation
  connect(rad.port_b,pip1.port_a) annotation (Line(
      points={{8,-12},{12,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pip2.port_a,hea.port_b) annotation (Line(
      points={{-16,-60},{-12,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hea.TSet,TSet.y) annotation (Line(
      points={{10,-54},{13.8,-54}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pip3.port_b,val.port_a) annotation (Line(
      points={{-40,-12},{-36,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(val.port_b,rad.port_a) annotation (Line(
      points={{-16,-12},{-12,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump1.port_b, pip3.port_a) annotation (Line(
      points={{-64,-12},{-60,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pip2.port_b, pump1.port_a) annotation (Line(
      points={{-36,-60},{-90,-60},{-90,-12},{-84,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(dpSet.y, pump1.dp_in) annotation (Line(
      points={{-72.2,14},{-74.2,14},{-74.2,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(thermostat.y,val.y) annotation (Line(
      points={{-26,5.6},{-26,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TAirSet.y,thermostat.u_s) annotation (Line(
      points={{-24.2,20},{-26,20},{-26,14.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TAmb.port, pip3.heatPort)
    annotation (Line(points={{-60,-40},{-50,-40},{-50,-17}}, color={191,0,0}));
  connect(TAmb.port, pip1.heatPort) annotation (Line(points={{-60,-40},{-60,-40},
          {22,-40},{22,-17}}, color={191,0,0}));
  connect(TAmb.port, pip2.heatPort) annotation (Line(points={{-60,-40},{-60,-40},
          {-26,-40},{-26,-55}}, color={191,0,0}));
  connect(pip1.port_b, storage.port_a[5]) annotation (Line(points={{32,-12},{38,
          -12},{38,-30},{41,-30}},     color={0,127,255}));
  connect(storage.port_b1, hea.port_a) annotation (Line(points={{41,-21},{36,-21},
          {36,-60},{8,-60}},      color={0,127,255}));
  connect(exp1.port_a, hea.port_a)
    annotation (Line(points={{26,-54},{26,-60},{8,-60}}, color={0,127,255}));
  connect(pip4.port_b, collector.port_a)
    annotation (Line(points={{82,-14},{88,-14},{88,-16},{92,-16}},
                                                 color={0,127,255}));
  connect(pump2.port_a, pip5.port_a)
    annotation (Line(points={{84,-60},{94,-60}}, color={0,127,255}));
  connect(pip5.port_b, collector.port_b) annotation (Line(points={{114,-60},{120,
          -60},{120,-16},{112,-16}},      color={0,127,255}));
  connect(prescribedTemperature.port, collector.heatPortCon)
    annotation (Line(points={{92,2},{107,2},{107,-7}},   color={191,0,0}));
  connect(pip5.heatPort, TAmb.port) annotation (Line(points={{104,-65},{104,-78},
          {-50,-78},{-50,-40},{-60,-40}}, color={191,0,0}));
  connect(pip4.heatPort, TAmb.port) annotation (Line(points={{72,-19},{72,-20},{
          72,-42},{48,-42},{48,-78},{-50,-78},{-50,-40},{-60,-40}}, color={191,0,
          0}));
  connect(exp2.port_a, pump2.port_b) annotation (Line(points={{54,-58},{54,-60},{64,-60}}, color={0,127,255}));
  connect(switch1.y, pump2.m_flow_in) annotation (Line(points={{75.6,-40},{74.2,
          -40},{74.2,-48}}, color={0,0,127}));
  connect(const.y, switch1.u1) annotation (Line(points={{87.8,-34},{84.8,-34},{84.8,
          -36.8}}, color={0,0,127}));
  connect(const1.y, switch1.u3) annotation (Line(points={{87.8,-46},{84.8,-46},{
          84.8,-43.2}}, color={0,0,127}));
  connect(switch1.u2, control.y)
    annotation (Line(points={{84.8,-40},{93.6,-40}}, color={255,0,255}));
  connect(control.u, add.y)
    annotation (Line(points={{102.8,-40},{105.6,-40}}, color={0,0,127}));
  connect(add.u2, storage.T[1]) annotation (Line(points={{114.8,-42.4},{122,-42.4},
          {122,-80},{38,-80},{38,-24.9},{40.6,-24.9}},        color={0,0,127}));
  connect(collector.TSeg[10], add.u1) annotation (Line(points={{105.2,-25.9},{122,
          -25.9},{122,-37.6},{114.8,-37.6}},      color={0,0,127}));
  connect(storage.port_HX_1_b, pump2.port_b) annotation (Line(
      points={{55,-36},{60,-36},{60,-60},{64,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(storage.port_HX_1_a, pip4.port_a) annotation (Line(
      points={{55,-34},{60,-34},{60,-14},{62,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(eqAirTemp.TEqAirWindow,prescribedTemperature2.T)
    annotation (Line(points={{11.8,92.6},{16,92.6},{16,114},{22.8,114}},
    color={0,0,127}));
  connect(eqAirTemp.TEqAir, prescribedTemperature1.T) annotation (Line(points={{
          11.8,84.4},{20,84.4},{20,94},{22.8,94}}, color={0,0,127}));
  connect(corGDoublePane.solarRadWinTrans,aggWindow.u)
    annotation (Line(points={{41,150},{48.6,150}},
                                                 color={0,0,127}));
  connect(aggWindow.y,building.solRad)
    annotation (Line(points={{64.7,150},{70,150},{70,138},{56,138},{56,124.8},{61,
          124.8}},
                color={0,0,127}));
  connect(weaDat.weaBus,weaBus)
    annotation (Line(points={{-60,156},{-58,156},{-58,112},{-68,112},{-68,106},{
          -67,106},{-67,100}},
                      color={255,204,51},
   thickness=0.5), Text(string="%second",index=1,extent={{6,3},{6,3}}));
  connect(weaBus.TDryBul,eqAirTemp.TDryBul)
    annotation (Line(points={{-67,100},{-67,92},{-22,92},{-22,84.2},{-7,84.2}},
    color={255,204,51},
    thickness=0.5), Text(string="%first",index=-1,extent={{-6,3},{-6,3}}));
  connect(internalGains.y[1],personsRad.Q_flow)
    annotation (Line(points={{52.8,48},{56,48},{56,62},{64,62}},
    color={0,0,127}));
  connect(internalGains.y[2],personsConv.Q_flow)
    annotation (Line(points={{52.8,48},{64,48}},            color={0,0,127}));
  connect(internalGains.y[3],machinesConv.Q_flow)
    annotation (Line(points={{52.8,48},{56,48},{56,32},{64,32}},
    color={0,0,127}));
  connect(const2.y, eqAirTemp.sunblind) annotation (Line(points={{2.3,111},{4,111},
          {4,102},{2,102},{2,99}}, color={0,0,127}));
  connect(HDifTil.HSkyDifTil,corGDoublePane.HSkyDifTil)
    annotation (Line(points={{-31,130},{-12,130},{10,130},{10,152},{16,152},{16,
          151.8},{20,151.8},{22,151.8}},
                       color={0,0,127}));
  connect(HDirTil.H,corGDoublePane.HDirTil)
    annotation (Line(points={{-31,156},{6,156},{22,156}},
                                                       color={0,0,127}));
  connect(HDirTil.H,solRad.u1)
    annotation (Line(points={{-31,156},{-26,156},{-26,108},{-23,108}},
    color={0,0,127}));
  connect(HDifTil.H,solRad.u2)
    annotation (Line(points={{-31,124},{-28,124},{-28,102},{-23,102}},
    color={0,0,127}));
  connect(HDifTil.HGroDifTil,corGDoublePane.HGroDifTil)
    annotation (Line(points={{-31,118},{12,118},{12,147.6},{22,147.6}},
    color={0,0,127}));
  connect(solRad.y,eqAirTemp. HSol)
    annotation (Line(points={{-11.5,105},{-10,105},{-10,94.4},{-7,94.4}},
    color={0,0,127}));
  connect(weaDat.weaBus,HDifTil[1].weaBus)
    annotation (Line(points={{-60,156},{-58,156},{-58,124},{-52,124}},
    color={255,204,51},thickness=0.5));
  connect(weaDat.weaBus,HDifTil[2].weaBus)
    annotation (Line(points={{-60,156},{-58,156},{-58,124},{-52,124}},
    color={255,204,51},thickness=0.5));
  connect(weaDat.weaBus,HDirTil[1].weaBus)
    annotation (Line(
    points={{-60,156},{-60,156},{-52,156}},
    color={255,204,51},
    thickness=0.5));
  connect(weaDat.weaBus,HDirTil[2].weaBus)
    annotation (Line(
    points={{-60,156},{-60,156},{-52,156}},
    color={255,204,51},
    thickness=0.5));
  connect(personsRad.port,building.intGainsRad)
    annotation (
    Line(points={{84,62},{100,62},{116,62},{116,119},{107,119}},color={191,0,
    0}));
  connect(thermalConductorWin.solid,building.window)
    annotation (Line(points={{54,115},{56,115},{56,114},{61,114},{61,113.8}},
                                                                         color=
    {191,0,0}));
  connect(prescribedTemperature2.port,thermalConductorWin.fluid)
    annotation (Line(points={{36,114},{44,114},{44,115}},
                                                       color={191,0,0}));
  connect(building.extWall,thermalConductorWall.solid)
    annotation (Line(points={{61,106.4},{56,106.4},{56,95},{52,95}},
    color={191,0,0}));
  connect(thermalConductorWall.fluid, prescribedTemperature1.port) annotation (
      Line(points={{42,95},{40,95},{40,94},{36,94}}, color={191,0,0}));
  connect(alphaWall.y,thermalConductorWall. Gc)
    annotation (Line(points={{46,82.4},{46,90},{47,90}},  color={0,0,127}));
  connect(alphaWin.y,thermalConductorWin. Gc)
    annotation (Line(points={{48,127.6},{48,120},{49,120}},
                                                         color={0,0,127}));
  connect(weaBus.TBlaSky,eqAirTemp. TBlaSky)
    annotation (Line(
    points={{-67,100},{-42,100},{-42,96},{-16,96},{-16,89.4},{-7,89.4}},
    color={255,204,51},
    thickness=0.5), Text(
    string="%first",
    index=-1,
    extent={{-6,3},{-6,3}}));
  connect(machinesConv.port,building.intGainsConv)
    annotation (
    Line(points={{84,32},{98,32},{112,32},{112,113.8},{107,113.8}},
                                                                  color={191,
    0,0}));
  connect(personsConv.port,building.intGainsConv)
    annotation (
    Line(points={{84,48},{112,48},{112,113.8},{107,113.8}},
                                                         color={191,0,0}));
  connect(prescribedTemperatureFloor.port,building.floor)
    annotation (Line(points={{83,88},{82.8,88},{82.8,93}}, color={191,0,0}));
  connect(TSoil.y,prescribedTemperatureFloor. T)
  annotation (Line(points={{95.6,72},{83,72},{83,74.8}},    color={0,0,127}));
  connect(prescribedTemperatureRoof.port,thermalConductorRoof. fluid)
    annotation (Line(points={{83,152},{83,152},{83,146}},
                                                       color={191,0,0}));
  connect(thermalConductorRoof.solid,building.roof)
    annotation (Line(points={{83,136},{82.8,136},{82.8,127}},
                                                           color={191,0,0}));
  connect(eqAirTempVDI.TEqAir,prescribedTemperatureRoof.T)
    annotation (Line(
    points={{65.8,172.4},{83,172.4},{83,165.2}},
                                              color={0,0,127}));
  connect(thermalConductorRoof.Gc,alphaRoof.y)
    annotation (Line(points={{88,141},{94,141},{97.6,141}},
                                                        color={0,0,127}));
  connect(eqAirTempVDI.TDryBul,eqAirTemp.TDryBul)
    annotation (Line(points={{47,172.2},{-82,172.2},{-82,80},{-24,80},{-24,84.2},
          {-7,84.2}},                                   color={0,0,127}));
  connect(eqAirTempVDI.TBlaSky,eqAirTemp.TBlaSky)
    annotation (Line(points={{47,177.4},{-18,177.4},{-18,178},{-82,178},{-82,86},
          {-42,86},{-42,96},{-16,96},{-16,89.4},{-7,89.4}},
                color={0,0,127}));
  connect(eqAirTempVDI.HSol[1],weaBus.HGloHor)
    annotation (Line(points={{47,182.4},{-84,182.4},{-84,100},{-67,100}},color={0,0,127}),
      Text(string="%second",index=1,extent={{6,3},{6,3}}));
  connect(HDirTil.inc,corGDoublePane. inc)
    annotation (Line(points={{-31,152},{-12,152},{6,152},{6,143.4},{22,143.4}},
    color={0,0,127}));
  connect(const3.y,eqAirTempVDI.sunblind[1])
    annotation (Line(points={{77.7,187},{72,187},{72,192},{56,192},{56,187}},
                                      color={0,0,127}));

  connect(thermostat.u_m, building.TIndAir) annotation (Line(points={{-21.2,10},
          {52,10},{120,10},{120,125},{107,125}}, color={0,0,127}));
  connect(rad.heatPortCon, building.intGainsConv) annotation (Line(points={{-4,-4.8},
          {-4,-4.8},{-4,14},{124,14},{124,113.8},{107,113.8}}, color={191,0,0}));
  connect(rad.heatPortRad, building.intGainsRad) annotation (Line(points={{0,-4.8},
          {0,-4.8},{0,12},{128,12},{128,119},{107,119}}, color={191,0,0}));
  connect(weaBus, HDifTilColl.weaBus) annotation (Line(
      points={{-67,100},{-67,40},{-12,40}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, HDirTilColl.weaBus) annotation (Line(
      points={{-67,100},{-67,26},{14,26}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus.TDryBul, prescribedTemperature.T) annotation (Line(
      points={{-67,100},{-68,100},{-68,60},{32,60},{32,2},{83.2,2}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(HDirTilColl.H, collector.IrrDir) annotation (Line(points={{26.6,26},{62,
          26},{98.3,26},{98.3,-8.3}}, color={0,0,127}));
  connect(HDirTilColl.inc, collector.inc) annotation (Line(points={{26.6,23.6},{
          105.1,23.6},{105.1,-8.3}}, color={0,0,127}));
  connect(HDifTilColl.H, collector.IrrDif) annotation (Line(points={{0.6,40},{16,
          40},{34,40},{34,18},{101.7,18},{101.7,-8.3}}, color={0,0,127}));
  annotation(experiment(StartTime=0, StopTime=31536000),
    __Dymola_Commands(file="modelica://BuildingSystems/Resources/Scripts/Dymola/Applications/HeatingSystems/SolarHeatingSystem.mos" "Simulate and plot"),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{140,200}},initialScale=0.1),graphics={
    Text(extent={{-56,-54},{48,-122}}, lineColor={0,0,255},textString="Solar heating system with low-order building model")}),
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,40}})));
end SystemModel;

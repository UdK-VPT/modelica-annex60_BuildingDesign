within SolarHeatingSystem.Interfaces;
connector FlowPorts
  "Port for fluid transport, large icon to be used for vectors of FlowPorts (base connector type)"
  extends Modelica.Fluid.Interfaces.FluidPort;
  annotation(defaultComponentName="flowPorts",Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-50,-200},{50,200}},grid={1,1},initialScale=0.2), graphics={
  Text(extent={{-75,130},{75,100}}, textString="%name"),
  Rectangle(extent={{25,-100},{-25,100}},lineColor={0,127,255}),
  Ellipse(extent={{-25,90},{25,40}},lineColor={0,0,0},fillColor={0,127,255},fillPattern=FillPattern.Solid),
  Ellipse(extent={{-25,25},{25,-25}},lineColor={0,0,0},fillColor={0,127,255},fillPattern=FillPattern.Solid),
  Ellipse(extent={{-25,-40},{25,-90}},lineColor={0,0,0},fillColor={0,127,255},fillPattern=FillPattern.Solid)}),
  Icon(coordinateSystem(preserveAspectRatio=false,extent={{-50,-200},{50,200}},grid={1,1},initialScale=0.2), graphics={
  Rectangle(extent={{50,-200},{-50,200}},lineColor={0,127,255},fillColor={255,255,255},
            fillPattern = FillPattern.Solid),
  Ellipse(extent={{-50,180},{50,80}},lineColor={0,0,0},fillColor={0,127,255},
            fillPattern = FillPattern.Solid),
  Ellipse(extent={{-50,50},{50,-50}},lineColor={0,0,0},fillColor={0,127,255},
            fillPattern = FillPattern.Solid),
  Ellipse(extent={{-50,-80},{50,-180}},lineColor={0,0,0},fillColor={0,127,255},
            fillPattern = FillPattern.Solid)}));
end FlowPorts;

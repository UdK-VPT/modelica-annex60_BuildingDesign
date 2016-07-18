within SolarHeatingSystem.Interfaces;
model GenOptInterface "Text file output for use with GenOpt"
  parameter String resultFileName = "result.txt"
    "File on which data is present";
  parameter String header = "Objective function value" "Header for result file";
  parameter String File = "00_InfoFile_after_GenOpt.txt"
    "File on which data is present";
  Modelica.Blocks.Interfaces.RealInput costFunction
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,80})));
initial algorithm
 if (resultFileName <> "") then
  Modelica.Utilities.Files.removeFile(resultFileName);
 end if;
 Modelica.Utilities.Streams.print(fileName=resultFileName,string=header);
 if (File <> "") then
  Modelica.Utilities.Files.removeFile(File);
 end if;
equation
 when terminal() then Modelica.Utilities.Streams.print("f(x) = " +    realString(number=costFunction, minimumWidth=10, precision=20), resultFileName);
    // INFORMATION FILE
    Modelica.Utilities.Streams.print("-------------------------------------------------", File);
    Modelica.Utilities.Streams.print("f(x):                       " +  realString(number=costFunction, minimumWidth=10, precision=20), File);
  end when;
end GenOptInterface;

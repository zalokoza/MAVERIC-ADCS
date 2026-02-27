startTime = datetime(2020,5,11,12,35,38);
stopTime = startTime + days(2);
sampleTime = 60;
sc = satelliteScenario(startTime,stopTime,sampleTime);
tleFile = "eccentricOrbitSatellite.tle";
satTwoBodyKeplerian = satellite(sc, , ...
    "Name","satTwoBodyKeplerian", ...
    "OrbitPropagator","two-body-keplerian");
v = satelliteScenarioViewer(sc);
satSGP4.MarkerColor = "green";
satSGP4.Orbit.LineColor = "green";
satSGP4.LabelFontColor = "green";
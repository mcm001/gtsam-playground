Guess for tags 10 
R: [
	-0.883832, -0.467793, 0.00334501;
	0.467776, -0.883676, 0.0173676;
	-0.00516855, 0.0169148, 0.999844
]
t:  7.28108  1.03629 0.589645
Final pose graph
size: 5

Factor 0:   keys = { l10 x0 }
  noise model: unit (2) 
ExpressionFactor with measurement: [
	44.506;
	267.746
]

Factor 1:   keys = { l10 x0 }
  noise model: unit (2) 
ExpressionFactor with measurement: [
	58.9329;
	270.574
]

Factor 2:   keys = { l10 x0 }
  noise model: unit (2) 
ExpressionFactor with measurement: [
	59.2499;
	251.934
]

Factor 3:   keys = { l10 x0 }
  noise model: unit (2) 
ExpressionFactor with measurement: [
	44.9801;
	248.141
]

Factor 4: PriorFactor on l10
  prior mean:  R: [
	0.5, -0.866025, 0;
	0.866025, 0.5, 0;
	0, 0, 1
]
t:  1.46152 0.245872  1.35585
  noise model: diagonal sigmas [0.1; 0.1; 0.1; 0.3; 0.3; 0.3];

initial error = 219655.848
Initial state:

Values with 2 values:
Value l10: (gtsam::Pose3)
R: [
	0.5, -0.866025404, 0;
	0.866025404, 0.5, 0;
	0, 0, 1
]
t: 1.461516 0.245872 1.355852

Value x0: (gtsam::Pose3)
R: [
	-0.88383168, -0.467793087, 0.00334500722;
	0.467776493, -0.883676139, 0.0173676303;
	-0.0051685543, 0.0169147776, 0.999843576
]
t:  7.28108468  1.03629214 0.589645256

final error = 0.87292482
Final state:

Values with 2 values:
Value l10: (gtsam::Pose3)
R: [
	0.5, -0.866025404, -4.85083723e-16;
	0.866025404, 0.5, -8.21853968e-16;
	9.54293609e-16, -9.17614651e-18, 1
]
t: 1.461516 0.245872 1.355852

Value x0: (gtsam::Pose3)
R: [
	-0.844620113, -0.246855609, 0.475057021;
	0.533346259, -0.464989877, 0.706630159;
	0.0464610876, 0.85020393, 0.524399318
]
t:   1.8044012 0.955168303 -1.02292607


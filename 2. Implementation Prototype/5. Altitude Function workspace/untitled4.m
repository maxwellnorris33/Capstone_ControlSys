model = 'ssc_hydraulic_actuator_digital_control';
open_system(model)
sim(model)
simlogRef = simlog_ssc_hydraulic_actuator_digital_control;
pRefNode = simlogRef.Hydraulic_Actuator.Hydraulic_Cylinder.Chamber_A.A.p;
pRef = pRefNode.series.values('Pa');
tRef = pRefNode.series.time;
h1 = figure;
semilogy(tRef(1:end-1),diff(tRef),'-x')
title('Solver Step Size')
xlabel('Time (s)')
ylabel('Step Size (s)')
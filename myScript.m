matlab.project.loadProject(pwd);
tf = sltest.testmanager.load('LaneFollowingTestScenarios.mldatx');
tf.run;
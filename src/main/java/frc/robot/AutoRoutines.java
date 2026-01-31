package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("choreo_180_turn");
        final AutoTrajectory simplePath = routine.trajectory("turningforward");

        final AutoRoutine routine2 = m_factory.newRoutine("choreo_180_turn");
        final AutoTrajectory simplePath2 = routine.trajectory("turningforward");


        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }
    public AutoRoutine simplePathAuto2() {
        final AutoRoutine routine2 = m_factory.newRoutine("choreo_cross_experiment");
        final AutoTrajectory simplePath2 = routine2.trajectory("choreo_cross_test");


        routine2.active().onTrue(
            simplePath2.resetOdometry()
                .andThen(simplePath2.cmd())
        );
        return routine2;
       
        
       
        
    
    }
}

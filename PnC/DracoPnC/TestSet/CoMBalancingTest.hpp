#pragma once

#include <PnC/Test.hpp>

class RobotSystem;
class DracoStateProvider;

enum CoMBalancingTestPhase{
    BT_initial_jpos = 0,
    BT_lift_up = 1,
    BT_com_ctrl = 2,
    NUM_BT_PHASE
};

class CoMBalancingTest: public Test{
    public:
        CoMBalancingTest(RobotSystem* );
        virtual ~CoMBalancingTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller* jpos_target_ctrl_;
        Controller* body_lift_ctrl_;
        Controller* com_ctrl_;

        YAML::Node cfg_;
        DracoStateProvider* sp_;
};

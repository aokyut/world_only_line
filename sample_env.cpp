#include "Env.h"
#include <iostream>
#include <vector>

int main(void)
{
    // TODO: LineBodyとJointの追加を
    physics::Env env;
    env.reset();
    env.testEnvSet();
    cout << "flag1" << endl;
    int max_step = 10000;
    vector<float> dummy_vec;

    for (int i = 0; i < max_step; i++)
    {
        // env.show();
        // cout << "flag1" << endl;
        env.step(dummy_vec);
        // cout << "flag2" << endl;
        env.render();
    }
}
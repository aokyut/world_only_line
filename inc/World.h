#pragma once
#include "LineBody.h"
#include "Constraint.h"

namespace physics
{
    class World
    {
    private:
        std::vector<LineBody *> bodies;
        std::vector<Constraint *> constraints;
        int body_count;

    public:
        World() : body_count(0)
        {
        }
        void addBody(LineBody *body)
        {
            bodies.push_back(body);
            body_count++;
        }
        void addJoint(Constraint *constraint)
        {
            constraints.push_back(constraint);
        }
        void step()
        {
            int cmax = constraints.size();

            // 拘束を処理する
            int num = 10;
            // std::cout << num << std::endl;
            for (int i = 0; i < cmax; i++)
            {
                // constraints[i]->check();
            }
            for (int i = 0; i < num * cmax; i++)
            {
                // constraints[rand() % cmax]->correct();
                // for (int j = 0; j < cmax; j++)
                // {
                //     constraints[cmax - j - 1]->correct();
                // }
            }

            for (int i = 0; i < body_count; i++)
            {
                bodies[i]->update();
            }
        }
        void render()
        {
            for (int i = 0; i < body_count; i++)
            {
                bodies[i]->draw();
            }
        }
    };
}
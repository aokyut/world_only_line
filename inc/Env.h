#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "World.h"
#include "LineBody.h"
#include "Constraint.h"

namespace physics
{
    class Env
    {
    private:
        World *world;
        // std::optional<Window> window;
        bool isOpenWindow;
        int tStep;
        float cameraPosX;
        float cameraPosY;
        float torqueScale = 20.0;
        vector<LineAgent> agents;
        int bodyNum = 0;
        int jointNum = 0;

    public:
        Vector2f g = Vector2f(0, -0.5);

        Env(float scale) : world(new World()), torqueScale(scale)
        {
        }

        void reset()
        {
            world->reset();
            tStep = 0;
            bodyNum = 0;
            jointNum = 0;
            agents = vector<LineAgent>();
        }

        void testEnvSet2()
        {
            addAgent(1);
            addLine(0, 1, 0.7);
            addLine(0, 1, -0.7);
            addLine(1, 0.5, -0.6);
            addLine(2, 0.5, 0.6);
            endAgentAssemble();
        }

        void testEnvSet()
        {
            addAgent(1);
            addLine(0, 1, 0.5);
            // addLine(0, 1, 0.1);
            addLine(0, 1, -0.5);
            endAgentAssemble();
        }

        void show()
        {
            world->show();
        }

        /// @brief
        /// @param actions: Matrix(n, 1)
        void step(MatrixXf actions)
        {
            int row = 0;
            for (LineAgent agent : agents)
            {
                for (int i = 0; i < agent.size(); i++)
                {
                    agent.setTorque(i, actions(row, 0), torqueScale);
                    row++;
                }
            }
            world->addAccelToAll(g);
            world->step();
            tStep++;
        }

        void addAgent(float l)
        {
            LineAgent agent = LineAgent(l, world);
            agents.push_back(agent);
            bodyNum++;
        }

        /// @brief add Line to Agent
        /// @param parentIndex: tar Line to connect
        /// @param l: length of additional Line
        /// @param r: tar pos of Line [-1 ~ 1]
        void addLine(int parentIndex, float l, float r)
        {
            agents[agents.size() - 1].addBody(parentIndex, l, r, world);
            bodyNum++;
            jointNum++;
        }

        /// @brief get body observation
        /// @return obs(bodyNum, {length, center_y, angle_vec(2), velocities(3)}(7))
        MatrixXf getObservationBody()
        {
            MatrixXf obs(bodyNum, 7);
            int tar_i = 0;
            for (LineAgent agent : agents)
            {
                for (LineBody *body : agent.bodies)
                {
                    Vector2f angle_vec = (body->t - body->s).normalized();
                    float center_y = body->c(1);
                    Vector2f vel = body->v;
                    float w = body->w;
                    VectorXf obs_row(7);
                    obs_row << body->length, center_y, angle_vec, vel, w;
                    obs.row(tar_i) = obs_row;
                    tar_i++;
                }
            }
            return obs;
        }

        /// @brief get joint observation
        /// @return obs(jointNum, {Joint.r, Joint.lim_angle_sin, angle(2)}(4))
        MatrixXf getObservationJoint()
        {
            MatrixXf obs(jointNum, 4);
            int tar_i = 0;
            for (LineAgent agent : agents)
            {
                for (HingeJoint *joint : agent.joints)
                {
                    Vector2f angle_vec = joint->getAngleVector();
                    VectorXf obs_row(4);
                    obs_row << joint->r, joint->limit_angle_sin, angle_vec;
                    obs.row(tar_i) = obs_row;
                    tar_i++;
                }
            }
            return obs;
        }

        MatrixXf getCenterXs()
        {
            MatrixXf xs(agents.size(), 1);
            for (int i = 0; i < agents.size(); i++)
            {
                xs(i, 0) = agents[i].getCenterX();
            }

            return xs;
        }

        MatrixXf getCenterYs()
        {
            MatrixXf ys(agents.size(), 1);
            for (int i = 0; i < agents.size(); i++)
            {
                ys(i, 0) = agents[i].getCenterY();
            }

            return ys;
        }

        MatrixXf getEnergyConsumptions()
        {
            MatrixXf es(agents.size(), 1);
            for (int i = 0; i < agents.size(); i++)
            {
                es(i, 0) = agents[i].getEnergyConsumption();
            }

            return es;
        }

        /// @brief get line points for renderer
        /// @return mat(bodies + 2, 4)
        MatrixXf getLines()
        {
            MatrixXf lines(bodyNum + 2, 4);
            // add bodies
            int tar_idx = 0;
            float max_center_x = 0;
            for (LineAgent agent : agents)
            {
                for (LineBody *body : agent.bodies)
                {
                    VectorXf row(4);
                    row << body->s, body->t;
                    lines.row(tar_idx) = row;
                    tar_idx++;
                }
                const float center_x = agent.getCenterX();
                if (max_center_x < center_x)
                {
                    max_center_x = center_x;
                }
            }

            VectorXf floor(4);
            floor << -1000, world->floor_y, 1000, world->floor_y;
            VectorXf vertical1(4);
            // VectorXf vertical2(4);
            max_center_x = ::floorf(max_center_x);
            vertical1 << max_center_x, world->floor_y, max_center_x, 1000;
            // vertical2 << max_center_x + 1, world->floor_y, max_center_x, 1000;
            lines.row(tar_idx) = floor;
            lines.row(tar_idx + 1) = vertical1;
            // lines.row(tar_idx + 2) = vertical2;

            return lines;
        }

        void endAgentAssemble()
        {
            agents[agents.size() - 1].tweak(world);
        }
    };
}
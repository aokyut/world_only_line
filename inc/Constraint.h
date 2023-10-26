#pragma once
#include <iostream>
#include <vector>
#include "LineBody.h"
#include <Eigen/Core>

using namespace std;

namespace physics
{
    enum CType
    {
        F, // 無拘束
        E, // 基本拘束
        C  // 相補拘束
    };

    // Jacobian of constraint
    struct HingeJacobian
    {
        bool isAngle;
        int iid;
        int jid;
        float vix;
        float viy;
        float wi;
        float vjx;
        float vjy;
        float wj;
    };

    class Constraint
    {
    protected:
    public:
        Constraint() {}
        virtual vector<HingeJacobian> getJacobian() {}
        virtual int wsize() {}
        virtual void fix(float, float) {} // applay penalty force
    };

    // 回転を制限されない拘束
    class HingeJoint : public Constraint
    {
    public:
        LineBody *ibody, *jbody;
        float r;
        float limit_angle_sin;
        bool flip;

        HingeJoint(LineBody *ibody, LineBody *jbody, float r, float limit_angle = 0.393, bool flip = false) : ibody(ibody),
                                                                                                              jbody(jbody),
                                                                                                              r(r),
                                                                                                              flip(flip)
        {
            if (flip)
            {
                limit_angle_sin = -cosf(limit_angle);
            }
            else
            {
                limit_angle_sin = cosf(limit_angle);
            }
        };

        vector<HingeJacobian> getJacobian() override
        {
            // 角度拘束条件の確認
            // check: alpha < theta_ij < beta
            // sin(alpha) = sin(beta) = limit_angle_sin
            Vector2f rj = jbody->s - jbody->c;
            Vector2f sti_normalized = (ibody->t - ibody->c).normalized();
            Vector2f stj_normalized = -rj.normalized();
            float sin_ij = sti_normalized(0) * stj_normalized(1) - sti_normalized(1) * stj_normalized(0);
            float cos_ij = sti_normalized.dot(stj_normalized);
            if (flip && sin_ij > limit_angle_sin)
            {
                cout << "sin_ij: " << sin_ij << endl
                     << "cos_ij: " << cos_ij << endl;
                vector<HingeJacobian> ans(3);
                // alpha = -pi/2 - phi
                // beta  = -pi/2 + phi
                // sin(theta) > sin(alpha) = sin(beta)
                Vector2f ri = ibody->ri(r) - ibody->c;
                int iid = ibody->_getId();
                int jid = jbody->_getId();
                struct HingeJacobian j1 = {false, iid, jid, 1, 0, -ri(1), -1, 0, rj(1)};
                struct HingeJacobian j2 = {false, iid, jid, 0, 1, ri(0), 0, -1, -rj(0)};
                ans[0] = j1;
                ans[1] = j2;
                if (cos_ij > 0)
                {
                    // theta > beta
                    // w = -theta' = -(wj - wi)
                    struct HingeJacobian j3 = {true, iid, jid, 0, 0, 1, 0, 0, -1};
                    ans[2] = j3;
                }
                else
                {
                    // theta < alpha
                    // w = theta'
                    struct HingeJacobian j3 = {true, iid, jid, 0, 0, -1, 0, 0, 1};
                    ans[2] = j3;
                }

                return ans;
            }
            else if (!flip && sin_ij < limit_angle_sin)
            {
                cout << "sin_ij: " << sin_ij << endl
                     << "cos_ij: " << cos_ij << endl;
                vector<HingeJacobian> ans(3);
                // alpha = -pi/2 - phi
                // beta  = -pi/2 + phi
                // sin(theta) > sin(alpha) = sin(beta)
                Vector2f ri = ibody->ri(r) - ibody->c;
                int iid = ibody->_getId();
                int jid = jbody->_getId();
                struct HingeJacobian j1 = {false, iid, jid, 1, 0, -ri(1), -1, 0, rj(1)};
                struct HingeJacobian j2 = {false, iid, jid, 0, 1, ri(0), 0, -1, -rj(0)};
                ans[0] = j1;
                ans[1] = j2;
                // alpha = pi/2 - phi
                // beta  = pi/2 + phi
                // sin(theta_ij) < sin(alpha) = sin(beta)
                if (cos_ij > 0)
                {
                    // theta < alpha
                    // w = theta'
                    struct HingeJacobian j3 = {true, iid, jid, 0, 0, -1, 0, 0, 1};
                    ans[2] = j3;
                }
                else
                {
                    // theta > beta
                    // w = -theta'
                    struct HingeJacobian j3 = {true, iid, jid, 0, 0, 1, 0, 0, -1};
                    ans[2] = j3;
                }

                return ans;
            }

            vector<HingeJacobian> ans(2);
            Vector2f ri = ibody->ri(r) - ibody->c;
            int iid = ibody->_getId();
            int jid = jbody->_getId();
            struct HingeJacobian j1 = {false, iid, jid, 1, 0, -ri(1), -1, 0, rj(1)};
            struct HingeJacobian j2 = {false, iid, jid, 0, 1, ri(0), 0, -1, -rj(0)};
            ans[0] = j1;
            ans[1] = j2;
            return ans;
        };

        int wsize() override
        {
            return 2;
        };

        void fix(float alpha, float beta) override
        {
            Vector2f rji = jbody->s - ibody->ri(r);
            Vector2f move = rji * alpha;
            ibody->slide(move);
            jbody->slide(-move);
        }
    };

    class Collision : Constraint
    {
    private:
        // 拘束対象となる二つの実体
        LineBody ibody, jbody;

    public:
        Collision(LineBody ibody, LineBody jbody)
            : ibody(ibody), jbody(jbody){};

        void check();
        vector<HingeJacobian> getJacobian() override{};
    };
}
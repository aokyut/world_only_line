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

        HingeJoint(LineBody *ibody, LineBody *jbody, float r) : ibody(ibody), jbody(jbody), r(r){};

        vector<HingeJacobian> getJacobian() override
        {
            vector<HingeJacobian> ans(2);
            Vector2f ri = ibody->ri(r) - ibody->c;
            Vector2f rj = jbody->s - jbody->c;
            int iid = ibody->_getId();
            int jid = jbody->_getId();
            struct HingeJacobian j1 = {iid, jid, 1, 0, -ri(1), -1, 0, rj(1)};
            struct HingeJacobian j2 = {iid, jid, 0, 1, ri(0), 0, -1, -rj(0)};
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
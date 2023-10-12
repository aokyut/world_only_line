#pragma once
#include "LineBody.h"

namespace physics
{
    enum CType
    {
        F, // 無拘束
        E, // 基本拘束
        C  // 相補拘束
    };

    class Constraint
    {
    protected:
    public:
        Constraint() {}
        virtual Matrix3f getJacobian() {}
    };

    // 回転を制限されない拘束
    class HingeJoint : public Constraint
    {
    public:
        LineBody *ibody, *jbody;
        float r;

        HingeJoint(LineBody *ibody, LineBody *jbody, float r) : ibody(ibody), jbody(jbody), r(r){};

        Matrix3f getJacobian() override{};
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
    };
}
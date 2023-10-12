#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <Eigen/Core>
#include "Bar.h"
using namespace Eigen;

namespace physics
{

    Matrix<float, 2, 2> rotateMat(float theta)
    {
        Matrix2f rotation;
        rotation << cosf(theta), -sinf(theta), sinf(theta), cosf(theta);
        return rotation;
    }

    const Matrix2f iMat()
    {
        Matrix2f imat;
        imat << 0, -1, 1, 0;
        return imat;
    }

    // 線分の物理的実体
    class LineBody
    {

    private:
        Bar bar;
        // 加速度
        Vector2f a;
        // 角加速度
        float aw;
        // 固定する
        bool fix;

    public:
        const float m;
        // 両端の座標
        Vector2f s, t;
        // 中心の座標
        Vector2f c;
        const float I;
        // 角速度
        float w;
        // 重心の速度
        Vector2f v;
        // デルタ
        const float delta;
        /// @param sx 始点のx座標
        /// @param sy 始点のy座標
        /// @param tx 終点のx座標
        /// @param ty 終点のy座標
        /// @param m 線分の重さ
        LineBody(float sx, float sy, float tx, float ty, float m, float delta)
            : m(m), s(Vector2f(sx, sy)), t(Vector2f(tx, ty)), I(((tx - sx) * (tx - sx) + (ty - sy) * (ty - sy)) * m / 12.0f), delta(delta), bar(Bar()), w(0), aw(0), fix(false)
        {
            // std::cout << "LineBody in" << sx << sy << tx << ty << m << delta << std::endl;
            // std::cout << "LineBody s and t " << s(0) << " " << s(1) << " " << t(0) << " " << t(1) << std::endl;
            c = 0.5f * (s + t);
            a << 0, 0;
            v << 0, 0;
        }

        /// @return 1ならば終点、0ならば始点となる直線上の座標
        Vector2f ri(float r)
        {
            return (1 - r) * s + r * t;
        }

        /// @param x 力が作用した点
        /// @param f 与えられる力
        void addForce(Vector2f x, Vector2f f)
        {
            // std::cout << "力1" << this << ": " << x << f << std::endl;
            Vector2f cx = x - c;
            float torque = cx(0) * f(1) - cx(1) * f(0);
            if (torque == 0.0f)
            {
                a += f / m;
                return;
            }
            cx.normalize();
            Vector2f centerForce = cx.dot(f) * cx;
            a += centerForce / m;
            aw += torque / I;
        }

        /// @param f 与えられる力
        void addForce(Vector2f f)
        {
            a += f / m;
            // std::cout << "力2 " << this << ": "
            //           << "\n"
            //           << a << std::endl;
            return;
        }

        /// @param dx 移動する差分
        void correctPos(Vector2f dx)
        {
            if (fix)
                return;
            s += dx;
            t += dx;
            c += dx;
        }

        void update()
        {
            // std::cout << "fix:" << fix << std::endl;
            if (fix)
            {
                v << 0, 0;
                a << 0, 0;
                w = 0;
                aw = 0;
                return;
            }
            v += delta * a;
            w += delta * aw;
            // std::cout << "加速度: \n"
            //           << a << "\n"
            //           << aw << std::endl;
            // 加速度をリセット
            a << 0, 0;
            aw = 0;

            // std::cout << "速度: \n"
            //           << v << "\n"
            //           << w << std::endl;

            c += delta * v;

            Matrix2f rotM = rotateMat(delta * w);
            s = rotM * (s + delta * v - c) + c;
            t = rotM * (t + delta * v - c) + c;

            // std::cout << "位置: \n"
            //           << c << "\n"
            //           << s << "\n"
            //           << t << "\n"
            //           << w << std::endl;
        }

        void draw()
        {
            // std::cout << "LineBody.draw " << s(0) << " " << s(1) << " " << t(0) << " " << t(1) << std::endl;
            bar.draw(s, t);
        }

        void setFix()
        {
            fix = true;
        }
    };

}
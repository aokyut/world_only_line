#pragma once
#include <memory>
#include "Object.h"
#include <Eigen/Core>

using namespace obj;
using namespace Eigen;

// 物理的状態を保持しない線の描画のためのクラス
class Bar
{
    std::shared_ptr<const Object> object;

public:
    Vertex p[4];
    Vertex dp[4];
    const GLfloat width;
    bool called;
    Bar(GLfloat width = 0.03f)
        : width(width), called(false)
    {
        // object.reset(new Object(2, 4, p));
    }

    // 描画
    void draw(Vertex s, Vertex t)
    {
        // objectを保持していない時
        // std::cout << "Bar::draw s:" << s.position[0] << s.position[1] << " t:" << t.position[0] << t.position[0] << std::endl;
        if (called == false)
        {
            called = true;
            Vertex center = (s + t) * 0.5;
            Vertex i = Vertex(0.0f, 1.0f);
            s = s - center;
            t = t - center;
            p[0] = s + (i * s / s.l2()) * width + center;
            p[1] = s + (i * s / s.l2()) * (-width) + center;
            p[2] = t + (i * t / t.l2()) * width + center;
            p[3] = t + (i * t / t.l2()) * (-width) + center;
            object.reset(new Object(2, 4, p));
            object->bind();
        }
        else
        {
            Vertex center = (s + t) * 0.5;
            Vertex i = Vertex(0.0f, 1.0f);
            s = s - center;
            t = t - center;
            p[0] = s + (i * s / s.l2()) * width + center;
            p[1] = s + (i * s / s.l2()) * (-width) + center;
            p[2] = t + (i * t / t.l2()) * width + center;
            p[3] = t + (i * t / t.l2()) * (-width) + center;
            object->bind();
            object->sub(4, p);
        }
        // 頂点配列オブジェクトを結合する
        // object->bind();
        // 描画を実行する
        execute();
    }

    void draw(Vector2f s, Vector2f t)
    {
        draw(Vertex(s(0), s(1)), Vertex(t(0), t(1)));
    }
    // 描画の実行
    virtual void execute() const
    {
        // 折れ線で描画する
        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    }
};
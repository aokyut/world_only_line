#pragma once
#include <memory>
#include "Object.h"

using namespace obj;

class Shape
{
    std::shared_ptr<const Object> object;

protected:
    // 描画に使う頂点の数
    const GLsizei vertexcount;

public:
    // コンストラクタ
    // size: 頂点の位置の次元
    // vertexcount: 頂点の数
    // vertex: 頂点属性を格納した配列
    Shape(GLint size, GLsizei vertexcount, const Vertex *vertex)
        : vertexcount(vertexcount)
    {
        object.reset(new Object(size, vertexcount, vertex));
    }

    // 描画
    void draw()
    {
        // 頂点配列オブジェクトを結合する
        object->bind();
        // 描画を実行する
        execute();
    }
    // 描画の実行
    virtual void execute() const
    {
        // 折れ線で描画する
        glDrawArrays(GL_TRIANGLE_FAN, 0, vertexcount);
    }

    void update(const Vertex *vertex)
    {
        object->sub(vertexcount, vertex);
    }
};
#pragma once
#include <iostream>
#include <GL/glew.h>
#include <math.h>

// 図形データ
namespace obj
{

    struct Vertex
    {
        GLfloat position[2];
        Vertex(float x, float y)
        {
            position[0] = x;
            position[1] = y;
        }
        Vertex()
        {
            Vertex(0.0f, 0.0f);
        }

        Vertex operator+(const Vertex &other) const
        {
            Vertex ans;
            ans.position[0] = position[0] + other.position[0];
            ans.position[1] = position[1] + other.position[1];
            return ans;
        }

        Vertex operator-(const Vertex &other) const
        {
            Vertex ans;
            ans.position[0] = position[0] - other.position[0];
            ans.position[1] = position[1] - other.position[1];
            return ans;
        }

        // 掛け算は複素数として計算
        Vertex operator*(const Vertex &other) const
        {
            Vertex ans;
            ans.position[0] = position[0] * other.position[0] - position[1] * other.position[1];
            ans.position[1] = position[0] * other.position[1] + position[1] * other.position[0];
            return ans;
        }

        Vertex operator*(const GLfloat &other) const
        {
            Vertex ans;
            ans.position[0] = position[0] * other;
            ans.position[1] = position[1] * other;
            return ans;
        }

        Vertex operator/(const GLfloat &other) const
        {
            Vertex ans;
            ans.position[0] = position[0] / other;
            ans.position[1] = position[1] / other;
            return ans;
        }

        GLfloat l2() const
        {
            return sqrt(position[0] * position[0] + position[1] * position[1]);
        }
    };
    static Vertex zero()
    {
        Vertex zerov;
        zerov.position[0] = 0.0f;
        zerov.position[1] = 0.0f;
        return zerov;
    }

    class Object
    {
        GLuint vao;
        GLuint vbo;

    public:
        // コンストラクタ
        // size: 頂点の位置の次元
        // vertexcount: 頂点の数
        // vertex: 頂点属性を格納した配列

        Object(GLint size, GLsizei vertexcount, const Vertex *vertex)
        {
            for (int i = 0; i < vertexcount; i++)
            {
                std::cout << vertex[i].position[0] << vertex[i].position[1] << std::endl;
            }

            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);

            glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferData(GL_ARRAY_BUFFER, vertexcount * sizeof(vertex), vertex, GL_DYNAMIC_DRAW);

            // 結合されている頂点バッファオブジェクトを in 変数から参照できるようにする
            glVertexAttribPointer(0, size, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(0);

            std::cout << "vao : " << vao << std::endl;
        }

        virtual ~Object()
        {
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
        }

        void bind() const
        {
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBindVertexArray(vao);
        }

        void sub(GLsizei vertexcount, const Vertex *vertex) const
        {
            // for (int i = 0; i < vertexcount; i++)
            // {
            //     std::cout << "vertex " << i << " " << vertex[i].position[0] << " " << vertex[i].position[1] << std::endl;
            // }
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertexcount * sizeof(vertex), vertex);
        }

    private:
        // コピーコンストラクタによるコピー禁止
        Object(const Object &o);
        // 代入によるコピー禁止
        Object &operator=(const Object &o);
    };
}
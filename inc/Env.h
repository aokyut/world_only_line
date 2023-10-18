#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include "World.h"
#include "Window.h"
#include "LineBody.h"
#include "Constraint.h"

namespace physics
{
    GLboolean printProgramInfoLog(GLuint program)
    {
        // リンク結果を取得する
        GLint status;
        glGetProgramiv(program, GL_LINK_STATUS, &status);
        if (status == GL_FALSE)
            std::cerr << "Link Error." << std::endl;
        // シェーダのリンク時のログの長さを取得する
        GLsizei bufSize;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufSize);
        if (bufSize > 1)
        {
            // シェーダのリンク時のログの内容を取得する
            std::vector<GLchar> infoLog(bufSize);
            GLsizei length;
            glGetProgramInfoLog(program, bufSize, &length, &infoLog[0]);
            std::cerr << &infoLog[0] << std::endl;
        }
        return static_cast<GLboolean>(status);
    }
    GLboolean printShaderInfoLog(GLuint shader, const char *str)
    {
        GLint status;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
        if (status == GL_FALSE)
            std::cerr << "Compile Error in " << str << std::endl;

        // シェーダのコンパイル時のログの長さを取得する
        GLsizei bufSize;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &bufSize);

        if (bufSize > 1)
        {
            // シェーダのコンパイル時のログの内容を取得する
            std::vector<GLchar> infoLog(bufSize);
            GLsizei length;
            glGetShaderInfoLog(shader, bufSize, &length, &infoLog[0]);
            std::cerr << &infoLog[0] << std::endl;
        }

        return static_cast<GLboolean>(status);
    }

    GLuint createProgram(const char *vsrc, const char *fsrc)
    {
        const GLuint program(glCreateProgram());

        if (vsrc != NULL)
        {
            // バーテックスシェーダのシェーダオブジェクトを作成する
            const GLuint vobj(glCreateShader(GL_VERTEX_SHADER));
            glShaderSource(vobj, 1, &vsrc, NULL);
            glCompileShader(vobj);

            // バーテックスシェーダのシェーダオブジェクトをプログラムに組み込む
            if (printShaderInfoLog(vobj, "vertex shader"))
                glAttachShader(program, vobj);
            glDeleteShader(vobj);
        }

        if (fsrc != NULL)
        {
            // フラグメントシェーダの作成
            const GLuint fobj(glCreateShader(GL_FRAGMENT_SHADER));
            glShaderSource(fobj, 1, &fsrc, NULL);
            glCompileShader(fobj);

            // フラグメントシェーダのシェーダオブジェクトをプログラムに組み込む
            if (printShaderInfoLog(fobj, "fragment shader"))
                glAttachShader(program, fobj);
            glDeleteShader(fobj);
        }

        // プログラムオブジェクトをリンク
        glBindAttribLocation(program, 0, "position");
        glBindFragDataLocation(program, 0, "fragment");
        glLinkProgram(program);

        // 作成したプログラムを返す
        if (printProgramInfoLog(program))
            return program;

        // プログラムオブジェクトが作成できなければ0を返す
        glDeleteProgram(program);
        return 0;
    }

    // name: シェーダのソースファイル名
    // buffer: 読み込んだソースファイルのテキスト
    bool readShaderSource(const char *name, std::vector<GLchar> &buffer)
    {
        // ファイル名が NULL だった
        if (name == NULL)
            return false;
        // ソースファイルを開く
        std::ifstream file(name, std::ios::binary);
        if (file.fail())
        {
            // 開けなかった
            std::cerr << "Error: Can't open source file: " << name << std::endl;
            return false;
        }

        // ファイルの末尾に移動し現在位置（＝ファイルサイズ）を得る
        file.seekg(0L, std::ios::end);
        GLsizei length = static_cast<GLsizei>(file.tellg());

        // ファイルサイズのメモリを確保
        buffer.resize(length + 1);

        // ファイルを先頭から読み込む
        file.seekg(0L, std::ios::beg);
        file.read(buffer.data(), length);
        buffer[length] = '\0';

        if (file.fail())
        {
            // うまく読み込めなかった
            std::cerr << "Error: Could not read souce file: " << name << std::endl;
            file.close();
            return false;
        }

        // 読み込み成功
        file.close();
        return true;
    }

    GLuint loadProgram(const char *vert, const char *frag)
    {
        // シェーダのソースファイルを読み込む
        std::vector<GLchar> vsrc;
        const bool vstat(readShaderSource(vert, vsrc));
        std::vector<GLchar> fsrc;
        const bool fstat(readShaderSource(frag, fsrc));
        // プログラムオブジェクトを作成する
        return vstat && fstat ? createProgram(vsrc.data(), fsrc.data()) : 0;
    }

    class Env
    {
    private:
        World *world;
        Window *window;
        // std::optional<Window> window;
        bool isOpenWindow;
        int tStep;
        GLuint program;
        GLint sizeLoc;
        GLint scaleLoc;
        GLint locationLoc;
        float cameraPosX;
        float cameraPosY;
        list<LineBody> bodies;
        list<Constraint> joints;

    public:
        Vector2f g = Vector2f(3, -0.5);

        Env() : world(new World())
        {
        }

        void reset()
        {
            world->reset();
            tStep = 0;
        }

        void testEnvSet2()
        {
            // snake
            physics::LineBody *line1 = new physics::LineBody(0, 1, 0, 0.5, 1);
            physics::LineBody *line2 = new physics::LineBody(0, 0.5, 0.5, 0.5, 1);
            physics::LineBody *line3 = new physics::LineBody(0.5, 0.5, 1, 0.5, 1);
            physics::LineBody *line4 = new physics::LineBody(1, 0.5, 1.5, 0.5, 1);
            physics::LineBody *line5 = new physics::LineBody(1.5, 0.5, 2.0, 0.5, 1);
            physics::LineBody *line6 = new physics::LineBody(2.0, 0.5, 5, 0.5, 1);
            // line1.setFix();
            physics::HingeJoint *c1 = new physics::HingeJoint(line1, line2, 1.0f);
            physics::HingeJoint *c2 = new physics::HingeJoint(line2, line3, 1.0f);
            physics::HingeJoint *c3 = new physics::HingeJoint(line3, line4, 1.0f);
            physics::HingeJoint *c4 = new physics::HingeJoint(line4, line5, 1.0f);
            physics::HingeJoint *c5 = new physics::HingeJoint(line5, line6, 1.0f);

            line1->setFix();

            world->addBody(line1);
            world->addBody(line2);
            world->addBody(line3);
            world->addBody(line4);
            world->addBody(line5);
            world->addBody(line6);
            world->addJoint(c1);
            world->addJoint(c2);
            world->addJoint(c3);
            world->addJoint(c4);
            world->addJoint(c5);
        }

        void testEnvSet()
        {
            physics::LineBody *line1 = new physics::LineBody(0, 0, 0, 1, 1);
            physics::LineBody *line2 = new physics::LineBody(0, 1, 1, 1, 1);
            physics::LineBody *line3 = new physics::LineBody(1, 1, 1, 0, 1);
            physics::LineBody *line4 = new physics::LineBody(1, 0, 0, 0, 1);
            physics::LineBody *line5 = new physics::LineBody(0, 0, 1, 1, 1);
            physics::HingeJoint *c1 = new physics::HingeJoint(line1, line2, 1.0f);
            physics::HingeJoint *c2 = new physics::HingeJoint(line2, line3, 1.0f);
            physics::HingeJoint *c3 = new physics::HingeJoint(line3, line4, 1.0f);
            physics::HingeJoint *c4 = new physics::HingeJoint(line4, line1, 1.0f);
            physics::HingeJoint *c5 = new physics::HingeJoint(line4, line5, 1.0f);
            physics::HingeJoint *c6 = new physics::HingeJoint(line5, line3, 1.0f);

            world->addBody(line1);
            world->addBody(line2);
            world->addBody(line3);
            world->addBody(line4);
            world->addBody(line5);
            world->addJoint(c1);
            world->addJoint(c2);
            world->addJoint(c3);
            world->addJoint(c4);
            world->addJoint(c5);
            world->addJoint(c6);

            world->show();
        }

        void show()
        {
            world->show();
        }

        void step(vector<float> actions)
        {
            world->addAccelToAll(g);
            world->step();
            tStep++;
        }

        void render()
        {
            if (isOpenWindow == false)
            {
                openWindow();
                world->resetShader();
                isOpenWindow = true;
            }

            // cout << (window) << endl;

            if (*window)
            {
                if (world->hasBody())
                {
                    vector<float> c = world->center();
                    cameraPosX = c[0];
                    cameraPosY = c[1];
                }

                window->setCameraPos(cameraPosX, cameraPosY);

                glClear(GL_COLOR_BUFFER_BIT);

                glUseProgram(program);

                glUniform2fv(sizeLoc, 1, window->getSize());
                glUniform1f(scaleLoc, window->getScale());
                glUniform2fv(locationLoc, 1, window->getLocation());
                world->render();
                window->swapBuffers();
            }
            else
            {
                closeWindow();
                isOpenWindow = false;
            }
        }

        void openWindow()
        {
            if (glfwInit() == GL_FALSE)
            {
                std::cerr << "Can't initialize GLFW" << std::endl;
                return;
            }

            atexit(glfwTerminate);

            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

            window = new Window();

            // init glew
            glewExperimental = GL_TRUE;
            if (glewInit() != GLEW_OK)
            {
                std::cerr << "Can't initialize GLEW" << std::endl;
            }

            // set background color
            glClearColor(1.0f, 1.0f, 1.0f, 0.1f);

            program = (loadProgram("point.vert", "point.frag"));

            sizeLoc = (glGetUniformLocation(program, "size"));
            scaleLoc = (glGetUniformLocation(program, "scale"));
            locationLoc = (glGetUniformLocation(program, "location"));

            std::cout << "glGetUniformLocation size : " << sizeLoc << std::endl;
            std::cout << "glGetUniformLocation scale : " << scaleLoc << std::endl;
            std::cout << "glGetUniformLocation location : " << locationLoc << std::endl;
        }

        void closeWindow()
        {
            glfwTerminate();
        }
    };
}
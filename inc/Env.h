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
        float torqueScale = 20.0;
        vector<LineAgent> agents;
        int bodyNum = 0;
        int jointNum = 0;

    public:
        Vector2f g = Vector2f(3, -0.5);

        Env() : world(new World())
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
        /// @return obs(bodyNum, {center_y, angle_vec(2), velocities(3)}(6))
        MatrixXf getObservationBody()
        {
            MatrixXf obs(bodyNum, 6);
            int tar_i = 0;
            for (LineAgent agent : agents)
            {
                for (LineBody *body : agent.bodies)
                {
                    Vector2f angle_vec = (body->t - body->s).normalized();
                    float center_y = body->c(1);
                    Vector2f vel = body->v;
                    float w = body->w;
                    VectorXf obs_row(6);
                    obs_row << center_y, angle_vec, vel, w;
                    obs.row(tar_i) = obs_row;
                    tar_i++;
                }
            }
            return obs;
        }

        /// @brief get joint observation
        /// @return obs(jointNum, {Joint.r, Joint.lim_angle_sin, angle(2)}(2))
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
            VectorXf vertical(4);
            max_center_x = std::floorf(2 * max_center_x) / 2;
            vertical << max_center_x, world->floor_y, max_center_x, 1000;
            lines.row(tar_idx) = floor;
            lines.row(tar_idx + 1) = vertical;

            return lines;
        }

        void endAgentAssemble()
        {
            agents[agents.size() - 1].tweak(world);
        }

        void render()
        {
            // cout << "isOpenWindow" << isOpenWindow << endl;
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
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Shape.h"
#include "Window.h"
#include "Bar.h"
#include "World.h"

// プログラムオブジェクトのリンク結果を表示する
// program: プログラムオブジェクト名
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

// シェーダのソースファイルを読み込んだメモリを返す
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

// シェーダのソースファイルを読み込んでプログラムオブジェクトを作成する
// vert: バーテックスシェーダのソースファイル名
// frag: フラグメントシェーダのソースファイル名
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

// obj::Vertex rectangleVertex[] =
//     {
//         {-1.1f, -0.5f},
//         {0.5f, -0.5f},
//         {0.5f, 0.5f},
//         {-0.5f, 0.5f}};

int main(void)
{
    // GLFWの初期化
    if (glfwInit() == GL_FALSE)
    {
        std::cerr << "Can't initialize GLFW" << std::endl;
        return 1;
    }

    // プログラム終了時のの処理を登録
    atexit(glfwTerminate);

    // OpenGL Version 3.2 Core Profile を選択する
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // ウィンドウの初期化
    Window window;

    // 作成したウィンドウをOpenGLの処理対象にする
    // glfwMakeContextCurrent(window);

    // GLEWを初期化する
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        // 初期化に失敗したとき
        std::cerr << "Can't initialize GLEW" << std::endl;
    }

    // ビューポートの設定
    // glViewport(100, 50, 300, 300);

    // 背景色の指定
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // シェーダプログラムの作成
    const GLuint program(loadProgram("point.vert", "point.frag"));

    // uniform変数の場所を取得する
    const GLint sizeLoc(glGetUniformLocation(program, "size"));
    const GLint scaleLoc(glGetUniformLocation(program, "scale"));
    const GLint locationLoc(glGetUniformLocation(program, "location"));
    std::cout << "glGetUniformLocation size : " << sizeLoc << std::endl;
    std::cout << "glGetUniformLocation scale : " << scaleLoc << std::endl;
    std::cout << "glGetUniformLocation location : " << locationLoc << std::endl;

    // 図形オブジェクトの作成
    // std::unique_ptr<Shape>
    //     shape(new Shape(2, 4, rectangleVertex));
    // std::cout << "ok" << std::endl;
    // // Bar bar(Bar(1.0f, 0.0f, 0.0f, 1.0f, 0.1f));
    // std::unique_ptr<Bar> bar(new Bar());
    // obj::Vertex dp = {0.0f, -0.001f};
    float delta = 0.01f;
    int idling = 10;
    physics::World world = physics::World();
    physics::LineBody line1 = physics::LineBody(0, 1, 0, 0.5, 1000);
    physics::LineBody line2 = physics::LineBody(0, 0.5, 0.5, 0.5, 1);
    physics::LineBody line3 = physics::LineBody(0.5, 0.5, 1, 0.5, 1);
    physics::LineBody line4 = physics::LineBody(1, 0.5, 1.5, 0.5, 1);
    physics::LineBody line5 = physics::LineBody(1.5, 0.5, 2.0, 0.5, 1);
    physics::LineBody line6 = physics::LineBody(2.0, 0.5, 2.5, 0.5, 1);
    line1.setFix();
    physics::HingeJoint c1 = physics::HingeJoint(&line1, &line2, 1.0f);
    physics::HingeJoint c2 = physics::HingeJoint(&line2, &line3, 1.0f);
    physics::HingeJoint c3 = physics::HingeJoint(&line3, &line4, 1.0f);
    physics::HingeJoint c4 = physics::HingeJoint(&line4, &line5, 1.0f);
    physics::HingeJoint c5 = physics::HingeJoint(&line5, &line6, 1.0f);
    world.addBody(&line1);
    world.addBody(&line2);
    world.addBody(&line3);
    world.addBody(&line4);
    world.addBody(&line5);
    world.addBody(&line6);
    world.addJoint(&c1);
    world.addJoint(&c2);
    world.addJoint(&c3);
    world.addJoint(&c4);
    world.addJoint(&c5);

    Vector2f x, f;
    x << 0, -0.25;
    f << 0, -1;
    // ウィンドウが開いているかぎり繰り返す
    int step = 0;
    while (window)
    {
        // step++;
        // if (step > 1)
        // {
        //     break;
        // }
        // ウィンドウを消去
        glClear(GL_COLOR_BUFFER_BIT);

        // シェーダプログラムの使用開始
        glUseProgram(program);

        // uniform変数に値を設定する
        glUniform2fv(sizeLoc, 1, window.getSize());
        glUniform1f(scaleLoc, window.getScale());
        glUniform2fv(locationLoc, 1, window.getLocation());

        //
        // 描画処理
        // rectangleVertex[0].position[0] += 0.01f;
        // // std::cout << rectangleVertex[0].position[0] << std::endl;
        // Vertex s = {rectangleVertex[0].position[0] + 0.5f, -1.0f};
        // Vertex t = {-1.0f, -2.0f};
        // bar->draw(s, t);
        for (int i = 0; i < idling; i++)
        {
            line2.addForce(f / idling);
            line3.addForce(f / idling);
            line4.addForce(f / idling);
            line5.addForce(f / idling);
            line6.addForce(f / idling);
            world.step();
        }
        world.render();

        // shape->draw();
        // shape->update(rectangleVertex);
        //

        // カラーバッファを入れ替える
        window.swapBuffers();
    }
}
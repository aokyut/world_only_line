#pragma once
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// ウィンドウ関連の処理
class Window
{
    // ウィンドウのハンドル
    GLFWwindow *const window;

    // ウィンドウのサイズ
    GLfloat size[2];

    // ワールド座標系に対するデバイス座標系の拡大率
    GLfloat scale;

    // 図形の正規化デバイス上での座標
    GLfloat location[2];

    // キーの状態
    int keystatus;

public:
    // コンストラクタ
    Window(int width = 640, int height = 480, const char *title = "Hello!")
        : window(glfwCreateWindow(width, height, title, NULL, NULL)), scale(100.0f), keystatus(GLFW_RELEASE)
    {
        location[0] = 0.0f;
        location[1] = 0.0f;
        if (window == NULL)
        {
            // ウィンドウが作成できなかった
            std::cerr << "Can't create GLFW window." << std::endl;
            exit(1);
        }
        // 現在のウィンドウを処理対象にする
        glfwMakeContextCurrent(window);
        // GLEW を初期化する
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK)
        {
            // GLEW の初期化に失敗した
            std::cerr << "Can't initialize GLEW" << std::endl;
            exit(1);
        }
        // 垂直同期のタイミングを待つ
        glfwSwapInterval(1);

        // マウスホイールを操作した際に呼び出す処理の登録
        glfwSetScrollCallback(window, wheel);

        // このインスタンスのthisポインタを記録しておく
        glfwSetWindowUserPointer(window, this);

        // ウィンドウのサイズ変更時に呼び出す処理の登録
        glfwSetWindowSizeCallback(window, resize);
        // 開いたウィンドウの初期設定
        resize(window, width, height);

        std::cout << "Window Constractor Called" << std::endl;
    }

    // デストラクタ
    virtual ~Window()
    {
        glfwDestroyWindow(window);
    }
    // 描画ループの継続判定
    explicit operator bool()
    {
        // イベントを取り出す
        if (keystatus == GLFW_RELEASE)
        {
            // glfwWaitEvents();
            glfwPollEvents();
        }
        else
        {
            glfwPollEvents();
        }

        // マウスの左ボタンの状態を調べる
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) != GLFW_RELEASE)
        {
            // マウスカーソルの位置を取得
            double x, y;
            glfwGetCursorPos(window, &x, &y);

            // マウスカーソルの正規化デバイス座標上での位置を求める
            location[0] = static_cast<GLfloat>(x) * 2.0f / size[0] - 1.0f;
            location[1] = 1.0f - static_cast<GLfloat>(y) * 2.0f / size[1];
        }

        // ウィンドウを閉じる必要がなければ true を返す
        return !glfwWindowShouldClose(window);
    }
    // ダブルバッファリング
    void swapBuffers() const
    {
        // カラーバッファを入れ替える
        glfwSwapBuffers(window);
    }

    static void resize(GLFWwindow *const window, int width, int height)
    {
        // フレームバッファのサイズを調べる
        int fbWidth, fbHeight;
        glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
        // フレームバッファ全体をビューポートに設定する
        glViewport(0, 0, fbWidth, fbHeight);

        // このインスタンスのthisポインタを得る
        Window *const instance(static_cast<Window *>(glfwGetWindowUserPointer(window)));

        if (instance != NULL)
        {
            // 開いたウィンドウのサイズを調整する
            instance->size[0] = static_cast<GLfloat>(width);
            instance->size[1] = static_cast<GLfloat>(height);
        }
    }

    static void wheel(GLFWwindow *const window, double x, double y)
    {
        Window *const instance(static_cast<Window *>(glfwGetWindowUserPointer(window)));

        if (instance != NULL)
        {
            instance->scale += static_cast<GLfloat>(y);
        }
    }

    //ウィンドウのサイズを取り出す
    const GLfloat *getSize() const { return size; }

    // ワールド座標系に対するデバイスの拡大率を取り出す。
    GLfloat getScale() const { return scale; }

    // 位置を取り出す
    const GLfloat *getLocation() const { return location; }
};
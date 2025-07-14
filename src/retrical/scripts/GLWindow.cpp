#include "GLWindow.h"
#include <exception>

std::map<SDL_Window*, GLWindow*> GLWindow::windows;
static bool gladLoaded = false;

const static std::string vertexShader =
R"(#version 130
const vec4 pos[3] = vec4[3](
    vec4(-3,-1,0,1),
    vec4(+1,-1,0,1),
    vec4(+1,+3,0,1)
);

const vec2 tex[3] = vec2[3](
    vec2(-1,0),
    vec2(1,0),
    vec2(1,-2)
);

out vec2 texcoord;

void main(void)
{
    gl_Position = pos[gl_VertexID % 3];
    texcoord    = tex[gl_VertexID % 3];
}
)";

const static std::string fragmentShader =
R"(#version 130
in vec2 texcoord;
out vec4 color;
uniform sampler2D image;
uniform mat4 colormat;

void main(void)
{
    color = texture(image, texcoord) * colormat;
}
)";

static void check_compile_error(GLuint shader)
{
    int compiled;
    int size;
    char * text;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &size);
        text = new char[size];
        glGetShaderInfoLog(shader, size, &size, text);
        printf("%s", text);
        abort();
    }
}

static void check_link_error(GLuint program)
{
    int linked;
    int size;
    char * text;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &size);
        text = new char[size];
        glGetProgramInfoLog(program, size, &size, text);
        printf("%s", text);
        abort();
    }
}

GLWindow::GLWindow(int x, int y)
{
    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    window = SDL_CreateWindow("", x, y, 1280, 400, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

    context = SDL_GL_CreateContext(window);
    mutex = SDL_CreateMutex();

    SDL_LockMutex(mutex);
    windows[window] = this;

    SDL_GL_MakeCurrent(window, context);

    if (!gladLoaded)
    {
        gladLoadGLLoader(SDL_GL_GetProcAddress);
        gladLoaded = true;
    }

    // Do load steps here.

    glClearColor(0, 0, 0, 1);

    // Create shader program.
    GLuint vs, fs;
    int length;
    const char *shader;
    int error;
    vs = glCreateShader(GL_VERTEX_SHADER);
    fs = glCreateShader(GL_FRAGMENT_SHADER);

    length = vertexShader.length();
    shader = vertexShader.c_str();
    glShaderSource(vs, 1, &shader, &length);
    glCompileShader(vs);
    check_compile_error(vs);

    length = fragmentShader.length();
    shader = fragmentShader.c_str();
    glShaderSource(fs, 1, &shader, &length);
    glCompileShader(fs);
    check_compile_error(fs);

    blitShaderProgram = glCreateProgram();
    glAttachShader(blitShaderProgram, vs);
    glAttachShader(blitShaderProgram, fs);
    glLinkProgram(blitShaderProgram);
    check_link_error(blitShaderProgram);

    loc_colormat = glGetUniformLocation(blitShaderProgram, "colormat");

    // Create and bind default vao.
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Set blit texture to zero (to be determined :))
    blitTexture = 0;

    glClear(GL_COLOR_BUFFER_BIT);

    SDL_GL_SetSwapInterval(0);
    SDL_GL_SwapWindow(window);

    SDL_UnlockMutex(mutex);
}

GLWindow::~GLWindow()
{
    SDL_LockMutex(mutex);
    windows.erase(window);
    SDL_UnlockMutex(mutex);

    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_DestroyMutex(mutex);

    SDL_QuitSubSystem(SDL_INIT_EVERYTHING);
}

void GLWindow::setTitle(const char * title)
{
    SDL_SetWindowTitle(window, title);
}

void GLWindow::updateImage(const void *image, int width, int height, GLenum format)
{
    SDL_LockMutex(mutex);
    SDL_GL_MakeCurrent(window, context);

    switch (format)
    {
        default:
            channels = 4;
            break;
        case GL_RED:
            channels = 1;
            break;
    }

    if (width != this->width || height != this->height || blitTexture == 0)
    {
        // Create a new texture to use.
        if (blitTexture != 0)
            glDeleteTextures(1, &blitTexture);

        glGenTextures(1, &blitTexture);
        glBindTexture(GL_TEXTURE_2D, blitTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        this->width = width;
        this->height = height;
    }

    // Update texture.
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, GL_UNSIGNED_BYTE, image);

    // Rerender the window contents.
    updateWindow();
    SDL_UnlockMutex(mutex);
}

void GLWindow::processEvent(SDL_Event &ea)
{
    if (ea.type != SDL_WINDOWEVENT) return;

    switch (ea.window.event)
    {
    case SDL_WINDOWEVENT_RESIZED:
        SDL_LockMutex(mutex);
        updateWindow();
        SDL_UnlockMutex(mutex);
        break;
    case SDL_WINDOWEVENT_CLOSE:
        quit = true;
    }
}

void GLWindow::processEvents()
{
    SDL_Event ea;
    for (int i = 0; i < 64 && SDL_PollEvent(&ea); i++)
    {
        if (ea.type == SDL_WINDOWEVENT)
        {
            SDL_Window *wnd = SDL_GetWindowFromID(ea.window.windowID);
            if (windows.find(wnd) != windows.end())
            {
                windows[wnd]->processEvent(ea);
            }
        }
    }
}

void GLWindow::redraw()
{
    SDL_LockMutex(mutex);
    updateWindow();
    SDL_UnlockMutex(mutex);
}

void GLWindow::updateWindow()
{
    SDL_GL_MakeCurrent(window, context);
    glClear(GL_COLOR_BUFFER_BIT);

    glBindTexture(GL_TEXTURE_2D, blitTexture);
    glUseProgram(blitShaderProgram);

    int x, y, w, h;
    {
        // Figure out the view port parameters
        int ww, wh;
        SDL_GetWindowSize(window, &ww, &wh);
        float aspect = (float)width/height;

        if (wh * aspect > ww)
        {
            // Texture is too wide.
            x = 0;
            w = ww;
            h = ww / aspect;
            y = (wh - h) / 2;
        }
        else
        {
            // Texture is too tall.
            y = 0;
            h = wh;
            w = wh * aspect;
            x = (ww - w) / 2;
        }
    }
    glViewport(x,y,w,h);

    float matrix[16];
    memset(matrix, 0, sizeof(matrix));
    switch (channels)
    {
        case 1:
            matrix[0 + 0 * 4] = 1.0f;
            matrix[0 + 1 * 4] = 1.0f;
            matrix[0 + 2 * 4] = 1.0f;
            matrix[0 + 3 * 4] = 1.0f;
            break;
        default:
        case 3:
        case 4:
            matrix[0 + 0 * 4] = 1.0f;
            matrix[1 + 1 * 4] = 1.0f;
            matrix[2 + 2 * 4] = 1.0f;
            matrix[3 + 3 * 4] = 1.0f;
            break;
    }

    glUniformMatrix4fv(loc_colormat, 1, false, matrix);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    SDL_GL_SwapWindow(window);
}

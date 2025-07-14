#ifndef _GLWINDOW_H_
#define _GLWINDOW_H_
// Requires SDL2 and GL 3.0
#include "gl/include/glad/glad.h"
#include "SDL2/SDL.h"
#include <string>
#include <map>

class GLWindow
{
public:
    SDL_Window *window;
    bool quit = false;
    GLWindow(int x = SDL_WINDOWPOS_CENTERED, int y = SDL_WINDOWPOS_CENTERED);
    ~GLWindow();
    void setTitle(const char *title);
    void setTitle(std::string &title) { setTitle(title.c_str()); }
    void updateImage(const void *image, int width, int height, GLenum format);
    void redraw();

    static void processEvents();
    void processEvent(SDL_Event &ea);
private:
    SDL_GLContext context;
    SDL_mutex *mutex;
    GLuint blitShaderProgram;
    GLuint blitTexture;
    int width;
    int height;
    int channels;
    int loc_colormat;
    void updateWindow();

    static std::map<SDL_Window*, GLWindow*> windows;
};

#endif

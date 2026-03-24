#ifndef MOTIONCONTROL_LOAD_SHADER_H_
#define MOTIONCONTROL_LOAD_SHADER_H_
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
GLuint LoadShaders(const char * vertex_file_path,const char * fragment_file_path);
#endif //MOTIONCONTROL_LOAD_SHADER_H_

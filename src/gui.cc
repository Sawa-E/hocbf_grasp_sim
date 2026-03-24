#include "gui.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// camera
glm::vec3 cameraPos   = glm::vec3(0.4f, 0.4f, 8.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);
float fov   =  80.0f;
float deltaTime = 0.0f;// time between current frame and last frame
const float vertices[] = {};

GLFWwindow *reset_and_open_window(const std::string window_name)
{
  //_monitor =  glfwGetPrimaryMonitor();
  const char* glsl_version = "#version 130";
  if (!glfwInit())
      exit(1);
  const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
  static GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, window_name.c_str(), NULL, NULL);
  if(window == nullptr){
    exit(1);
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  /*
  Set Key callback for dynamic break or immediate stop
  */
  glfwSetKeyCallback(window, key_callback);

  bool err = gl3wInit() != 0;
  if (err)
  {
      fprintf(stderr, "Failed to initialize OpenGL loader!\n");
      exit(1);
      //return;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  //ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  auto io = ImGui::GetIO(); //(void)io_;
  //ここでフォントを変更する
  io.Fonts->AddFontFromFileTTF("../config/fonts/Roboto-Medium.ttf", 16.0f, NULL);
  return window;
}

void initialize_gui(GLFWwindow *window, unsigned int &VAO, unsigned int &VBO, unsigned int &texture, GLuint &shaderProgram)
{
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetCursorPosCallback(window, mouse_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);

  glEnable(GL_DEPTH_TEST);
  //const GLuint shaderProgram(LoadShaders("point.vert","point.frag"));
#ifndef PCI_MODE
  shaderProgram = LoadShaders("../config/opengl/point.vert","../config/opengl/point.frag");
#endif
#ifdef PCI_MODE
  (void)VAO;
  (void)VBO;
  (void)texture;
  (void)shaderProgram;
#endif
  // set up vertex data (and buffer(s)) and configure vertex attributes
  // ------------------------------------------------------------------
  //unsigned int VBO, VAO;
#ifndef PCI_MODE
  unsigned int EBO;
  set_vertex(VAO, VBO, EBO);
  set_texture(texture, "../config/texture/container.jpg");

  glUseProgram(shaderProgram);
  glUniform1i(glGetUniformLocation(shaderProgram, "texture1"), 0);
#endif
}

void clean_up_window(GLFWwindow *window)
{
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

void toggle_button(const char* str_id, bool* v)
{
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  float height = ImGui::GetFrameHeight();
  float width = height * 1.55f;
  float radius = height * 0.50f;

  if (ImGui::InvisibleButton(str_id, ImVec2(width, height)))
      *v = !*v;
  ImU32 col_bg;
  if (ImGui::IsItemHovered())
    col_bg = *v ? IM_COL32(145+20, 211, 68+20, 255) : IM_COL32(218-20, 218-20, 218-20, 255);
  else
    col_bg = *v ? IM_COL32(145, 211, 68, 255) : IM_COL32(218, 218, 218, 255);

  draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.5f);
  draw_list->AddCircleFilled(ImVec2(*v ? (p.x + width - radius) : (p.x + radius), p.y + radius), radius - 1.5f, IM_COL32(255, 255, 255, 255));
}

void process_input(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    float cameraSpeed = 2.5 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    (void)window;
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    (void)window;
    (void)xpos;
    (void)ypos;
}

void set_vertex(unsigned int &VAO, unsigned int &VBO,unsigned int &EBO)
{
  (void)EBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // position attribute
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);
  // texture coord attribute
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
}

void set_texture(unsigned int &texture, const std::string file_name)
{
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  int width, height, nrChannels;
  stbi_set_flip_vertically_on_load(true);
  unsigned char *data = stbi_load(file_name.c_str(), &width, &height, &nrChannels, 0);
  if (data){
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
      glGenerateMipmap(GL_TEXTURE_2D);
  }
  else{
      std::cout << "Failed to load texture" << std::endl;
  }
  stbi_image_free(data);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)window;
    (void)xoffset;
    fov -= yoffset;
}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    (void)window;
    (void)button;
    (void)action;
    (void)mods;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)scancode ;
    (void)mods ;
    if (key == GLFW_KEY_Q && action == GLFW_PRESS)
    {
      std::cout << "Q key!!" << std::endl ;
      glfwSetWindowShouldClose(window, GL_TRUE);
      std::cout << "Close !!" << std::endl ;
    }
}

void gui::show_widgets(const std::string &widget_name)
{
  IM_ASSERT(ImGui::GetCurrentContext() != NULL && "Missing dear imgui context. Refer to examples app!");
  ImGuiWindowFlags window_flags = 0;
  int width, height;

  glfwGetFramebufferSize(window_ptr_, &width, &height);
  ImGui::SetNextWindowPos(layout_.get_position(width, height, widget_name.c_str()));
  ImGui::SetNextWindowSize(layout_.get_size(width, height, widget_name.c_str()));
  if (!ImGui::Begin(widget_name.c_str(), &name_to_show_flag_and_widget[widget_name].first, window_flags))
  {ImGui::End();return;}
  // show registered widget
  name_to_show_flag_and_widget[widget_name].second(robot_system_ptr_);
  ImGui::End();
}

void gui::draw_windows()
{
  static ImVec4 clear_color(0.11f, 0.11f, 0.11f, 1.00f);
  glfwPollEvents();

  // Start dear imgui
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // write gui widget
  for (auto &entry : name_to_show_flag_and_widget)
    if (entry.second.first)
      show_widgets(entry.first);
  // debug

  // Rendering
  ImGui::Render();
  int display_w, display_h;

  /*If you do not draw manipulator, you should release comment out of following 3 sentences.*/
  glViewport(0, 0, display_w, display_h);
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  glfwGetFramebufferSize(window_ptr_, &display_w, &display_h);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_ptr_);
}

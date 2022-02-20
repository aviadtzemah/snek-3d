

#include <chrono>
#include <thread>

#include "../gl.h"
#include "Display.h"

#include "igl/igl_inline.h"
#include <igl/get_seconds.h>
#include "igl/opengl/glfw/renderer.h"

#include <glad/glad.h>
#include <external/stb_image.h>

#include <external/glm/glm.hpp>
#include <external/glm/gtc/matrix_transform.hpp>
#include <external/glm/gtc/type_ptr.hpp>

#include <external/learnopengl/filesystem.h>
#include <external/learnopengl/shader_m.h>
#include <external/learnopengl/camera.h>
#include <external/learnopengl/model.h>

#include <iostream>
#include <tutorial/sandBox/sandBox.h>

#include <igl/PI.h>

//void framebuffer_size_callback(GLFWwindow* window, int width, int height);
//void mouse_callback(GLFWwindow* window, double xpos, double ypos);
////void mouse_move(GLFWwindow* window, double x, double y);
//void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier);

//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
//void processInput(GLFWwindow* window, int key, int scancode, int action, int modifier);
unsigned int loadTexture(const char* path);
unsigned int loadCubemap(vector<std::string> faces);

// settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = (float)SCR_WIDTH / 2.0;
float lastY = (float)SCR_HEIGHT / 2.0;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}



Display::Display(int windowWidth, int windowHeight, const std::string& title)
{
	bool resizable = true, fullscreen = false;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			exit(EXIT_FAILURE);
		}
		glfwWindowHint(GLFW_SAMPLES, 8);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		
//#ifdef __APPLE__
//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//#endif
//		if (fullscreen)
//		{
//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
//			windowWidth = mode->width;
//			windowHeight = mode->height;
//		}
//		else
//		{
			// Set default windows width
			//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
			//	windowWidth = renderer->core().viewport[2];
			//else 
			//	if (windowWidth <= 0)
			//	windowWidth = 1280;
			//// Set default windows height
			//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
			//	windowHeight = renderer->core().viewport[3];
			//else if (windowHeight <= 0)
			//	windowHeight = 800;
//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
//		}
		window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
		if (!window)
		{
			glfwTerminate();
			exit(EXIT_FAILURE);
		}
		glfwMakeContextCurrent(window);
		// Load OpenGL and its extensions
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			printf("Failed to load OpenGL and its extensions\n");
			exit(EXIT_FAILURE);
		}
//#if defined(DEBUG) || defined(_DEBUG)
//		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
//		int major, minor, rev;
//		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
//		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
//		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
//		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
//		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
//		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
//#endif
		
		//i added

		//glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
		//glfwSetCursorPosCallback(window, mouse_callback);
		//glfwSetScrollCallback(window, scroll_callback);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetKeyCallback(window, processInput);

		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

		//Tamir: changes from here
		// Initialize FormScreen
	   // __viewer = this;
		// Register callbacks
		//glfwSetKeyCallback(window, glfw_key_callback);
		//glfwSetCursorPosCallback(window,glfw_mouse_move);
		//glfwSetScrollCallback(window, glfw_mouse_scroll);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetWindowSizeCallback(window,glfw_window_size);
	
		
		//glfwSetCharModsCallback(window,glfw_char_mods_callback);
		//glfwSetDropCallback(window,glfw_drop_callback);
		// Handle retina displays (windows and mac)
		//int width, height;
		//glfwGetFramebufferSize(window, &width, &height);
		//int width_window, height_window;
		//glfwGetWindowSize(window, &width_window, &height_window);
		//highdpi = windowWidth/width_window;
		
		//glfw_window_size(window,width_window,height_window);
		//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();
		
}
bool level2_once = false;
bool level3_once = false;
bool Display::launch_rendering(bool loop)
{
	glEnable(GL_DEPTH_TEST);

	// build and compile shaders
	// -------------------------
	Shader skyboxShader("C:/AnimationProject/snek-3d/6.1.skybox.vs", "C:/AnimationProject/snek-3d/6.1.skybox.fs");
	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};

	// skybox VAO
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// load textures
	// -------------
	vector<std::string> faces
	{
		FileSystem::getPath("tutorial/textures/Daylight_Box_Right.bmp"),
		FileSystem::getPath("tutorial/textures/Daylight_Box_Left.bmp"),
		FileSystem::getPath("tutorial/textures/Daylight_Box_Top.bmp"),
		FileSystem::getPath("tutorial/textures/nick.bmp"),
		FileSystem::getPath("tutorial/textures/Daylight_Box_Front.bmp"),
		FileSystem::getPath("tutorial/textures/Daylight_Box_Back.bmp")
	};
	unsigned int cubemapTexture = loadCubemap(faces);

	//// shader configuration
	//// --------------------

	skyboxShader.use();
	skyboxShader.setInt("skybox", 0);

	// glfwMakeContextCurrent(window);
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	for(int i=0;i< renderer->GetScene()->data_list.size();i++)
		renderer->core().toggle(renderer->GetScene()->data_list[i].show_lines);
	while (!glfwWindowShouldClose(window))
	{
		double tic = igl::get_seconds();
		renderer->Animate();
		if (renderer->menu)
		{
			renderer->menu->pre_draw();
			renderer->menu->callback_draw_viewer_menu();
		}
		renderer->draw(window);

		if (renderer->GetScene()->level == 3 && tic - renderer->GetScene()->start_game_time >= 16) {
			if (!level3_once) {
				renderer->GetScene()->velocity *= 3;
				renderer->GetScene()->bez_dis = 0.1;
				level3_once = true;
			}
		}
		if (renderer->GetScene()->level == 2 && tic - renderer->GetScene()->start_game_time >= 22) {
			if (!level2_once) {
				renderer->GetScene()->velocity *= 1.8;
				renderer->GetScene()->bez_dis = 0.05;
				level2_once = true;
			}
		}
		if (renderer->GetScene()->start_game_time > 0 && (tic - renderer->GetScene()->start_game_time > 45
			|| renderer->GetScene()->score >= renderer->GetScene()->req_score)) {
			renderer->GetScene()->isActive = false;
			renderer->GetScene()->start_game_time = 0;
			renderer->GetScene()->game_ended = true;
			renderer->GetScene()->bez_dis = 0.01;
			level2_once = false;
			level3_once = false;
		}

		if (renderer->GetScene()->camera_setting == 0){ //3rd person
			camera.Pitch = 0.0f;

			Eigen::Vector3f pivot(0, 0, -1.6);
			Eigen::Vector3f current = renderer->GetScene()->camera_direction;
            double angle = pivot.dot(current) / (pivot.norm() * current.norm()); // currently holds cos(angle)

            if (angle <= 1 && angle >= -1) {
                angle = acos(angle) * 180.0 / igl::PI;;
            }
            else {
                angle = angle > 1 ? acos(1) * 180.0 / igl::PI : acos(-1);
                angle = angle > 1 ? acos(1) * 180.0 / igl::PI : acos(-1) ;
            }
			camera.Yaw = -90.0f + angle;

			camera.updateCameraVectors();
		}
		else{ // top
			camera.Yaw = -90.0f;
			camera.Pitch = -90.0f;
			camera.updateCameraVectors();
		}

		//start
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		//// input
		//// -----
		////processInput(window);

		//// render
		//// ------
		//// these lines clear the screen completely
		//glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// draw scene as normal
		glm::mat4 model = glm::mat4(1.0f);
		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 300.0f);
		//eigen equal
		//Eigen::Matrix4f view = Eigen::Matrix4f::Identity();;
		//igl::look_at(renderer->core().camera_eye, renderer->core().camera_center, renderer->core().camera_up, view);

		// draw skybox as last
		glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
		skyboxShader.use();
		view = glm::mat4(glm::mat3(camera.GetViewMatrix())); // remove translation from the view matrix
		skyboxShader.setMat4("view", view);
		skyboxShader.setMat4("projection", projection);
		// skybox cube
		glBindVertexArray(skyboxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		glDepthFunc(GL_LESS); // set depth function back to default
		//end

		if (renderer->menu)
		{
			renderer->menu->post_draw();
		}
		glfwSwapBuffers(window);
		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion
			glfwPollEvents();

			
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwPollEvents();
			frame_counter = 0;
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif

	}
	glDeleteVertexArrays(1, &skyboxVAO);
	glDeleteBuffers(1, &skyboxVBO);

	return EXIT_SUCCESS;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
//void processInput(GLFWwindow* window, int key, int scancode, int action, int modifier)
//{
//	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
//	Eigen::Vector3d tmp;
//	SandBox* scn = (SandBox*)rndr->GetScene();
//	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
//		glfwSetWindowShouldClose(window, GL_TRUE);
//
//	else if (action == GLFW_PRESS || action == GLFW_REPEAT)
//		switch (key)
//		{
//		case 'w':
//		case 'W':
//			//scn->direction = 1;
//			//rndr->RotateCamera(0, 90 * igl::PI / 180);
//			rndr->core().camera_translation = Eigen::Vector3f(0, 1, 0);
//			//rndr->TranslateCamera(Eigen::Vector3f(-0.01f, 0, 0));
//
//			//scn->RotateInSystem(Eigen::Vector3d(0, 0, 1), 180 * igl::PI / 180);
//			camera.ProcessKeyboard(FORWARD, deltaTime);
//			break;
//		case 's':
//		case 'S':
//			scn->direction = 0;
//			camera.ProcessKeyboard(BACKWARD, deltaTime);
//			std::cout << "delta: " << deltaTime << std::endl;
//			break;
//		case 'a':
//		case 'A':
//			scn->direction = 3;
//			rndr->TranslateCamera(Eigen::Vector3f(-0.01f, 0, 0));
//			camera.ProcessKeyboard(LEFT, deltaTime);
//			break;
//		case 'd':
//		case 'D':
//			scn->direction = 2;
//			camera.ProcessKeyboard(RIGHT, deltaTime);			
//			break;
//		default:
//			Eigen::Vector3f shift;
//			float scale;
//			rndr->core().get_scale_and_shift_to_fit_mesh(scn->data().V, scn->data().F, scale, shift);
//
//			std::cout << "near " << rndr->core().camera_dnear << std::endl;
//			std::cout << "far " << rndr->core().camera_dfar << std::endl;
//			std::cout << "angle " << rndr->core().camera_view_angle << std::endl;
//			std::cout << "base_zoom " << rndr->core().camera_base_zoom << std::endl;
//			std::cout << "zoom " << rndr->core().camera_zoom << std::endl;
//			std::cout << "shift " << shift << std::endl;
//			std::cout << "translate " << rndr->core().camera_translation << std::endl;
//
//			break;//do nothing
//		}
//	
//	//if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//	//	glfwSetWindowShouldClose(window, true);
//
//	//if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
//	//	camera.ProcessKeyboard(FORWARD, deltaTime);
//	//if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
//	//	camera.ProcessKeyboard(BACKWARD, deltaTime);
//	//if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
//	//	camera.ProcessKeyboard(LEFT, deltaTime);
//	//if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
//	//	camera.ProcessKeyboard(RIGHT, deltaTime);
//}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
//void framebuffer_size_callback(GLFWwindow* window, int width, int height)
//{
//	// make sure the viewport matches the new window dimensions; note that width and 
//	// height will be significantly larger than specified on retina displays.
//	glViewport(0, 0, width, height);
//}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
//void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
//{
//	float xpos = static_cast<float>(xposIn);
//	float ypos = static_cast<float>(yposIn);
//	if (firstMouse)
//	{
//		lastX = xpos;
//		lastY = ypos;
//		firstMouse = false;
//	}
//
//	float xoffset = xpos - lastX;
//	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//	lastX = xpos;
//	lastY = ypos;
//
//	camera.ProcessMouseMovement(xoffset, yoffset);
//}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
//static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
//{
//	camera.ProcessMouseScroll(static_cast<float>(yoffset));
//}

//void mouse_move(GLFWwindow* window, double x, double y)
//{
//	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
//	rndr->UpdatePosition(x, y);
//	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
//	{
//		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
//	}
//	else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
//	{
//		rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
//	}
//}

void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer* scn = rndr->GetScene();

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);


		double depth, closestZ = 1;
		int i = 0, savedIndx = scn->selected_data_index, lastIndx = scn->selected_data_index;

		for (; i < scn->data_list.size(); i++)
		{
			scn->selected_data_index = i;
			depth = rndr->Picking(x2, y2);
			if (depth < 0 && (closestZ > 0 || closestZ < depth))
			{
				savedIndx = i;
				closestZ = depth;
				std::cout << "found " << depth << std::endl;
			}
		}
		scn->selected_data_index = savedIndx;
		//scn->data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
		//if (lastIndx != savedIndx)
			//scn->data_list[lastIndx].set_colors(Eigen::RowVector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0));

		rndr->UpdatePosition(x2, y2);

	}
	else
	{
		rndr->GetScene()->isPicked = false;

	}
}
// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const* path)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int width, height, nrComponents;
	unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
}

// loads a cubemap texture from 6 individual texture faces
// order:
// +X (right)
// -X (left)
// +Y (top)
// -Y (bottom)
// +Z (front) 
// -Z (back)
// -------------------------------------------------------
unsigned int loadCubemap(std::vector<std::string> faces)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}

void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);//{
}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	glfwSetWindowUserPointer(window, userPointer);
}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}


Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

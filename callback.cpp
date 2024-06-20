#include "callback.h"

void glfwErrorCallback(int error, const char* description)
{
	fprintf(stderr, "GLFW error occured. Code: %d. Description: %s\n", error, description);
}


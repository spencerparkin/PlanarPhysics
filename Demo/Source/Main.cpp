#include "Main.h"
#include "App.h"

int main()
{
	App app;

	if (!app.Setup())
		return -1;

	int exitCode = app.Run();

	app.Shutdown();

	return exitCode;
}
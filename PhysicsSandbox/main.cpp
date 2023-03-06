#include "pch.h"


#define RUN_TEST_MAIN 0
#define MEM_DEBUG     0

#if MEM_DEBUG
#define _CRT_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif

// Usually, just run the sandbox app
#if not(RUN_TEST_MAIN)

#include "PhysicsSandboxApp.h"

int main(int argc, char* argv[])
{
#if MEM_DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    drb::PhysicsSandboxApp app{};

    if (not app.Init(argc, argv)) {
        return 1;
    }

    app.Run();


    return 0;
}


#else

// Run some test code

// INSERT TEST INCLUDES HERE
// ...


int main(int argc, char* argv[])
{
#if MEM_DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

	// INSERT TEST CODE HERE
    // ...

    return 0;
}

#endif
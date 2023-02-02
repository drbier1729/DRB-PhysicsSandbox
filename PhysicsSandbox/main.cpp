#include "pch.h"
#include "PhysicsSandboxApp.h"


int main(int argc, char* argv[])
{
    drb::PhysicsSandboxApp app{};

    if (not app.Init(argc, argv)) {
        return 1;
    }

    app.Run();


    return 0;
}
